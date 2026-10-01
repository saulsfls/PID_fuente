"""
CONTROL DE FP v10 - PROBE + PI continuo + slope híbrido + stats robustas
=========================================================================

Diagnóstico v9.9:
  - Slope medido en PROBE ≈ 0.005 (200x menor que -0.8 asumido).
  - KI_F=0.0018 no compensa drift → offset -0.20 en FP.
  - PASO_BUSCAR=3° mueve PHI solo 0.015 → BUSCAR inútil.
  - PHI_STEP_ABS_MAX=0.40° ≈ 0.002 PHI → demasiado lento.
  - Solo 49.6% tiempo en ±3%.

Cambios v10:
  1. Slope HÍBRIDO: promedio(medido, teórico sqrt(2·PHI)·π/180).
  2. Paso de fase ADAPTATIVO según |PHI| (0.03° a 8°).
  3. KI_F x5, DF_SLEW x3, DF_MAX x2 → integrador útil.
  4. BUSCAR real, con dirección desde slope_sign y paso proporcional a |PHI|.
  5. Deadband adaptativa al slope efectivo.
  6. Módulo estadisticas.py: métricas de estabilidad dentro del rango,
     rachas, excursiones, export a Excel.
  7. Al inicio pregunta si recopilar estadísticas; al final, export.
"""
import time
import numpy as np
from collections import deque
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

from pfv2.estadisticas import (EstadisticasRobustas, preguntar_si_no,
                          preguntar_texto)

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================
DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.05

FP_RANGO_3PCT = 0.03
FP_MIN_RANGO = 0.970
FP_MAX_RANGO = 1.030

AMPLITUD_FG = 5.0
OFFSET_V_FG = 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# ---------- PROBE (identificación de slope) ----------
PROBE_STEP_DEG = 2.0
PROBE_SETTLE_SAMPLES = 6
PROBE_CYCLES = 3
PROBE_MIN_DPHI = 0.0003
PROBE_MAX_TIME = 10.0

# ---------- PLL ----------
KP_PHI = 0.60
KI_F   = 0.010
DF_MAX = 0.10
DF_SLEW = 0.020
DF_LP  = 0.25
EFF_DT_MAX = 0.5

# Paso máximo de fase por muestra — AHORA ADAPTATIVO
PHI_STEP_HARD_MAX = 8.0
PHI_STEP_ABS_MIN = 0.005

# Deadband (adaptativa al slope)
PHI_DEADBAND_BASE = 0.0010
PHI_DEADBAND_MIN = 0.0004

# ---------- BUSCAR ----------
PHI_BUSCAR_ENTER = 0.10
PHI_BUSCAR_EXIT  = 0.04
N_FRESH_ENTER_BUSCAR = 3
N_FRESH_EXIT_BUSCAR  = 2
PASO_BUSCAR_MIN     = 1.0
PASO_BUSCAR_MAX     = 10.0
N_SETTLE_BUSCAR = 4

# ---------- Outliers / stale ----------
STALE_TOL = 0.02
OUTLIER_JUMP = 0.15
N_OUTLIERS_RESET = 3

# ---------- Anti-estancamiento ----------
N_STUCK_FRESH = 80
STUCK_EPS = 0.001

PHI_LOCKED = 0.02


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def sign_safe(x, default=1.0):
    if x is None or abs(x) < 1e-12:
        return default
    return 1.0 if x > 0 else -1.0


# ==============================================================================
# RUNTRACKER
# ==============================================================================
class RunTracker:
    def __init__(self, name=""):
        self.name = name
        self.cur_samples = 0
        self.cur_time = 0.0
        self.max_time = 0.0
        self.max_samples = 0
        self.total_time_in = 0.0
        self.total_time = 0.0
        self.run_times = []

    def update(self, cond, dt):
        self.total_time += dt
        if cond:
            self.cur_samples += 1
            self.cur_time += dt
            self.total_time_in += dt
            if self.cur_time > self.max_time:
                self.max_time = self.cur_time
            if self.cur_samples > self.max_samples:
                self.max_samples = self.cur_samples
        else:
            self._close()

    def _close(self):
        if self.cur_samples > 0:
            self.run_times.append(self.cur_time)
            self.cur_samples = 0
            self.cur_time = 0.0

    def close(self):
        self._close()

    def summary(self):
        n = len(self.run_times)
        pct = 100 * self.total_time_in / max(self.total_time, 1e-6)
        if n == 0:
            return {"n": 0, "max_time": 0.0, "max_samples": 0,
                    "mean_time": 0.0, "median_time": 0.0,
                    "pct": pct, "total_time_in": self.total_time_in}
        return {"n": n, "max_time": self.max_time,
                "max_samples": self.max_samples,
                "mean_time": float(np.mean(self.run_times)),
                "median_time": float(np.median(self.run_times)),
                "pct": pct, "total_time_in": self.total_time_in}


# ==============================================================================
# ESTADÍSTICAS (resumen estándar)
# ==============================================================================
class Estadisticas:
    def __init__(self):
        self.fp_hist = []
        self.phi_hist = []
        self.fase_hist = []
        self.df_hist = []
        self.dphi_hist = []
        self.modo_hist = []
        self.tiempos_modo = {}
        self.n_fresh = 0
        self.n_stale = 0
        self.n_outliers = 0
        self.n_trans = 0
        self.n_stuck = 0
        self.n_lock_lost = 0
        self.t_total = 0.0

        self.t_band = {0.01: 0.0, 0.02: 0.0, 0.03: 0.0, 0.05: 0.0}

        self.run_3pct = RunTracker("|FP-1| <= 0.03")
        self.run_2pct = RunTracker("|FP-1| <= 0.02")
        self.run_1pct = RunTracker("|FP-1| <= 0.01")

        self.t_first_3pct = None
        self.t_first_lock = None

        self.n_excursions = 0
        self.excursions = []
        self._exc_t0 = 0.0
        self._exc_clock = 0.0
        self._exc_phi_max = 0.0
        self._exc_active = False

    def registrar(self, fp, phi, fase, df, dphi, modo, dt):
        self.fp_hist.append(fp)
        self.phi_hist.append(phi)
        self.fase_hist.append(fase)
        self.df_hist.append(df)
        if dphi is not None:
            self.dphi_hist.append(dphi)
        self.modo_hist.append(modo)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        self.t_total += dt
        self._exc_clock += dt

        err = abs(fp - 1.0)
        for k in self.t_band:
            if err <= k:
                self.t_band[k] += dt

        self.run_3pct.update(err <= 0.03, dt)
        self.run_2pct.update(err <= 0.02, dt)
        self.run_1pct.update(err <= 0.01, dt)

        if err > 0.03:
            if not self._exc_active:
                self._exc_active = True
                self._exc_t0 = self._exc_clock
                self._exc_phi_max = abs(phi)
                self.n_excursions += 1
            else:
                self._exc_phi_max = max(self._exc_phi_max, abs(phi))
        else:
            if self._exc_active:
                dur = self._exc_clock - self._exc_t0
                self.excursions.append((self._exc_t0, dur, self._exc_phi_max))
                self._exc_active = False

    def _imprimir_rachas(self, tr):
        s = tr.summary()
        print(f"  ▸ {tr.name}")
        print(f"      Nº rachas:     {s['n']}")
        if s['n'] > 0:
            print(f"      Máx duración:  {s['max_time']:.2f} s ({s['max_samples']} m)")
            print(f"      Media:         {s['mean_time']:.2f} s")
            print(f"      Mediana:       {s['median_time']:.2f} s")
        print(f"      Tiempo total:  {s['total_time_in']:.1f} s ({s['pct']:.1f}%)")

    def resumen(self, controlador, t_total, n_iter, en_rango):
        self.run_3pct.close()
        self.run_2pct.close()
        self.run_1pct.close()
        if self._exc_active:
            dur = self._exc_clock - self._exc_t0
            self.excursions.append((self._exc_t0, dur, self._exc_phi_max))

        print("\n" + "=" * 90)
        print(" RESUMEN FINAL v10 — PROBE + PI CONTINUO + BUSCAR ADAPTATIVO")
        print("=" * 90)

        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:        {t_total:.1f} s")
        print(f"  Iteraciones:         {n_iter}")
        print(f"  Tasa:                {n_iter/max(t_total,1e-6):.2f} Hz")
        print(f"  Frescas:             {self.n_fresh}")
        print(f"  Stale:               {self.n_stale}")
        print(f"  Outliers:            {self.n_outliers}")
        print(f"  Estancamientos:      {self.n_stuck}")
        print(f"  Pérdidas lock:       {self.n_lock_lost}")

        print(f"\n[ PROBE — Identificación de slope ]")
        if controlador.slope_est is not None:
            print(f"  Slope estimado:      {controlador.slope_est:+.6f} (PHI/deg)")
            print(f"  Signo:               {controlador.slope_sign:+.0f}")
            print(f"  Confianza:           {'CONFIRMADO' if controlador.slope_confirmed else 'baja'}")
            print(f"  Muestras probe:      {controlador.probe_n_samples}")
            if controlador.probe_raw:
                print(f"  Mediciones crudas:")
                for i, (dphi, dphi_meas) in enumerate(controlador.probe_raw):
                    print(f"    #{i+1}: Δφ={dphi:+.2f}° → ΔPHI={dphi_meas:+.5f}")
        else:
            print(f"  PROBE no completado")

        if self.fp_hist:
            fp_abs = np.abs(self.fp_hist)
            print(f"\n[ Control FP ]")
            print(f"  FP media:            {np.mean(fp_abs):.4f}")
            print(f"  FP std:              {np.std(fp_abs):.4f}")
            print(f"  Sesgo (|FP|-1):      {np.mean(fp_abs)-1.0:+.4f}")
            print(f"  |FP-1| mediana:      {np.median(np.abs(fp_abs-1.0)):.4f}")
            print(f"  |FP-1| p90:          {np.percentile(np.abs(fp_abs-1.0),90):.4f}")
            print(f"  |FP-1| p99:          {np.percentile(np.abs(fp_abs-1.0),99):.4f}")

        print(f"\n[ OBJETIVO ±0.03 (3%) ]")
        p01 = 100 * self.t_band[0.01] / max(self.t_total, 1e-6)
        p02 = 100 * self.t_band[0.02] / max(self.t_total, 1e-6)
        p03 = 100 * self.t_band[0.03] / max(self.t_total, 1e-6)
        p05 = 100 * self.t_band[0.05] / max(self.t_total, 1e-6)
        print(f"  ±0.01:  {p01:5.1f}%")
        print(f"  ±0.02:  {p02:5.1f}%")
        print(f"  ±0.03:  {p03:5.1f}%  ← OBJETIVO ≥95%")
        print(f"  ±0.05:  {p05:5.1f}%")

        print(f"\n[ Excursiones fuera de ±3% ]")
        print(f"  Total:               {self.n_excursions}")
        if self.excursions:
            durs = [e[1] for e in self.excursions]
            phis = [e[2] for e in self.excursions]
            print(f"  Duración media:      {np.mean(durs):.3f} s")
            print(f"  Duración mediana:    {np.median(durs):.3f} s")
            print(f"  Duración máx:        {np.max(durs):.3f} s")
            print(f"  |PHI|max media:      {np.mean(phis):.4f}")
            print(f"  |PHI|max máx:        {np.max(phis):.4f}")
            n_long = sum(1 for d in durs if d > 2.0)
            print(f"  Excursiones >2s:     {n_long}")

        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            print(f"  {m:<12} {t:7.1f}s ({pct:5.1f}%)")

        print(f"\n[ Rachas ]")
        self._imprimir_rachas(self.run_3pct)
        self._imprimir_rachas(self.run_2pct)
        self._imprimir_rachas(self.run_1pct)

        print(f"\n[ Trazabilidad ]")
        print(f"  Mejor FP:            {controlador.mejor_fp:.5f}")
        print(f"  Mejor |PHI|:         {controlador.mejor_phi_abs:.5f}")
        if self.t_first_3pct is not None:
            print(f"  1er ±3%:             {self.t_first_3pct:.2f} s")
        if self.t_first_lock is not None:
            print(f"  1er lock:            {self.t_first_lock:.2f} s")

        if self.df_hist:
            print(f"\n[ Δf ]")
            print(f"  Media:               {np.mean(self.df_hist):+.5f} Hz")
            print(f"  Std:                 {np.std(self.df_hist):.5f} Hz")
            print(f"  Max abs:             {np.max(np.abs(self.df_hist)):.5f} Hz")

        if self.dphi_hist:
            nz = [abs(x) for x in self.dphi_hist if abs(x) > 1e-4]
            if nz:
                print(f"\n[ dφ ]")
                print(f"  Media:               {np.mean(nz):.4f}°")
                print(f"  Max:                 {np.max(nz):.4f}°")

        print(f"\n[ VEREDICTO ]")
        if p03 >= 95:
            print(f"  ✓✓ OBJETIVO CUMPLIDO  (±0.01:{p01:.1f}% ±0.02:{p02:.1f}% ±0.03:{p03:.1f}%)")
        elif p03 >= 90:
            print(f"  ✓ CASI  (±0.01:{p01:.1f}% ±0.02:{p02:.1f}% ±0.03:{p03:.1f}%)")
        else:
            print(f"  ✗ INSUFICIENTE  (±0.01:{p01:.1f}% ±0.02:{p02:.1f}% ±0.03:{p03:.1f}%)")
        print("=" * 90)


# ==============================================================================
# CONTROLADOR v10
# ==============================================================================
class ControladorFPv10:
    def __init__(self):
        # Actuadores
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.df_integral = 0.0

        # Estado
        self.modo = "PROBE"   # PROBE → PLL/BUSCAR
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False

        # PHI
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.phi_stale = True
        self.stale_run = 0
        self.dt_since_fresh = 0.0

        # Slope (aprendido en PROBE)
        self.slope_est = None
        self.slope_sign = 0.0
        self.slope_confirmed = False
        self.probe_raw = []
        self.probe_n_samples = 0

        # Probe state machine
        self.probe_state = 0
        self.probe_counter = 0
        self.probe_phase_before = 0.0
        self.probe_phi_before = 0.0
        self.probe_phi_a = 0.0
        self.probe_current_cycle = 0

        # PLL
        self.ki_eff = KI_F
        self.kp_eff = KP_PHI
        self.deadband = PHI_DEADBAND_BASE

        # Fase y phi previos
        self.fase_prev = None
        self.phi_prev = None

        # Histéresis BUSCAR
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

        # Mejor
        self.mejor_phi_abs = 999.0
        self.mejor_fp = 0.0
        self.mejor_fase = 0.0

        # Anti-estancamiento
        self.best_phi_recent = 999.0
        self.cnt_since_improve = 0

        # Outliers
        self.outliers_consec = 0

        # dt
        self.dt_recent = deque(maxlen=20)

    # ------------------------------------------------------------------
    def _resp(self, accion, f_fg, cambio_fase=False, fresh=False, dphi=0.0):
        return {
            "accion": accion,
            "fase": self.fase_cmd,
            "delta_f": self.delta_f,
            "frecuencia": f_fg,
            "cambio_fase": cambio_fase,
            "modo": self.modo,
            "phi": self.phi_fresh,
            "fresh": fresh,
            "stale": self.phi_stale,
            "mejor_fp": self.mejor_fp,
            "mejor_phi_abs": self.mejor_phi_abs,
            "slope": self.slope_est if self.slope_est else 0.0,
            "dphi": dphi,
            "df_integral": self.df_integral,
        }

    def _aplicar_fase(self, delta):
        if abs(delta) < 1e-4:
            return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = envolver_fase(self.fase_cmd + delta)
        self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
        self.move_pending = True
        self.fresh_after_move = False
        return True

    def _actualizar_phi(self, phi_raw, stats=None):
        phi_w = phi_raw
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            return True
        if abs(phi_w - self.phi_prev_raw) < STALE_TOL:
            self.stale_run += 1
            self.phi_stale = True
            return False
        jump = abs(phi_w - self.phi_fresh)
        if jump > OUTLIER_JUMP:
            if stats is not None:
                stats.n_outliers += 1
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.outliers_consec += 1
            return False
        self.outliers_consec = 0
        self.phi_fresh = phi_w
        self.phi_prev_raw = phi_w
        self.phi_stale = False
        self.stale_run = 0
        if self.move_pending:
            self.fresh_after_move = True
        return True

    # ------------------------------------------------------------------
    # Slope helpers
    # ------------------------------------------------------------------
    def _slope_teorico(self, phi):
        """
        Modelo físico: PHI ≈ 1 - cos(φ)  ⇒  dPHI/dφ_deg ≈ sin(φ_rad)·π/180
        Para φ pequeño: dPHI/dφ_deg ≈ sqrt(2·PHI)·π/180
        """
        phi = abs(phi)
        if phi < 1e-9:
            return 1e-6
        return np.sqrt(2.0 * phi) * np.pi / 180.0

    def _max_step_adaptativo(self, aphi):
        """Paso máximo en grados de φ según |PHI|."""
        if aphi > 0.10:   return 8.0
        if aphi > 0.05:   return 4.0
        if aphi > 0.02:   return 1.5
        if aphi > 0.008:  return 0.4
        if aphi > 0.003:  return 0.10
        return 0.03

    def _slope_efectivo(self, phi):
        """
        Combina slope medido en PROBE con slope teórico.
        Si difieren mucho, prioriza el teórico.
        """
        slope_t = self._slope_teorico(phi)
        if self.slope_confirmed and self.slope_est is not None:
            if abs(self.slope_est) < 3.0 * slope_t + 1e-6:
                return 0.5 * abs(self.slope_est) + 0.5 * slope_t
        return slope_t

    # ------------------------------------------------------------------
    # PROBE
    # ------------------------------------------------------------------
    def _probe_step(self, fresh, stats):
        """Retorna True cuando termina el probe."""
        if not fresh:
            self.probe_counter += 1
            return False

        if self.probe_state == 0:
            self.probe_phase_before = self.fase_cmd
            self.probe_phi_before = self.phi_fresh
            self.probe_state = 1
            self.probe_counter = 0
            return False

        elif self.probe_state == 1:
            self._aplicar_fase(+PROBE_STEP_DEG)
            self.probe_state = 2
            self.probe_counter = 0
            return False

        elif self.probe_state == 2:
            if self.probe_counter >= PROBE_SETTLE_SAMPLES:
                self.probe_phi_a = self.phi_fresh
                self.probe_state = 4
                self.probe_counter = 0
            else:
                self.probe_counter += 1
            return False

        elif self.probe_state == 4:
            self._aplicar_fase(-PROBE_STEP_DEG)
            self.probe_state = 5
            self.probe_counter = 0
            return False

        elif self.probe_state == 5:
            if self.probe_counter >= PROBE_SETTLE_SAMPLES:
                phi_b = self.phi_fresh
                dphi_total = self.probe_phi_a - phi_b
                dphi_deg = 2 * PROBE_STEP_DEG
                if abs(dphi_total) > PROBE_MIN_DPHI:
                    self.probe_raw.append((dphi_deg, dphi_total))
                    self.probe_n_samples += 1
                self.probe_current_cycle += 1

                if self.probe_current_cycle >= PROBE_CYCLES:
                    self.probe_state = 7
                else:
                    self.probe_phi_before = self.phi_fresh
                    self.probe_state = 0
                    self.probe_counter = 0
            else:
                self.probe_counter += 1
            return False

        elif self.probe_state == 7:
            if self.probe_raw:
                total_dphi = sum(m[0] for m in self.probe_raw)
                total_dphi_meas = sum(m[1] for m in self.probe_raw)
                self.slope_est = total_dphi_meas / total_dphi
                self.slope_sign = sign_safe(self.slope_est, default=1.0)
                if abs(self.slope_est) > 1e-5:
                    self.slope_confirmed = True
            else:
                self.slope_est = 0.005
                self.slope_sign = 1.0
            self.probe_state = 8
            return False

        elif self.probe_state == 8:
            # Transición: si PHI es grande, entramos en BUSCAR
            if abs(self.phi_fresh) > PHI_BUSCAR_ENTER:
                self.modo = "BUSCAR"
                self.fresh_enter_buscar = N_FRESH_ENTER_BUSCAR
            else:
                self.modo = "PLL"
            if stats is not None:
                stats.n_trans += 1
            self.cnt = 0
            self.move_pending = False
            self.best_phi_recent = abs(self.phi_fresh)
            self.cnt_since_improve = 0
            return True

        return False

    # ------------------------------------------------------------------
    def _buscar(self, fresh):
        """
        BUSCAR: paso grande en la dirección correcta cuando |PHI| es grande.
        """
        if not fresh:
            f_fg = FREC_NOMINAL + self.delta_f
            return self._resp(f"BUSCAR-stale", f_fg)

        aphi = abs(self.phi_fresh)

        # Paso proporcional a |PHI| pero acotado
        paso = clamp(aphi * 15.0, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX)

        # Dirección: usamos slope_sign si está confirmado
        if self.slope_confirmed:
            direccion = -self.slope_sign
        else:
            direccion = -1.0 if self.phi_fresh > 0 else 1.0

        dphi = direccion * paso
        cambio = self._aplicar_fase(dphi)

        f_fg = FREC_NOMINAL + self.delta_f
        return self._resp(f"BUSCAR |PHI|={aphi:.3f} dφ={dphi:+.2f}",
                          f_fg, cambio_fase=cambio, fresh=True, dphi=dphi)

    # ------------------------------------------------------------------
    def actualizar(self, fp_med, phi_med, f_med, dt, stats=None, t_rel=None):
        if phi_med is None:
            return self._resp("SIN_PHI", FREC_NOMINAL + self.delta_f)

        self.dt_recent.append(dt)
        fresh = self._actualizar_phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh += dt
        if fresh:
            self.dt_since_fresh = 0.0
            if stats is not None:
                stats.n_fresh += 1
        elif stats is not None:
            stats.n_stale += 1

        fp_abs = abs(fp_med) if fp_med is not None else 1.0

        if fresh and abs(self.phi_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.phi_fresh)
            self.mejor_fase = self.fase_cmd
            self.mejor_fp = fp_abs

        # ---- MODO PROBE ----
        if self.modo == "PROBE":
            done = self._probe_step(fresh, stats)
            accion = f"PROBE st={self.probe_state}"
            if done:
                accion = "PROBE done"
            return self._resp(accion, FREC_NOMINAL, fresh=fresh)

        # ---- Histéresis BUSCAR ----
        if fresh:
            if abs(self.phi_fresh) > PHI_BUSCAR_ENTER:
                self.fresh_enter_buscar += 1
            else:
                self.fresh_enter_buscar = 0
            if abs(self.phi_fresh) < PHI_BUSCAR_EXIT:
                self.fresh_exit_buscar += 1
            else:
                self.fresh_exit_buscar = 0

        # Entrar a BUSCAR
        if (abs(self.phi_fresh) > PHI_BUSCAR_ENTER
                and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR
                and self.modo != "BUSCAR"):
            self.modo = "BUSCAR"
            if stats is not None:
                stats.n_trans += 1

        # Salir de BUSCAR
        if (self.modo == "BUSCAR"
                and abs(self.phi_fresh) < PHI_BUSCAR_EXIT
                and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR):
            self.modo = "PLL"
            self.cnt = 0
            if stats is not None:
                stats.n_trans += 1

        # Anti-estancamiento
        if fresh:
            if abs(self.phi_fresh) < self.best_phi_recent - STUCK_EPS:
                self.best_phi_recent = abs(self.phi_fresh)
                self.cnt_since_improve = 0
            else:
                self.cnt_since_improve += 1

        if self.modo == "BUSCAR":
            return self._buscar(fresh)

        return self._pll(fresh, dt)

    # ------------------------------------------------------------------
    def _pll(self, fresh, dt):
        """
        PLL v10:
          - Fase: dφ = -KP · PHI / slope_eff · dt   (paso adaptativo)
          - Frecuencia: integrador útil compensando drift
          - Deadband adaptativa al slope
        """
        if not fresh:
            f_fg = FREC_NOMINAL + self.delta_f
            return self._resp(f"PLL-stale df={self.delta_f:+.5f}", f_fg)

        eff_dt = min(self.dt_since_fresh if self.dt_since_fresh > 0 else dt,
                     EFF_DT_MAX)
        if eff_dt < 0.02:
            eff_dt = 0.02

        aphi = abs(self.phi_fresh)
        slope_eff_aprox = self._slope_efectivo(max(aphi, 1e-6))
        self.deadband = max(PHI_DEADBAND_MIN, 0.5 * slope_eff_aprox)

        phi_eff = self.phi_fresh
        if aphi < self.deadband:
            phi_eff = 0.0

        # ---------- FRECUENCIA ----------
        if self.slope_confirmed:
            if abs(phi_eff) > 0 and self.dt_since_fresh < 0.3:
                self.df_integral += -self.ki_eff * self.slope_sign * phi_eff * eff_dt
                self.df_integral = clamp(self.df_integral, -DF_MAX, DF_MAX)

        df_target = self.df_integral
        d = df_target - self.delta_f
        if abs(d) > DF_SLEW:
            d = np.sign(d) * DF_SLEW
        self.delta_f = clamp(self.delta_f + d * DF_LP, -DF_MAX, DF_MAX)

        # ---------- FASE ----------
        cambio_fase = False
        dphi = 0.0
        if self.slope_confirmed and abs(phi_eff) > 0:
            slope_use = max(self._slope_efectivo(phi_eff), 1e-6)
            dphi = -self.kp_eff * phi_eff / slope_use * eff_dt
            max_step = self._max_step_adaptativo(aphi)
            dphi = clamp(dphi, -max_step, max_step)
            dphi = clamp(dphi, -PHI_STEP_HARD_MAX, PHI_STEP_HARD_MAX)

            if abs(dphi) >= PHI_STEP_ABS_MIN:
                cambio_fase = self._aplicar_fase(dphi)
            else:
                dphi = 0.0

        f_fg = FREC_NOMINAL + self.delta_f
        accion = (f"PLL df={self.delta_f:+.5f} dφ={dphi:+.3f} "
                  f"s={self._slope_efectivo(aphi):.5f}")
        return self._resp(accion, f_fg, cambio_fase=cambio_fase,
                          fresh=True, dphi=dphi)


# ==============================================================================
# LOGGING
# ==============================================================================
def encabezado():
    print("-" * 150)
    print(f"{'t':>4} | {'FP':>7} | {'*':>1} | {'PHI':>8} | {'Fase':>8} | "
          f"{'Δf':>10} | {'Slope':>9} | {'dφ':>7} | {'Modo':>6} | {'Acción':>28}")
    print("-" * 150)
    print("  *  → |FP-1| ≤ 0.03")
    print("-" * 150)


def fila(t, fp_med, res):
    err_fp = abs(abs(fp_med) - 1.0)
    mark = "*" if err_fp <= FP_RANGO_3PCT else " "
    print(f"{t:>4} | {fp_med:>7.4f} | {mark:>1} | "
          f"{res['phi']:>+8.4f} | {res['fase']:>+8.2f} | "
          f"{res['delta_f']:>+10.5f} | {res['slope']:>+9.6f} | "
          f"{res.get('dphi', 0.0):>+7.3f} | {res['modo']:>6} | "
          f"{res['accion']:>28}")


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    print("=" * 150)
    print(" CONTROL FP v10 — PROBE + PI CONTINUO + BUSCAR ADAPTATIVO")
    print("=" * 150)
    print(f" PROBE: +{PROBE_STEP_DEG}°/-{PROBE_STEP_DEG}° × {PROBE_CYCLES} ciclos, "
          f"settle {PROBE_SETTLE_SAMPLES} muestras")
    print(f" PLL: KP_PHI={KP_PHI}  KI_F={KI_F}  paso adaptativo (max {PHI_STEP_HARD_MAX}°)")
    print(f" BUSCAR: |PHI|>{PHI_BUSCAR_ENTER} → paso 1–10°, dirección por slope_sign")
    print("=" * 150)

    # ---------- PREGUNTA INTERACTIVA DE ESTADÍSTICAS ----------
    print("\n" + "-" * 90)
    print(" CONFIGURACIÓN DE ESTADÍSTICAS")
    print("-" * 90)
    usar_stats = preguntar_si_no(
        "¿Recopilar estadísticas robustas (estabilidad dentro del rango)? [s/N]: ",
        default='n'
    )
    stats_robustas = None
    if usar_stats:
        stats_robustas = EstadisticasRobustas(
            fp_target=1.0, tolerancia=FP_RANGO_3PCT, tolerancia_fina=0.01
        )
        print("  → Se recopilarán estadísticas robustas.\n")
    else:
        print("  → Solo resumen estándar.\n")

    fg = None
    wt = None
    controlador = ControladorFPv10()
    stats = Estadisticas()

    total = 0
    en_rango = 0
    t_ctrl = None

    try:
        print("\n[1/3] Conectando equipos (modo EXTREME)...")
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()
        print(f"  FG420:  {fg.obtener_idn()[:60]}")
        print(f"  WT3000: {wt.obtener_idn()[:60]}")

        fg.extreme(
            canal=1, frecuencia_hz=FREC_NOMINAL,
            amplitud_vpp=AMPLITUD_FG, offset_v=OFFSET_V_FG,
            fase_grados=0.0, encender_salida=True,
        )
        wt.extreme(
            elemento_entrada=ELEMENTO_WT,
            incluir_potencias=True, configurar_salida=True,
        )
        print("  Modo EXTREME aplicado.")

        print("\n[2/3] Estabilizando 5 s...")
        for _ in range(10):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print(" OK")

        print("\n[3/3] INICIANDO CONTROL\n")
        encabezado()

        t_ctrl = time.time()
        t_prev = t_ctrl

        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            t_rel = t_now - t_ctrl

            try:
                m = wt.leer_mediciones_minimas()
            except Exception as e:
                print(f"[X] Error lectura: {e}")
                time.sleep(INTERVALO_MUESTREO)
                continue

            if wt.is_outlier():
                continue

            fp_med = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med = m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None \
                else abs(np.cos(np.radians(phi_med)))
            total += 1
            if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO:
                en_rango += 1

            res = controlador.actualizar(fp_med, phi_med, f_med, dt, stats,
                                          t_rel=t_rel)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["phi"], res["fase"],
                            res["delta_f"], res.get("dphi"),
                            res["modo"], dt)

            # ---- Estadísticas robustas ----
            if stats_robustas is not None:
                stats_robustas.agregar(t_rel, fp_abs, res.get("phi", 0.0),
                                       modo=res["modo"])

            err_fp = abs(fp_abs - 1.0)
            if stats.t_first_3pct is None and err_fp <= 0.03:
                stats.t_first_3pct = t_rel
            if stats.t_first_lock is None and abs(res["phi"]) < PHI_LOCKED:
                stats.t_first_lock = t_rel

            if res.get("fresh") or total % 8 == 0:
                fila(int(t_rel), fp_abs, res)

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        # ---- Resumen estándar ----
        stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)

        # ---- Estadísticas robustas + export ----
        if stats_robustas is not None:
            stats_robustas.cerrar(t_final=time.time() - t_ctrl)
            stats_robustas.imprimir()

            print()
            if preguntar_si_no("¿Exportar estadísticas a Excel? [s/N]: ",
                               default='n'):
                ruta = preguntar_texto(
                    "  Ruta destino (Enter = carpeta actual): ", default='.'
                )
                if not ruta:
                    ruta = '.'
                ts = time.strftime("%Y%m%d_%H%M%S")
                nombre = preguntar_texto(
                    f"  Nombre del archivo (Enter = 'estadisticas_FP_{ts}.xlsx'): ",
                    default=f"estadisticas_FP_{ts}.xlsx"
                )
                stats_robustas.exportar_excel(ruta, nombre)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)
            if stats_robustas is not None:
                stats_robustas.cerrar(t_final=time.time() - t_ctrl)
                stats_robustas.imprimir()

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[!] Apagando...")
        if fg is not None:
            try:
                fg.establecer_salida(1, False)
                fg.desconectar()
            except Exception:
                pass
        if wt is not None:
            try:
                wt.desconectar()
            except Exception:
                pass
        print("[✓] Listo.")


if __name__ == "__main__":
    main()