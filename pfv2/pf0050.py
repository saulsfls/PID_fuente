"""
CONTROL DE FACTOR DE POTENCIA v11.27-006
--------------------------------------------------------------------------------
Objetivo: FP = 0.0500  (φ_obj ≈ 87.1340°)
Instrumentación: Yokogawa FG420 + Yokogawa WT3000 (vía GPIB)

CAMBIOS v11.27-006:
  - FIX CRÍTICO: El PLL ahora respeta el signo de la pendiente (slope) incluso 
    si su magnitud cae por debajo de SLOPE_MIN_ABS, apoyándose en buscar_sign.
  - Sintonización más suave del PI: KP baja a 0.25, KI baja a 0.40 para reducir 
    el "chattering" violento detectado (1200+ trims en 3 mins).
  - Reducción de SLOPE_MIN_ABS a 0.15 (el log anterior mostró slope real de 0.204).
  - Estadísticas Avanzadas: Cálculo de RMSE, estadísticas aisladas para el 
    estado de PLL, tasa de trims/seg, y conteo de cruces por cero (oscilaciones).
--------------------------------------------------------------------------------
"""

import math
import sys
import time
from pathlib import Path
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000

# ==================== CONFIGURACIÓN Y CONSTANTES ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.09

# ---------------- OBJETIVO: FP = 0.05 ----------------
FP_OBJETIVO  = 0.05
PHI_OBJETIVO = 87.1340

FP_MIN_RANGO, FP_MAX_RANGO  = 0.02, 0.08
FP_TIGHT_LOW, FP_TIGHT_HIGH = 0.03, 0.07
PHI_TIGHT = 0.58

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 8.0, 2.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 5, 2
PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 6.0, 2.0, 6.0
N_SETTLE_BUSCAR = 3

# --- Sonda de signo en BUSCAR ---
PROBE_STEP_DEG     = 4.0     # ° tamaño de la sonda
PROBE_WAIT_SAMPLES = 2       # muestras frescas a esperar tras la sonda
PROBE_FLIP_MARGIN  = 2.5     # ° — si |err| crece más que esto, invierte
BUSCAR_TIMEOUT_SEC = 6.0     # s — fuerza PLL si BUSCAR no converge

# --- Pendiente ---
SLOPE_FIXED_BUSCAR = -1.00
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -0.80, 0.15, 25  # MIN_ABS bajado a 0.15
SLOPE_CLIP_LOW, SLOPE_CLIP_HIGH = -1.50, +1.50
SLOPE_ADAPT_W = 0.12

# --- Control PI (solo fase; df=0) suavizado ---
KP_PHI_VEL = 0.25   # Reducido de 0.55
KI_PHI_VEL = 0.40   # Reducido de 1.00
INTEGRAL_CMD_LIMIT   = 2.00
INTEGRAL_ENABLE_BAND = 2.00
INTEGRAL_RESET_TRIM  = 0.90
INTEGRAL_DECAY_OUT   = 0.99

# --- Filtro EMA del error ---
ERR_FILT_ALPHA = 0.50

STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.25
MAX_OUTLIERS_CONSEC = 5

# --- Trim de fase ---
PHI_TRIM_DEADBAND     = 0.15
PHI_TRIM_MAX          = 1.20  # Reducido de 2.20 para evitar saltos agresivos
PHI_TRIM_MIN          = 0.05
N_FRESH_COOLDOWN_TRIM = 1

OUTLIER_JUMP = 30.0
DELTA_MIN_MOVER = 0.05


def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS AVANZADAS ====================
class Stats:
    BANDS = [0.005, 0.01, 0.02, 0.03]

    def __init__(self):
        self.fp_hist, self.fp_err_hist, self.phi_err_hist = [], [], []
        self.fp_pll_hist = []  # Exclusivo para métricas estables
        
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = self.n_stale = self.n_outliers = 0
        
        self.n_trims = 0
        self.trims_phi_delta = []
        
        self.t_in_band    = {b: 0.0  for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}
        
        # Detección de oscilaciones
        self.last_err_sign = None
        self.zero_crossings = 0

    def registrar(self, fp, phi_err, modo, dt):
        self.t_total += dt
        self.n_total += 1
        self.fp_hist.append(fp)
        
        fp_err = fp - FP_OBJETIVO
        self.fp_err_hist.append(abs(fp_err))
        self.phi_err_hist.append(abs(phi_err))
        
        # Conteo de oscilaciones (cruces por cero del error de FP)
        current_sign = 1 if fp_err > 0 else -1 if fp_err < 0 else 0
        if self.last_err_sign is not None and current_sign != 0 and current_sign != self.last_err_sign:
            self.zero_crossings += 1
        self.last_err_sign = current_sign if current_sign != 0 else self.last_err_sign

        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt
        
        if modo == "PLL":
            self.fp_pll_hist.append(fp)

        for b in self.BANDS:
            if abs(fp_err) <= b:
                self.t_in_band[b] += dt
                if self.t_first_band[b] is None:
                    self.t_first_band[b] = self.t_total

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNÓSTICO v11.27-006 — Objetivo FP={FP_OBJETIVO} "
              f"(φ_obj={PHI_OBJETIVO:.4f}°)")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            rmse = np.sqrt(np.mean(np.square(np.array(self.fp_hist) - FP_OBJETIVO)))
            print(f"[ Control Global ]  media={media_fp:.4f}  std={np.std(self.fp_hist):.4f}  "
                  f"sesgo={sesgo:+.4f}  RMSE={rmse:.4f}")
            
        if self.fp_pll_hist:
            pll_mean = np.mean(self.fp_pll_hist)
            pll_std = np.std(self.fp_pll_hist)
            print(f"[ Estabilidad PLL ] media={pll_mean:.4f}  std={pll_std:.4f}  "
                  f"(Filtrado sin fase BUSCAR)")

        print(f"\n[ Distribución de Modos y Estabilidad ]")
        for m, t in self.times_mode.items():
            pct_m = (t / max(self.t_total, 1e-6)) * 100
            print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")
            
        tasa_trims = self.n_trims / max(self.times_mode.get("PLL", 1), 1e-6)
        print(f"   Trims en PLL : {self.n_trims} intervenciones ({tasa_trims:.2f} trims/seg) - Medio |Δφ|: {(np.mean(self.trims_phi_delta) if self.trims_phi_delta else 0.0):.2f}°")
        print(f"   Oscilaciones : {self.zero_crossings} cruces por el objetivo")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.3f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.02] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else "BUENO" if pct_t >= 50 else "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±0.02: {pct_t:.1f}%)")
        
        if ctrl is not None:
            print(f"[ Parámetros Internos ]")
            print(f"   Pendiente final  : {ctrl.slope:+.3f} (muestras={ctrl.slope_samples})")
            print(f"   Signo BUSCAR     : {ctrl.buscar_sign:+.0f} (sondas={ctrl.n_probes})")
            print(f"   Integrador (I)   : {ctrl.integral_cmd:+.3f}°")
            print(f"   Fase_cmd final   : {ctrl.fase_cmd:+.3f}°")
        print("=" * 78)


# ==================== CONTROLADOR ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.modo = "BUSCAR"
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(0.0)
        self.error_filt  = error_fase(0.0)
        self.phi_stale = True
        self.slope = SLOPE_INIT
        self.slope_samples = SLOPE_SAMPLES_INIT
        self.fase_prev = None
        self.phi_prev = None
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        self.move_time = 0.0
        self.last_fresh_time = time.time()
        self.n_outliers_consec = 0
        self.cooldown_trim = 0
        self.integral_cmd = 0.0

        # --- Sonda de signo ---
        self.buscar_sign = +1.0            
        self.probe_started = False
        self.probe_done = False
        self.probe_err_before = 0.0
        self.probe_counter = 0
        self.buscar_start_time = 0.0
        self.buscar_timeout_triggered = False
        self.n_probes = 0

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": FREC_NOMINAL + self.delta_f,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale, "slope": self.slope or 0.0}

    def _mover_fase(self, delta):
        if abs(delta) < DELTA_MIN_MOVER: return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = clamp(envolver_fase(self.fase_cmd + delta), FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        self.move_pending = True
        self.fresh_after_move = False
        self.move_time = time.time()
        self.cnt = 0
        return True

    def _phi(self, phi_raw, stats):
        phi_w = envolver_fase(phi_raw)
        now = time.time()
        if self.phi_prev_raw is None:
            self.phi_fresh, self.error_fresh = phi_w, error_fase(phi_w)
            self.error_filt = self.error_fresh
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.last_fresh_time = now
            return True
        step_prev = abs(envolver_fase(phi_w - self.phi_prev_raw))
        if step_prev > OUTLIER_JUMP:
            stats.n_outliers += 1
            self.n_outliers_consec += 1
            if self.n_outliers_consec < MAX_OUTLIERS_CONSEC:
                self.phi_prev_raw = phi_w
                self.phi_stale = False
                return False
        else:
            self.n_outliers_consec = 0
        self.phi_prev_raw = phi_w
        step_fresh = abs(envolver_fase(phi_w - self.phi_fresh))
        force_fresh = (now - self.last_fresh_time) > STALE_FORCE_SEC
        if step_fresh < STALE_TOL and not force_fresh:
            self.phi_stale = True
            return False
        self.phi_fresh, self.error_fresh = phi_w, error_fase(phi_w)
        self.error_filt = ERR_FILT_ALPHA * self.error_fresh + (1 - ERR_FILT_ALPHA) * self.error_filt
        self.phi_stale = False
        self.last_fresh_time = now
        if self.move_pending: self.fresh_after_move = True
        return True

    def _slope(self):
        if not self.move_pending: return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False; return
        if time.time() - self.move_time < SLOPE_SETTLE_SEC: return
        if not self.fresh_after_move: return
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 0.2 or abs(dphi) < 0.05:
            self.move_pending = False; return
        sn = dphi / dfase
        if not (0.1 <= abs(sn) <= 4.0):
            self.move_pending = False; return
        sn = clamp(sn, SLOPE_CLIP_LOW, SLOPE_CLIP_HIGH)
        w = SLOPE_ADAPT_W
        self.slope = (1 - w) * self.slope + w * sn
        self.slope_samples += 1
        self.move_pending = False

    def _cambiar(self, modo):
        if modo == self.modo: return
        previo = self.modo
        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

        if modo == "BUSCAR":
            self.probe_started = False
            self.probe_done = False
            self.probe_counter = 0
            self.buscar_start_time = time.time()
            self.buscar_timeout_triggered = False
        elif modo == "PLL":
            self.integral_cmd = 0.0
            self.error_filt = self.error_fresh
            # Heredamos el mejor signo conocido al entrar a PLL
            if self.slope is not None and (self.slope * self.buscar_sign) < 0:
                self.slope = SLOPE_INIT * self.buscar_sign

    def actualizar(self, fp, phi_med, dt, stats):
        if phi_med is None: return self._r("SIN_PHI")
        fresh = self._phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh = 0.0 if fresh else self.dt_since_fresh + dt
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1

        if fresh:
            self.cooldown_trim = max(0, self.cooldown_trim - 1)

        self._slope()
        if fresh:
            self.fresh_enter_buscar = (self.fresh_enter_buscar + 1
                                       if abs(self.error_fresh) > PHI_BUSCAR_ENTER else 0)
            self.fresh_exit_buscar = (self.fresh_exit_buscar + 1
                                      if abs(self.error_fresh) < PHI_BUSCAR_EXIT else 0)
        if self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            self._cambiar("BUSCAR")
        elif self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            self._cambiar("PLL")
        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh, stats)

    def _buscar(self, fresh):
        if not fresh: return self._r("BUSCAR-stale")

        if (not self.buscar_timeout_triggered
                and time.time() - self.buscar_start_time > BUSCAR_TIMEOUT_SEC):
            self.buscar_timeout_triggered = True
            self._cambiar("PLL")
            return self._r("BUSCAR-timeout → PLL")

        if self.cnt < N_SETTLE_BUSCAR: return self._r("BUSCAR-settle")
        self.delta_f = 0.0

        err = self.error_fresh
        s_eff = SLOPE_FIXED_BUSCAR * self.buscar_sign

        if not self.probe_done:
            if not self.probe_started:
                self.probe_started = True
                self.probe_err_before = err
                self.probe_counter = 0
                self.n_probes += 1
                d = -err / s_eff
                if abs(d) < PROBE_STEP_DEG * 0.5:
                    d = PROBE_STEP_DEG * (1.0 if d >= 0 else -1.0)
                d = clamp(d, -PROBE_STEP_DEG, PROBE_STEP_DEG)
                cambio = self._mover_fase(d)
                return self._r(f"BUSCAR-probe φ{d:+.1f}°", cambio_fase=cambio, fresh=True)
            else:
                if not self.fresh_after_move:
                    return self._r("BUSCAR-probe-espera")
                self.probe_counter += 1
                if self.probe_counter < PROBE_WAIT_SAMPLES:
                    return self._r(f"BUSCAR-probe-wait({self.probe_counter})")
                
                err_after = err
                err_before = self.probe_err_before
                mismo_signo = (err_after * err_before) >= 0
                delta_err = abs(err_after) - abs(err_before)
                if mismo_signo and delta_err > PROBE_FLIP_MARGIN:
                    self.buscar_sign *= -1.0
                    self.probe_done = True
                    return self._r(f"BUSCAR-signo flip → {self.buscar_sign:+.0f}",
                                   cambio_fase=False, fresh=True)
                else:
                    self.probe_done = True
                    return self._r(f"BUSCAR-signo OK ({self.buscar_sign:+.0f})",
                                   cambio_fase=False, fresh=True)

        s_eff = SLOPE_FIXED_BUSCAR * self.buscar_sign
        d = -err / s_eff
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, stats):
        self.delta_f = 0.0

        if fresh:
            # FIX CRÍTICO: Respetar la dirección descubierta si la magnitud de pendiente es dudosa
            s_val = self.slope if self.slope is not None else (SLOPE_FIXED_BUSCAR * self.buscar_sign)
            
            if abs(s_val) < SLOPE_MIN_ABS:
                s_sign = 1.0 if s_val >= 0 else -1.0
                # Si el signo discrepa de la sonda comprobada, prevalece la sonda
                if s_val * self.buscar_sign < 0: 
                    s_sign = self.buscar_sign
                s = SLOPE_MIN_ABS * s_sign
            else:
                s = s_val

            err = self.error_fresh
            err_f = self.error_filt
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)

            e = -err_f / s

            # ---- P ----
            dP = KP_PHI_VEL * e

            # ---- I con anti-windup ----
            if abs(err_f) < INTEGRAL_ENABLE_BAND:
                self.integral_cmd += KI_PHI_VEL * e * eff_dt
                self.integral_cmd = clamp(self.integral_cmd,
                                          -INTEGRAL_CMD_LIMIT, INTEGRAL_CMD_LIMIT)
            else:
                self.integral_cmd *= INTEGRAL_DECAY_OUT

            d = dP + self.integral_cmd
            d = clamp(d, -PHI_TRIM_MAX, PHI_TRIM_MAX)

            # Deadband + cooldown
            if abs(err_f) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    stats.registrar_trim(d)
                    self.integral_cmd *= INTEGRAL_RESET_TRIM
                    self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                    return self._r(f"PLL φ{d:+.2f}° (ef={err_f:+.2f})",
                                   cambio_fase=True, fresh=True)

        return self._r(f"PLL ef={self.error_filt:+.2f}° (I={self.integral_cmd:+.2f})",
                       fresh=fresh)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.27-006 — Objetivo FP={FP_OBJETIVO} "
          f"(φ_obj={PHI_OBJETIVO:.4f}°)")
    print("=" * 78)
    fg = wt = None
    ctrl = ControladorFP()
    stats = Stats()
    total = 0
    t_ctrl = None

    try:
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=0.0, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True, configurar_salida=True)

        print(f"\n{'t (s)':>6} | {'FP Medido':>10} | {'|FP-0.05|':>10} | "
              f"{'Modo':>7} | {'Acción':>30}")
        print("-" * 78)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO)
                continue
            if wt.is_outlier(): continue
            fp_med, phi_med, f_med = m.get("factor_potencia"), m.get("angulo_fase"), m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None else abs(np.cos(np.radians(phi_med)))
            total += 1
            res = ctrl.actualizar(fp_med, phi_med, dt, stats)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt)

            print(f"{int(t_now - t_ctrl):>6} | {fp_abs:>10.4f} | "
                  f"{abs(fp_abs - FP_OBJETIVO):>10.4f} | {res['modo']:>7} | {res['accion']:>30}")

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)
    except KeyboardInterrupt:
        if total > 0 and t_ctrl is not None: stats.resumen(ctrl)
    finally:
        if fg: fg.desconectar()
        if wt: wt.desconectar()

if __name__ == "__main__":
    main()