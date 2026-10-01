# ============================================================================
#  ControladorFP v12.00 — FP ≈ 0 (φ → ±90°, reactancia pura)
#  Feedforward de frecuencia + filtro exponencial de fase
#  Control robusto alrededor de la singularidad φ = ±90°
#  Basado en v11.27, con adaptación de paso en banda singular
# ============================================================================

import sys
import time
from pathlib import Path
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000
from estadisticas import solicitar_estadisticas, finalizar_seguro

# ==================== CONFIGURACIÓN Y CONSTANTES v12.00 ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 400
INTERVALO_MUESTREO = 0.09

# --- Punto de operación: FP = 0 → φ = ±90° ---
FP_OBJETIVO  = 0.0
PHI_OBJETIVO = 90.0            # singularidad: cos(90°) = 0
PHI_RAMA     = +1              # forzamos rama +90° (puede ser -1)

# --- Tolerancias (absolutas) ---
TOL_OBJETIVO = 0.001           # |FP| < 0.001  ↔  |err_φ| < 0.057°
TOL_FINA     = 0.0003          # |FP| < 0.0003 ↔  |err_φ| < 0.017°
PHI_TIGHT    = 0.06            # tolerancia útil en grados

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# --- Warmup ---
WARMUP_N = 8

# --- Feedforward de frecuencia ---
FF_ENABLE     = True
FF_GAIN       = 1.00
FF_LIMIT_HZ   = 0.50
ALPHA_FREQ    = 0.03
N_FREQ_WARMUP = 5

# --- Integrador de frecuencia ---
FF_INTEG_ENABLE   = True
FF_INTEG_GAIN     = 2e-4
FF_INTEG_LIMIT_HZ = 0.05
FF_INTEG_BAND_DEG = 5.0

# --- Filtro exponencial de fase ---
PHI_FILTER_ALPHA = 0.25

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER = 4.00
PHI_BUSCAR_EXIT  = 1.00
N_FRESH_ENTER_BUSCAR = 5
N_FRESH_EXIT_BUSCAR  = 6

# --- Zona crítica de singularidad ---
SINGULAR_BAND_DEG      = 3.0    # |err| < 3°  → tratamos como zona crítica
PHI_TRIM_MAX_NEAR      = 0.03   # paso máximo reducido (PLL)  cerca de 90°
PHI_TRIM_MAX_FAR       = 0.10   # paso máximo normal         (PLL)  lejos
PASO_BUSCAR_MAX_NEAR   = 0.40   # paso máximo reducido (BUSCAR) cerca de 90°
PASO_BUSCAR_MAX_FAR    = 3.00   # paso máximo normal    (BUSCAR) lejos

# --- Modo BUSCAR ---
PASO_BUSCAR_INICIAL   = 2.0
PASO_BUSCAR_INICIAL_K = 4.0
PASO_BUSCAR_MIN       = 0.20
N_SETTLE_BUSCAR       = 2

# --- Pendiente ---
SLOPE_FIXED_BUSCAR = -1.00
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -1.00, 0.85, 40
SLOPE_CLIP_LOW, SLOPE_CLIP_HIGH = -1.10, -0.90
SLOPE_ADAPT_W = 0.05

# --- Control PI de fase ---
KP_PHI_VEL = 0.15
KI_PHI_VEL = 0.35
INTEGRAL_CMD_LIMIT   = 1.00
INTEGRAL_ENABLE_BAND = 0.60

# --- Trim de fase ---
PHI_TRIM_DEADBAND     = 0.04   # ↓ más fino por la mayor sensibilidad
PHI_TRIM_MIN          = 0.01
N_FRESH_COOLDOWN_TRIM = 3

# --- Filtros y temporización ---
STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX = 1.5
SLOPE_SETTLE_SEC = 0.35
MAX_OUTLIERS_CONSEC = 5

OUTLIER_JUMP    = 30.0
DELTA_MIN_MOVER = 0.005        # ↓ más fino para FP=0

# --- Debug ---
DEBUG_PHI = True
DEBUG_EVERY_N = 20


# ---------------------------------------------------------------------------
# Helpers de fase
# ---------------------------------------------------------------------------
def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def normalizar_fase_operativa(phi_deg, rama=+1):
    """
    Lleva la lectura del WT3000 a la vecindad de la rama elegida de
    PHI_OBJETIVO (=±90°). Resuelve la ambigüedad cos(φ)=cos(−φ).
    """
    objetivo = rama * PHI_OBJETIVO
    p = envolver_fase(phi_deg)
    dist_pos = abs(envolver_fase(p - objetivo))
    dist_neg = abs(envolver_fase(-p - objetivo))
    if dist_neg < dist_pos:
        p = envolver_fase(-p)
    return p


def error_fase(phi_deg):
    return envolver_fase(phi_deg - PHI_RAMA * PHI_OBJETIVO)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS LOCALES ====================
class Stats:
    BANDS = [0.0003, 0.001, 0.002, 0.005, 0.010]

    def __init__(self):
        self.fp_hist, self.fp_err_hist, self.phi_err_hist = [], [], []
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0, "WARMUP": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = self.n_stale = self.n_outliers = 0
        self.n_trims = 0
        self.trims_phi_delta = []
        self.t_in_band    = {b: 0.0  for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}
        self.t_first_lock = None
        self.f_red_hist = []
        self.delta_f_hist = []

    def registrar(self, fp, phi_err, modo, dt, f_red=None, delta_f=None):
        self.t_total += dt
        self.n_total += 1
        self.fp_hist.append(fp)
        fp_err = abs(fp - FP_OBJETIVO)
        self.fp_err_hist.append(fp_err)
        self.phi_err_hist.append(abs(phi_err))
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt
        for b in self.BANDS:
            if fp_err <= b:
                self.t_in_band[b] += dt
                if self.t_first_band[b] is None:
                    self.t_first_band[b] = self.t_total
        if (modo == "PLL" and self.t_first_lock is None
                and fp_err <= TOL_OBJETIVO):
            self.t_first_lock = self.t_total
        if f_red is not None:
            self.f_red_hist.append(f_red)
        if delta_f is not None:
            self.delta_f_hist.append(delta_f)

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl=None):
        print("\n" + "=" * 78)
        print(f" RESUMEN v12.00 (FP objetivo={FP_OBJETIVO}) — φ_obj={PHI_RAMA*PHI_OBJETIVO:+.4f}°")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")

        if self.fp_hist:
            print(f"[ Control FP ]  media={np.mean(self.fp_hist):.6f}  "
                  f"|FP| medio={np.mean(np.abs(self.fp_hist)):.6f}  "
                  f"mediana|FP|={np.median(np.abs(self.fp_hist)):.6f}  "
                  f"std={np.std(self.fp_hist):.6f}")
            print(f"                |FP| med={np.median(self.fp_err_hist):.6f}  "
                  f"p90={np.percentile(self.fp_err_hist, 90):.6f}")
            print(f"[ Control φ  ]  |err_φ| med={np.median(self.phi_err_hist):.4f}°  "
                  f"p90={np.percentile(self.phi_err_hist, 90):.4f}°")

        if self.f_red_hist:
            print(f"[ Feedforward ]  f_red media={np.mean(self.f_red_hist):.4f} Hz  "
                  f"(std {np.std(self.f_red_hist)*1000:.1f} mHz)")
            print(f"                 Δf medio={np.mean(self.delta_f_hist)*1000:+.2f} mHz  "
                  f"(std {np.std(self.delta_f_hist)*1000:.1f} mHz)")

        print(f"\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            if t > 0:
                pct_m = (t / max(self.t_total, 1e-6)) * 100
                print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.4f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[TOL_OBJETIVO] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else
             "BUENO"     if pct_t >= 60 else
             "ACEPTABLE" if pct_t >= 40 else
             "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±{TOL_OBJETIVO}: {pct_t:.1f}%)")
        if self.t_first_lock is not None:
            print(f"[ Primer lock (±{TOL_OBJETIVO}) ]  t={self.t_first_lock:.2f}s")

        print(f"[ Trims aplicados ]  n={self.n_trims}  "
              f"|Δφ| medio={(np.mean(self.trims_phi_delta) if self.trims_phi_delta else 0.0):.4f}°  "
              f"(tolerancia útil: {PHI_TIGHT:.4f}°)")

        if ctrl is not None:
            print(f"[ Pendiente final ]  s={ctrl.slope:+.3f}  (muestras={ctrl.slope_samples})")
            print(f"[ Integrador fase ]  integral_cmd={ctrl.integral_cmd:+.4f}°  "
                  f"(límite ±{INTEGRAL_CMD_LIMIT:.2f})")
            if FF_INTEG_ENABLE:
                print(f"[ Integrador freq ]  integral_f={ctrl.integral_f*1000:+.3f} mHz")
            print(f"[ fase_cmd final ]  {ctrl.fase_cmd:+.4f}°  "
                  f"(φ_meas={ctrl.phi_fresh:+.4f}°)")
            print(f"[ delta_f final ]   {ctrl.delta_f*1000:+.2f} mHz")
            print(f"[ Anomalías φ ]  signo={ctrl.n_ambig_signo}  "
                  f"270°={ctrl.n_ambig_270}  saltos={ctrl.n_jumps}  "
                  f"cruces_90={ctrl.n_cruces_90}  cambios_fase={ctrl.n_cambios_fase}")
        print("=" * 78)


# ==================== CONTROLADOR v12.00 ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.f_filt = FREC_NOMINAL
        self.n_freq_actualizaciones = 0

        self.phi_filtrado = None
        self.phi_prev_raw = None
        self.phi_fresh = PHI_RAMA * PHI_OBJETIVO
        self.error_fresh = 0.0
        self.phi_stale = True

        self.modo = "BUSCAR"
        self.cnt = 0
        self.n_total_muestras = 0
        self.have_fresh_once = False

        self.move_pending = False
        self.fresh_after_move = False
        self.slope = SLOPE_INIT
        self.slope_samples = SLOPE_SAMPLES_INIT
        self.fase_prev = None
        self.phi_prev = None
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0

        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        self.move_time = 0.0
        self.last_fresh_time = time.perf_counter()

        self.n_outliers_consec = 0
        self.cooldown_trim = 0
        self.integral_cmd = 0.0
        self.integral_f = 0.0

        # contadores de debug
        self.n_ambig_signo = 0
        self.n_ambig_270 = 0
        self.n_jumps = 0
        self.n_cruces_90 = 0
        self._prev_err_sign = 0

    # ---------------------------------------------------------------
    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": FREC_NOMINAL + self.delta_f,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale,
                "slope": self.slope or 0.0,
                "f_red": self.f_filt, "phi_filt": self.phi_filtrado,
                "integral_f": self.integral_f}

    # ---------------------------------------------------------------
    def _paso_max(self):
        """Paso máximo dinámico según cercanía a la singularidad."""
        if abs(self.error_fresh) < SINGULAR_BAND_DEG:
            return PHI_TRIM_MAX_NEAR, PASO_BUSCAR_MAX_NEAR
        return PHI_TRIM_MAX_FAR, PASO_BUSCAR_MAX_FAR

    # ---------------------------------------------------------------
    def _actualizar_feedforward(self, f_red, dt):
        if not FF_ENABLE or f_red is None:
            return
        self.f_filt = (1 - ALPHA_FREQ) * self.f_filt + ALPHA_FREQ * f_red
        self.n_freq_actualizaciones += 1
        if self.n_freq_actualizaciones < N_FREQ_WARMUP:
            self.delta_f = 0.0
            return

        raw_delta = FF_GAIN * (self.f_filt - FREC_NOMINAL)
        delta_p = clamp(raw_delta, -FF_LIMIT_HZ, FF_LIMIT_HZ)

        if (FF_INTEG_ENABLE and self.have_fresh_once
                and self.modo == "PLL"
                and abs(self.error_fresh) < FF_INTEG_BAND_DEG):
            self.integral_f += FF_INTEG_GAIN * self.error_fresh * dt
            self.integral_f = clamp(self.integral_f,
                                    -FF_INTEG_LIMIT_HZ, FF_INTEG_LIMIT_HZ)

        self.delta_f = clamp(delta_p + self.integral_f,
                             -FF_LIMIT_HZ, FF_LIMIT_HZ)

    # ---------------------------------------------------------------
    def _filtrar_phi(self, phi_w):
        if self.phi_filtrado is None:
            self.phi_filtrado = phi_w
            return phi_w
        delta = envolver_fase(phi_w - self.phi_filtrado)
        self.phi_filtrado = envolver_fase(
            self.phi_filtrado + PHI_FILTER_ALPHA * delta
        )
        return self.phi_filtrado

    # ---------------------------------------------------------------
    def _mover_fase(self, delta):
        if abs(delta) < DELTA_MIN_MOVER:
            return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = clamp(envolver_fase(self.fase_cmd + delta),
                              FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        self.move_pending = True
        self.fresh_after_move = False
        self.move_time = time.perf_counter()
        self.cnt = 0
        return True

    # ---------------------------------------------------------------
    def _phi(self, phi_raw, stats):
        p_env = envolver_fase(phi_raw)
        objetivo = PHI_RAMA * PHI_OBJETIVO

        # Detección de ambigüedad de signo: rama espejo
        es_ambig_signo = abs(envolver_fase(p_env + objetivo)) < 5.0
        es_ambig_270 = abs(envolver_fase(p_env - (objetivo - 180.0))) < 5.0
        if es_ambig_signo:
            self.n_ambig_signo += 1
        if es_ambig_270:
            self.n_ambig_270 += 1

        phi_w = normalizar_fase_operativa(phi_raw, rama=PHI_RAMA)
        now = time.perf_counter()

        # Primera muestra
        if self.phi_prev_raw is None:
            phi_filt = self._filtrar_phi(phi_w)
            self.phi_fresh, self.error_fresh = phi_filt, error_fase(phi_filt)
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.last_fresh_time = now
            return True

        step_prev = abs(envolver_fase(phi_w - self.phi_prev_raw))
        if step_prev > OUTLIER_JUMP:
            stats.n_outliers += 1
            self.n_jumps += 1
            self.n_outliers_consec += 1
            if self.n_outliers_consec < MAX_OUTLIERS_CONSEC:
                self.phi_stale = True
                return False
        self.n_outliers_consec = 0
        self.phi_prev_raw = phi_w

        phi_filt = self._filtrar_phi(phi_w)

        step_fresh = abs(envolver_fase(phi_filt - self.phi_fresh))
        force_fresh = (now - self.last_fresh_time) > STALE_FORCE_SEC
        if step_fresh < STALE_TOL and not force_fresh:
            self.phi_stale = True
            return False

        self.phi_fresh, self.error_fresh = phi_filt, error_fase(phi_filt)
        self.phi_stale = False
        self.last_fresh_time = now
        self.have_fresh_once = True

        # Detección de cruce del eje ±90° (cambio de signo del error)
        s = 1 if self.error_fresh > 0 else (-1 if self.error_fresh < 0 else 0)
        if s != 0 and self._prev_err_sign != 0 and s != self._prev_err_sign:
            self.n_cruces_90 += 1
        if s != 0:
            self._prev_err_sign = s

        if self.move_pending:
            self.fresh_after_move = True
        return True

    # ---------------------------------------------------------------
    def _slope(self):
        if not self.move_pending:
            return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False
            return
        if time.perf_counter() - self.move_time < SLOPE_SETTLE_SEC:
            return
        if not self.fresh_after_move:
            return
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 0.2 or abs(dphi) < 0.01:
            self.move_pending = False
            return
        sn = dphi / dfase
        if not (0.1 <= abs(sn) <= 4.0):
            self.move_pending = False
            return
        sn = clamp(sn, SLOPE_CLIP_LOW, SLOPE_CLIP_HIGH)
        w = SLOPE_ADAPT_W
        self.slope = (1 - w) * self.slope + w * sn
        self.slope_samples += 1
        self.move_pending = False

    # ---------------------------------------------------------------
    def _cambiar(self, modo):
        if modo == self.modo:
            return
        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        if modo == "PLL":
            self.integral_cmd = 0.0
            self.integral_f = clamp(self.integral_f,
                                    -FF_INTEG_LIMIT_HZ, FF_INTEG_LIMIT_HZ)

    # ---------------------------------------------------------------
    def actualizar(self, fp, phi_med, dt, stats, f_red=None):
        if phi_med is None:
            self._actualizar_feedforward(f_red, dt)
            return self._r("SIN_PHI")

        self.n_total_muestras += 1
        fresh = self._phi(phi_med, stats)
        self._actualizar_feedforward(f_red, dt)

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

        if self.n_total_muestras < WARMUP_N or not self.have_fresh_once:
            return self._r("WARMUP", fresh=fresh)

        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh, stats)

    # ---------------------------------------------------------------
    def _buscar(self, fresh):
        if not fresh:
            return self._r("BUSCAR-stale")
        if self.cnt < N_SETTLE_BUSCAR:
            return self._r("BUSCAR-settle")

        err = self.error_fresh
        s = SLOPE_FIXED_BUSCAR
        d = -err / s
        # Paso inicial grande si estamos muy lejos
        if abs(err) > 20.0:
            d = np.sign(d) * max(abs(d), PASO_BUSCAR_INICIAL * PASO_BUSCAR_INICIAL_K)
        # Paso adaptativo según cercanía a la singularidad
        _, paso_max = self._paso_max()
        d = clamp(d, -paso_max, paso_max)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.3f}°", cambio_fase=cambio, fresh=True)

    # ---------------------------------------------------------------
    def _pll(self, fresh, stats):
        if fresh:
            s = self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) \
                else SLOPE_FIXED_BUSCAR
            err = self.error_fresh
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            e = -err / s
            dP = KP_PHI_VEL * e
            if abs(err) < INTEGRAL_ENABLE_BAND:
                self.integral_cmd += KI_PHI_VEL * e * eff_dt
                self.integral_cmd = clamp(self.integral_cmd,
                                          -INTEGRAL_CMD_LIMIT, INTEGRAL_CMD_LIMIT)
            d = dP + self.integral_cmd
            trim_max, _ = self._paso_max()
            d = clamp(d, -trim_max, trim_max)
            if abs(err) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    stats.registrar_trim(d)
                    self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                    return self._r(f"PLL φ{d:+.4f}° (e={err:+.4f})",
                                   cambio_fase=True, fresh=True)
        return self._r(f"PLL e={self.error_fresh:+.4f}° (I={self.integral_cmd:+.4f})",
                       fresh=fresh)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v12.00 — Objetivo FP={FP_OBJETIVO} (φ={PHI_RAMA*PHI_OBJETIVO:+.2f}°)")
    print(f" Feedforward: {'ON' if FF_ENABLE else 'OFF'}  "
          f"| FF integ: {'ON' if FF_INTEG_ENABLE else 'OFF'}  "
          f"| Filtro φ: α={PHI_FILTER_ALPHA}  | Warmup: {WARMUP_N}")
    print(f" KP={KP_PHI_VEL}  KI={KI_PHI_VEL}  DB={PHI_TRIM_DEADBAND}°  "
          f"Singular band={SINGULAR_BAND_DEG}°")
    print(f" BUSCAR[{PHI_BUSCAR_EXIT:.2f},{PHI_BUSCAR_ENTER:.2f}]  "
          f"Paso BUSCAR: lejos={PASO_BUSCAR_MAX_FAR}° cerca={PASO_BUSCAR_MAX_NEAR}°")
    print("=" * 78)

    stats_robustas = solicitar_estadisticas(
        fp_target=FP_OBJETIVO,
        tolerancia=TOL_OBJETIVO,
        tolerancia_fina=TOL_FINA,
    )

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
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True,
                   configurar_salida=True)

        print(f"\n{'t (s)':>6} | {'|FP|':>8} | {'|ΔFP|':>10} | "
              f"{'φ_meas':>9} | {'err_φ':>9} | {'Δf (mHz)':>9} | {'Modo':>7} | {'Acción':>28}")
        print("-" * 110)

        t_ctrl = t_prev = time.perf_counter()
        while time.perf_counter() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.perf_counter()
            dt = t_now - t_prev
            t_prev = t_now
            t_rel = t_now - t_ctrl

            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO)
                continue
            if wt.is_outlier():
                continue

            fp_med  = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med   = m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None \
                else abs(np.cos(np.radians(phi_med)))
            total += 1

            res = ctrl.actualizar(fp_med, phi_med, dt, stats, f_red=f_med)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt,
                            f_red=f_med, delta_f=res["delta_f"])

            if stats_robustas is not None:
                stats_robustas.agregar(
                    t_rel, fp_abs,
                    res.get("error_fase", 0.0),
                    modo=res["modo"],
                )

            print(f"{t_rel:>6.1f} | {fp_abs:>8.6f} | "
                  f"{abs(fp_abs - FP_OBJETIVO):>10.6f} | "
                  f"{res['phi']:>+9.4f} | {res['error_fase']:>+9.4f} | "
                  f"{res['delta_f']*1000:>+9.2f} | "
                  f"{res['modo']:>7} | {res['accion']:>28}")

            if DEBUG_PHI:
                phi_env = envolver_fase(phi_med)
                es_anomalia = (abs(phi_env) > 150.0 and fp_abs < 0.05)
                if es_anomalia or (total % DEBUG_EVERY_N == 0):
                    print(f"      [DBG] φ_raw={phi_med:+9.4f}°  "
                          f"φ_norm={normalizar_fase_operativa(phi_med, PHI_RAMA):+9.4f}°  "
                          f"φ_obj={PHI_RAMA*PHI_OBJETIVO:+.4f}°  "
                          f"err={res['error_fase']:+8.4f}°  "
                          f"cmd={res['fase']:+8.3f}°  "
                          f"slope={res['slope']:+.3f}  "
                          f"Iφ={ctrl.integral_cmd:+.4f}  "
                          f"If={ctrl.integral_f*1000:+.3f}mHz  "
                          f"signo={ctrl.n_ambig_signo}  "
                          f"270={ctrl.n_ambig_270}  "
                          f"jumps={ctrl.n_jumps}  "
                          f"x90={ctrl.n_cruces_90}")

            elapsed = time.perf_counter() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario (Ctrl+C).")
        if total > 0 and t_ctrl is not None:
            try:
                stats.resumen(ctrl)
                if DEBUG_PHI:
                    print(f"\n[DBG final] ambig signo={ctrl.n_ambig_signo}  "
                          f"ambig 270°={ctrl.n_ambig_270}  "
                          f"saltos={ctrl.n_jumps}  "
                          f"cruces 90°={ctrl.n_cruces_90}  "
                          f"cambios_fase={ctrl.n_cambios_fase}")
            except Exception as e:
                print(f"[!] Error en resumen: {e}")

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()
        try:
            stats.resumen(ctrl)
        except Exception as e2:
            print(f"[!] Además falló resumen: {e2}")

    finally:
        finalizar_seguro(stats_robustas)

        print("\n[!] Apagando...")
        if fg:
            try: fg.establecer_salida(1, False)
            except Exception: pass
            try: fg.desconectar()
            except Exception: pass
        if wt:
            try: wt.desconectar()
            except Exception: pass
        print("[✓] Listo.")


if __name__ == "__main__":
    main()