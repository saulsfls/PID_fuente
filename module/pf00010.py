"""
pf00010.py — Nivel 4: Algoritmo de control de FP v11.27
=========================================================================
Objetivo: FP = 0.0010   (φ_obj ≈ 89.9427°)
Instrumentación controlada por main.py (no desde aquí).

Contrato estándar (TODOS los AlgoritmoPFxxx):
    __init__(config=None, log_cb=None, ref_source="voltage"|"current")
    set_ref_source(src) / get_ref_source()
    reset()
    procesar(fp, phi, f_legacy, dt, t_rel, f_med_u=None, f_med_i=None)
    resumen_str(t_total=None) / resumen(t_total=None)
    header_text()

Contenido:
  - ControladorFP v11.27 (INTACTO): feedforward de frecuencia + integrador
    de frecuencia, filtro exponencial de fase, corrección de ambigüedad
    de fase alrededor de ±90° (crítico para FP≈0), warmup global, debug.
  - Stats v11.27 (INTACTO).
  - Wrapper AlgoritmoPF00010 con la misma API que AlgoritmoPF1/PF010/PF001.

NOTA IMPORTANTE:
  El controlador usa `f_red` (frecuencia medida) para el feedforward y
  `time.perf_counter()` para el timing interno. La API pública NO cambia.

NOTA v11.27b (compatibilidad main.py v11.28):
  * __init__ acepta ref_source.
  * procesar() acepta f_med_u/f_med_i; el wrapper elige cuál usar como
    f_red según ref_source.
  * Logging con throttle (LOG_EVERY_N_STALE) para que la consola nunca
    quede en silencio si todas las muestras resultan "stale".
"""
import io
import time
from contextlib import redirect_stdout

import numpy as np

# ==============================================================================
# REFERENCIA (contrato estándar)
# ==============================================================================
REF_SOURCE_VOLTAGE = "voltage"
REF_SOURCE_CURRENT = "current"
DEFAULT_REF_SOURCE = REF_SOURCE_VOLTAGE

# ==============================================================================
# CONFIGURACIÓN v11.27 (idéntica)
# ==============================================================================
FP_OBJETIVO  = 0.001
PHI_OBJETIVO = 89.9427

TOL_OBJETIVO = 0.0003
TOL_FINA     = 0.0001
PHI_TIGHT    = 0.02

FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# --- Warmup global ---
WARMUP_N = 8

# --- Feedforward de frecuencia ---
FF_ENABLE     = True
FF_GAIN       = 1.00
FF_LIMIT_HZ   = 0.50
ALPHA_FREQ    = 0.03
N_FREQ_WARMUP = 5

# --- Feedforward: integrador de frecuencia ---
FF_INTEG_ENABLE     = True
FF_INTEG_GAIN       = 2e-4
FF_INTEG_LIMIT_HZ   = 0.05
FF_INTEG_BAND_DEG   = 5.0

# --- Filtro exponencial de fase ---
PHI_FILTER_ALPHA = 0.25

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER = 4.00
PHI_BUSCAR_EXIT  = 1.00
N_FRESH_ENTER_BUSCAR = 5
N_FRESH_EXIT_BUSCAR  = 6

# --- Comportamiento del modo BUSCAR ---
PASO_BUSCAR_INICIAL   = 2.0
PASO_BUSCAR_INICIAL_K = 4.0
PASO_BUSCAR_MIN       = 0.20
PASO_BUSCAR_MAX       = 3.00
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
PHI_TRIM_DEADBAND     = 0.08
PHI_TRIM_MAX          = 0.10
PHI_TRIM_MIN          = 0.01
N_FRESH_COOLDOWN_TRIM = 3

# --- Filtros y temporización ---
STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX = 1.5
SLOPE_SETTLE_SEC = 0.35
MAX_OUTLIERS_CONSEC = 5

OUTLIER_JUMP    = 30.0
DELTA_MIN_MOVER = 0.01

# --- Debug ---
DEBUG_PHI = True
DEBUG_EVERY_N = 20

# --- Logging (throttle para muestras stale) ---
LOG_EVERY_N_STALE = 10


# ==============================================================================
# Helpers de fase
# ==============================================================================
def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def normalizar_fase_operativa(phi_deg):
    """
    Lleva la lectura del WT3000 a la vecindad de PHI_OBJETIVO.
    Resuelve la ambigüedad de signo cuando el objetivo está cerca de ±90°
    y el FP es pequeño (cos(φ) es par en φ → +φ y −φ dan el mismo FP).
    """
    p = envolver_fase(phi_deg)
    dist_pos = abs(envolver_fase(p - PHI_OBJETIVO))
    dist_neg = abs(envolver_fase(-p - PHI_OBJETIVO))
    if dist_neg < dist_pos:
        p = envolver_fase(-p)
    return p


def error_fase(phi_deg):
    return envolver_fase(phi_deg - PHI_OBJETIVO)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==============================================================================
# STATS v11.27 (idéntico)
# ==============================================================================
class Stats:
    BANDS = [0.0001, 0.0003, 0.0005, 0.001, 0.002]

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
        print(f" RESUMEN v11.27 (FP objetivo={FP_OBJETIVO}) — φ_obj={PHI_OBJETIVO:.4f}°")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            mediana_fp = np.median(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            print(f"[ Control FP ]  media={media_fp:.6f}  mediana={mediana_fp:.6f}  "
                  f"std={np.std(self.fp_hist):.6f}  sesgo={sesgo:+.6f}")
            print(f"                |FP-obj| med={np.median(self.fp_err_hist):.6f}  "
                  f"p90={np.percentile(self.fp_err_hist, 90):.6f}")

        if self.f_red_hist:
            print(f"[ Feedforward ]  f_red media={np.mean(self.f_red_hist):.4f} Hz  "
                  f"(std {np.std(self.f_red_hist)*1000:.1f} mHz)")
            print(f"                 Δf medio={np.mean(self.delta_f_hist)*1000:+.2f} mHz  "
                  f"(std {np.std(self.delta_f_hist)*1000:.1f} mHz)")

        print(f"\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            pct_m = (t / max(self.t_total, 1e-6)) * 100
            print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO:.3f}| ]")
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
            print(f"[ fase_cmd final ]  {ctrl.fase_cmd:+.4f}°")
            print(f"[ delta_f final ]   {ctrl.delta_f*1000:+.2f} mHz")
            print(f"[ Anomalías φ ]  signo={ctrl.n_ambig_signo}  "
                  f"270°={ctrl.n_ambig_270}  saltos={ctrl.n_jumps}  "
                  f"cambios_fase={ctrl.n_cambios_fase}")
        print("=" * 78)


# ==============================================================================
# CONTROLADOR v11.27 (idéntico)
# ==============================================================================
class ControladorFP:
    def __init__(self, log_cb=None):
        self._log_cb = log_cb

        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.f_filt = FREC_NOMINAL
        self.n_freq_actualizaciones = 0

        self.phi_filtrado = None
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(0.0)
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

        self.n_ambig_signo = 0
        self.n_ambig_270 = 0
        self.n_jumps = 0

    # ---------------------------------------------------------------
    def _log(self, msg):
        """Hook de log — no se usa hoy pero se deja por consistencia."""
        if self._log_cb:
            try:
                self._log_cb(msg)
                return
            except Exception:
                pass
        print(msg)

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

        es_ambig_signo = abs(envolver_fase(p_env + PHI_OBJETIVO)) < 5.0
        es_ambig_270 = abs(envolver_fase(p_env - (PHI_OBJETIVO - 180.0))) < 5.0
        if es_ambig_signo:
            self.n_ambig_signo += 1
        if es_ambig_270:
            self.n_ambig_270 += 1

        phi_w = normalizar_fase_operativa(phi_raw)

        now = time.perf_counter()

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
        if abs(dfase) < 0.2 or abs(dphi) < 0.02:
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
        if abs(err) > 20.0:
            d = np.sign(d) * max(abs(d), PASO_BUSCAR_INICIAL * PASO_BUSCAR_INICIAL_K)
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
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
            d = clamp(d, -PHI_TRIM_MAX, PHI_TRIM_MAX)
            if abs(err) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    stats.registrar_trim(d)
                    self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                    return self._r(f"PLL φ{d:+.4f}° (e={err:+.4f})",
                                   cambio_fase=True, fresh=True)
        return self._r(f"PLL e={self.error_fresh:+.4f}° (I={self.integral_cmd:+.4f})",
                       fresh=fresh)


# ==============================================================================
# WRAPPER — misma API que AlgoritmoPF1 / PF010 / PF001
# ==============================================================================
class AlgoritmoPF00010:
    """
    Nivel 4 — Objetivo FP = 0.0010 (φ_obj = 89.9427°).
    No toca instrumentos; main.py maneja FG420 y WT3000.
    """
    NOMBRE    = "PF0001 · FP=0.001 (FF+integrador v11.27)"
    NIVEL     = 4

    # Metadatos para la GUI
    TARGET_FP  = FP_OBJETIVO      # 0.001
    TOL_HALF   = TOL_FINA         # 0.0001 "excelente"
    TOL_FP     = TOL_OBJETIVO     # 0.0003 "en rango"
    TOL_LOOSE  = 0.001            # "cerca"
    PHI_TARGET = PHI_OBJETIVO

    DISPLAY_RANGES = [
        ("±0.0001 ✓✓", "#3ddc84"),
        ("±0.0003 ✓",  "#3ddc84"),
        ("±0.001 ◐",   "#ffd166"),
        (">0.001 ✗",   "#ff5c7a"),
    ]

    HEADER_LINES = [
        "-" * 98,
        f"{'t[s]':>7} | {'FP':>9} | {'|FP-0.001|':>11} | {'f_red':>8} | "
        f"{'Δf (mHz)':>9} | {'Modo':>7} | Acción",
        "-" * 98,
    ]

    def __init__(self, config=None, log_cb=None, ref_source=None):
        self.config = config or {}
        self._log_cb = log_cb

        # ref_source: el wrapper lo usa para elegir f_med_u (FU) o
        # f_med_i (FI) como f_red para el feedforward del controlador.
        src = ref_source
        if src is None:
            src = self.config.get("ref_source")
        if src is None:
            src = DEFAULT_REF_SOURCE
        src = str(src).lower()
        if src not in (REF_SOURCE_VOLTAGE, REF_SOURCE_CURRENT):
            src = DEFAULT_REF_SOURCE
        self.ref_source = src

        self.controlador = ControladorFP(log_cb=self._log)
        self.stats = Stats()
        self.n_iter = 0
        self.en_rango = 0
        self._log_n = 0

    # ---------- infra ----------
    def set_log_cb(self, cb):
        self._log_cb = cb

    def set_ref_source(self, src):
        src = str(src).lower()
        if src not in (REF_SOURCE_VOLTAGE, REF_SOURCE_CURRENT):
            raise ValueError(
                f"ref_source inválido: {src}. Use 'voltage' o 'current'.")
        self.ref_source = src
        self._log(f"[ref] Fuente de referencia → {src.upper()} "
                  f"(feedforward usa f_red del canal seleccionado)")

    def get_ref_source(self):
        return self.ref_source

    def _log(self, msg):
        if self._log_cb:
            try:
                self._log_cb(msg)
                return
            except Exception:
                pass
        print(msg)

    def reset(self):
        self.controlador = ControladorFP(log_cb=self._log)
        self.stats = Stats()
        self.n_iter = 0
        self.en_rango = 0
        self._log_n = 0

    # ---------- selección de frecuencia de referencia ----------
    def _elegir_f_ref(self, f_legacy, f_med_u, f_med_i):
        if f_med_u is None and f_med_i is None:
            return f_legacy
        if self.ref_source == REF_SOURCE_CURRENT:
            if f_med_i is not None:
                return f_med_i
            if f_med_u is not None:
                return f_med_u
        else:
            if f_med_u is not None:
                return f_med_u
            if f_med_i is not None:
                return f_med_i
        return f_legacy

    # ---------- núcleo ----------
    def procesar(self, fp_med, phi_med, f_med, dt, t_rel,
                 f_med_u=None, f_med_i=None):
        self.n_iter += 1
        self._log_n += 1

        f_ref = self._elegir_f_ref(f_med, f_med_u, f_med_i)

        if fp_med is not None:
            fp_abs = abs(fp_med)
        elif phi_med is not None:
            fp_abs = abs(np.cos(np.radians(phi_med)))
        else:
            fp_abs = 0.0

        # Rango amplio de aceptación (para métricas del main)
        if FP_OBJETIVO * 0.5 <= fp_abs <= FP_OBJETIVO * 3.0:
            self.en_rango += 1

        res = self.controlador.actualizar(fp_med, phi_med, dt, self.stats,
                                          f_red=f_ref)

        self.stats.registrar(fp_abs, res["error_fase"], res["modo"], dt,
                             f_red=f_ref, delta_f=res["delta_f"])

        err_fp = abs(fp_abs - FP_OBJETIVO)

        is_fresh = bool(res.get("fresh"))
        should_log = is_fresh or (self._log_n % LOG_EVERY_N_STALE == 0)
        if should_log:
            mark = "*" if err_fp <= TOL_OBJETIVO else " "
            f_tag = " " if is_fresh else "·"
            f_red_txt = f"{f_ref:.4f}" if f_ref is not None else "  --  "
            self._log(
                f"{t_rel:7.2f}s | FP={fp_abs:.6f} [{mark}]{f_tag} | "
                f"{err_fp:>11.6f} | "
                f"{f_red_txt:>8} | {res['delta_f']*1000:>+9.2f} | "
                f"{res['modo']:>7} | {res['accion']:>32}"
            )

        # Debug de ambigüedad de fase (anomalías o cada N iteraciones)
        if DEBUG_PHI and phi_med is not None:
            phi_env = envolver_fase(phi_med)
            es_anomalia = (abs(phi_env) > 150.0 and fp_abs < 0.05)
            if es_anomalia or (self.n_iter % DEBUG_EVERY_N == 0):
                ctrl = self.controlador
                self._log(
                    f"       [DBG] φ_raw={phi_med:+9.4f}°  "
                    f"φ_norm={normalizar_fase_operativa(phi_med):+9.4f}°  "
                    f"φ_obj={PHI_OBJETIVO:.4f}°  "
                    f"err={res['error_fase']:+8.4f}°  "
                    f"cmd={res['fase']:+8.3f}°  "
                    f"s={res['slope']:+.3f}  "
                    f"Iφ={ctrl.integral_cmd:+.4f}  "
                    f"If={ctrl.integral_f*1000:+.3f}mHz  "
                    f"amb_sig={ctrl.n_ambig_signo}  "
                    f"amb_270={ctrl.n_ambig_270}  "
                    f"jumps={ctrl.n_jumps}"
                )

        return {
            "frecuencia":   res["frecuencia"],
            "fase":         res["fase"],
            "cambio_fase":  res["cambio_fase"],
            "modo":         res["modo"],
            "phi":          res["phi"],
            "delta_f":      res["delta_f"],
            "fp":           fp_abs,
            "fresh":        is_fresh,
            "ref_source":   self.ref_source,
            "f_ref":        f_ref,
            "f_med_u":      f_med_u,
            "f_med_i":      f_med_i,
        }

    # ---------- resumen ----------
    def resumen_str(self, t_total=None):
        buf = io.StringIO()
        with redirect_stdout(buf):
            self.stats.resumen(self.controlador)
            print("\n[ Referencia de frecuencia ]")
            print(f"  Fuente seleccionada: {self.ref_source.upper()}")
            if self.ref_source == REF_SOURCE_VOLTAGE:
                print("  → FU (frecuencia del VOLTAJE) del WT3000  (feedforward)")
            else:
                print("  → FI (frecuencia de la CORRIENTE) del WT3000  (feedforward)")
            print("  (El controlador v11.27 usa la frecuencia medida solo para "
                  "feedforward; la realimentación de fase es independiente.)")
        return buf.getvalue()

    def resumen(self, t_total=None):
        txt = self.resumen_str(t_total)
        self._log(txt)
        return txt

    # ---------- header ----------
    def header_text(self):
        ref_tag = ("FU (voltaje)" if self.ref_source == REF_SOURCE_VOLTAGE
                   else "FI (corriente)")
        return (
            "-" * 98 + "\n"
            f"  Algoritmo PF00010 v11.27 — objetivo FP=0.001 (φ_obj=89.9427°) · "
            f"referencia: {ref_tag}\n"
            + "-" * 98 + "\n"
            + "\n".join(self.HEADER_LINES[1:])
        )