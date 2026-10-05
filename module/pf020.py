"""
pf020.py — Nivel intermedio: Algoritmo de control de FP v11.10
=========================================================================
Objetivo: FP = 0.2000   (φ_obj ≈ 78.46°)
Instrumentación controlada por main.py (no desde aquí).

Contrato estándar (TODOS los AlgoritmoPFxxx):
    __init__(config=None, log_cb=None, ref_source="voltage"|"current")
    set_ref_source(src) / get_ref_source()
    reset()
    procesar(fp, phi, f_legacy, dt, t_rel, f_med_u=None, f_med_i=None)
    resumen_str(t_total=None) / resumen(t_total=None)
    header_text()

Contenido:
  - ControladorFP v11.10 (INTACTO): PI en frecuencia (no en fase),
    BUSCAR con paso grande y fase de calibración inicial, banda muerta
    de 0.8° y cooldown de 4 muestras para estabilizar el sesgo.
  - Stats v11.10 (INTACTO).
  - Wrapper AlgoritmoPF020 con la misma API que los demás.

NOTA:
  Este controlador NO usa feedforward de frecuencia. Su lazo de frecuencia
  es un PI clásico (KP_PLL / KI_PLL) que corrige el sesgo residual.
  A diferencia de PF1/PF001/PF0001, aquí df ≠ 0 y sí mueve la frecuencia
  del FG, además de la fase.

NOTA v11.10b (compatibilidad main.py v11.28):
  * __init__ acepta ref_source. El controlador v11.10 NO usa la
    frecuencia medida; ref_source se guarda solo por compatibilidad y
    aparece en el resumen para trazabilidad.
  * procesar() acepta f_med_u/f_med_i (se ignoran en la lógica; se
    exponen en el diccionario de retorno por uniformidad con otros
    niveles).
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
# CONFIGURACIÓN v11.10 (idéntica)
# ==============================================================================
PHI_OBJETIVO = 78.46
FP_OBJETIVO  = 0.2

FP_MIN_RANGO, FP_MAX_RANGO  = 0.17, 0.23
FP_TIGHT_LOW, FP_TIGHT_HIGH = 0.19, 0.21
PHI_TIGHT = 0.58

# Tolerancias para stats robustas
TOL_OBJETIVO = 0.02   # ±10% de 0.2
TOL_FINA     = 0.01   # ± 5% de 0.2

FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 40.0, 12.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 3, 3
PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 15.0, 5.0, 25.0
N_SETTLE_BUSCAR = 15

# --- Ganancias y parámetros dinámicos ---
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -0.7, 0.25, 25
KP_PLL, KI_PLL = 0.0045, 0.0040
DF_MAX, DF_LP  = 0.10, 0.35

STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.25
MAX_OUTLIERS_CONSEC = 5

# --- Banda muerta y cooldown ---
PHI_TRIM_DEADBAND, PHI_TRIM_MAX, PHI_TRIM_MIN = 0.8, 7.0, 0.3
N_FRESH_COOLDOWN_TRIM = 4

CALIB_PASO = 5.0
OUTLIER_JUMP = 30.0
DELTA_MIN_MOVER = 0.15

# --- Logging (throttle para muestras stale) ---
LOG_EVERY_N_STALE = 10


def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==============================================================================
# STATS v11.10 (idéntico)
# ==============================================================================
class Stats:
    BANDS = [0.01, 0.02, 0.03, 0.05]

    def __init__(self):
        self.fp_hist, self.fp_err_hist, self.phi_err_hist = [], [], []
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = self.n_stale = self.n_outliers = 0
        self.n_trims = 0
        self.trims_phi_delta = []
        self.t_in_band    = {b: 0.0  for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}

    def registrar(self, fp, phi_err, modo, dt):
        self.t_total += dt
        self.n_total += 1
        fp_abs = abs(fp)
        fp_err = abs(fp_abs - FP_OBJETIVO)
        self.fp_hist.append(fp_abs)
        self.fp_err_hist.append(fp_err)
        self.phi_err_hist.append(abs(phi_err))
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt
        for b in self.BANDS:
            if fp_err <= b:
                self.t_in_band[b] += dt
                if self.t_first_band[b] is None:
                    self.t_first_band[b] = self.t_total

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl=None):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNÓSTICO Y ESTADÍSTICAS v11.10 — Objetivo FP={FP_OBJETIVO}")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  "
              f"outliers={self.n_outliers}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            print(f"[ Control FP ]  media={media_fp:.4f}  std={np.std(self.fp_hist):.4f}  "
                  f"sesgo (offset)={sesgo:+.4f}  "
                  f"|FP-{FP_OBJETIVO}| med={np.median(self.fp_err_hist):.4f}  "
                  f"trims={self.n_trims}")
            if self.trims_phi_delta:
                print(f"[ Trims ]  |dφ| medio={np.mean(self.trims_phi_delta):.3f}°  "
                      f"max={np.max(self.trims_phi_delta):.3f}°")

        print(f"\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            pct_m = (t / max(self.t_total, 1e-6)) * 100
            print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.2f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.02] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=80%)" if pct_t >= 80
             else "BUENO" if pct_t >= 50 else "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±0.02: {pct_t:.1f}%)")
        print("=" * 78)


# ==============================================================================
# CONTROLADOR v11.10 (idéntico)
# ==============================================================================
class ControladorFP:
    def __init__(self, log_cb=None):
        self._log_cb = log_cb

        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.df_integral = 0.0
        self.modo = "BUSCAR"
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(0.0)
        self.phi_stale = True
        self.slope = SLOPE_INIT
        self.slope_samples = SLOPE_SAMPLES_INIT
        self.fase_prev = None
        self.phi_prev = None
        self.paso_buscar = PASO_BUSCAR_INICIAL
        self.dir_buscar = +1.0
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        self.calib_done = False
        self.move_time = 0.0
        self.last_fresh_time = time.time()
        self.n_outliers_consec = 0
        self.cooldown_trim = 0

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
        if abs(dfase) < 2.0 or abs(dphi) < 0.2:
            self.move_pending = False; return
        sn = dphi / dfase
        if not (0.1 <= abs(sn) <= 4.0):
            self.move_pending = False; return
        w = 0.10
        self.slope = (1 - w) * self.slope + w * sn
        self.slope_samples += 1
        self.move_pending = False

    def _cambiar(self, modo):
        if modo == self.modo: return
        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

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
        if self.cnt < N_SETTLE_BUSCAR: return self._r("BUSCAR-settle")
        self.delta_f = 0.0
        if not self.calib_done:
            d = self.dir_buscar * CALIB_PASO
            cambio = self._mover_fase(d)
            if cambio: self.calib_done = True
            return self._r(f"CALIB φ{d:+.0f}°", cambio_fase=cambio, fresh=True)
        err = self.error_fresh
        d = -err / self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) \
            else self.dir_buscar * self.paso_buscar
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, stats):
        if fresh:
            s = self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) \
                else SLOPE_INIT
            err = self.error_fresh

            if abs(err) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
                d = clamp(-err / s, -PHI_TRIM_MAX, PHI_TRIM_MAX)
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    stats.registrar_trim(d)
                    self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                    return self._r(f"PLL-trim φ{d:+.1f}°",
                                   cambio_fase=True, fresh=True)

            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            err_int = -err / s
            self.df_integral = clamp(
                self.df_integral + KI_PLL * err_int * eff_dt,
                -DF_MAX, DF_MAX)
            df_out = KP_PLL * err_int + self.df_integral
            self.delta_f = clamp((1 - DF_LP) * self.delta_f + DF_LP * df_out,
                                 -DF_MAX, DF_MAX)
        return self._r(f"PLL df={self.delta_f:+.5f}", fresh=fresh)


# ==============================================================================
# WRAPPER — misma API que AlgoritmoPF1 / PF010 / PF001 / PF0001 / PF050
# ==============================================================================
class AlgoritmoPF020:
    """
    Nivel intermedio — Objetivo FP = 0.2000 (φ_obj = 78.46°).
    No toca instrumentos; main.py maneja FG420 y WT3000.
    """
    NOMBRE    = "PF020 · FP=0.200 (v11.10 sesgo/BUSCAR)"
    NIVEL     = 2

    # Metadatos para la GUI
    TARGET_FP  = FP_OBJETIVO      # 0.2
    TOL_HALF   = TOL_FINA         # 0.01 "excelente"
    TOL_FP     = TOL_OBJETIVO     # 0.02 "en rango"
    TOL_LOOSE  = 0.05             # "cerca"
    PHI_TARGET = PHI_OBJETIVO

    DISPLAY_RANGES = [
        ("±0.01 ✓✓", "#3ddc84"),
        ("±0.02 ✓",  "#3ddc84"),
        ("±0.05 ◐",  "#ffd166"),
        (">0.05 ✗",  "#ff5c7a"),
    ]

    HEADER_LINES = [
        "-" * 78,
        f"{'t[s]':>6} | {'FP':>8} | {'|FP-0.2|':>10} | "
        f"{'Δf (mHz)':>9} | {'Modo':>7} | Acción",
        "-" * 78,
    ]

    def __init__(self, config=None, log_cb=None, ref_source=None):
        self.config = config or {}
        self._log_cb = log_cb

        # ref_source: se guarda por compatibilidad; el controlador v11.10
        # NO usa la frecuencia medida en su lógica (su PI de frecuencia
        # trabaja sobre el error de fase, no sobre f_red).
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
                  f"(no usada por el lazo de control de PF020)")

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

    # ---------- selección de frecuencia de referencia (no usada por el lazo) ----------
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

        if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO:
            self.en_rango += 1

        # El controlador v11.10 no usa f_red; se ignora (compatibilidad de firma)
        res = self.controlador.actualizar(fp_med, phi_med, dt, self.stats)

        self.stats.registrar(fp_abs, res["error_fase"], res["modo"], dt)

        err_fp = abs(fp_abs - FP_OBJETIVO)

        is_fresh = bool(res.get("fresh"))
        should_log = is_fresh or (self._log_n % LOG_EVERY_N_STALE == 0)
        if should_log:
            mark = "*" if err_fp <= TOL_OBJETIVO else " "
            f_tag = " " if is_fresh else "·"
            self._log(
                f"{t_rel:6.1f} | {fp_abs:>8.4f} | "
                f"{err_fp:>10.4f} [{mark}]{f_tag} | "
                f"{res['delta_f']*1000:>+9.2f} | "
                f"{res['modo']:>7} | {res['accion']:>24}"
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
                print("  → FU (frecuencia del VOLTAJE) del WT3000")
            else:
                print("  → FI (frecuencia de la CORRIENTE) del WT3000")
            print("  (El controlador v11.10 NO realimenta con la frecuencia "
                  "medida; ref_source es solo trazabilidad.)")
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
            "-" * 78 + "\n"
            f"  Algoritmo PF020 v11.10 — objetivo FP=0.200 (φ_obj=78.46°) · "
            f"referencia: {ref_tag} (no usada por el lazo)\n"
            + "-" * 78 + "\n"
            + "\n".join(self.HEADER_LINES[1:])
        )