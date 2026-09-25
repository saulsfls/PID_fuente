"""
CONTROL DE FACTOR DE POTENCIA v11.29-008
--------------------------------------------------------------------------------
Objetivo: FP = 0.0500
Instrumentación: Yokogawa FG420 + Yokogawa WT3000 (vía GPIB)

CAMBIO DE ESTRATEGIA v11.29-008
--------------------------------
El objetivo real es FP=0.05, por lo que esta versión controla principalmente
sobre el ERROR DE FP y usa el ángulo de fase como variable auxiliar.

Problemas detectados en v11.28-007:
  - BUSCAR ocupó 78.4% del tiempo y terminó en fase_cmd=-168.256°.
  - Signo BUSCAR +1 y pendiente +0.800 no evitaron la deriva hacia el límite.
  - La búsqueda por |error de fase| no garantiza directamente minimizar |FP-0.05|.
  - Al reentrar en BUSCAR se reiniciaba la dirección, perdiendo información local.

Cambios:
  1) BUSCAR minimiza directamente |FP-FP_OBJETIVO|.
  2) Se limita la zona de búsqueda a una ventana segura de fase de comando.
  3) Cada movimiento se valida después de asentamiento; si empeora FP, revierte
     y reduce el paso. Nunca acelera cuando el resultado es incierto.
  4) Se estima una pendiente LOCAL dFP/dfase_cmd directamente del WT3000.
     Esto evita depender de una pendiente dphi/dfase_cmd que puede cambiar de signo.
  5) El PLL usa dFP/dfase_cmd para calcular los trims. Cerca del objetivo, FP es
     la variable primaria y el ángulo solo queda como diagnóstico.
  6) Se mantiene anti-windup y un integrador pequeño para eliminar sesgo permanente.
  7) No se permite BUSCAR->PLL mientras el error FP siga fuera de la ventana de
     adquisición. Se prioriza alcanzar primero FP≈0.05.
  8) La frecuencia queda fija en 60.000 Hz.
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

# ==================== CONFIGURACIÓN ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.09

# ---------------- OBJETIVO ----------------
FP_OBJETIVO = 0.05
PHI_OBJETIVO = 87.1340       # diagnóstico solamente

# Ventanas de control en FP
FP_SEARCH_ENTRY = 0.20       # BUSCAR mientras |FP-obj| > 0.20
FP_PLL_ENTRY_ERR = 0.020     # pasar a PLL si se mantiene dentro de ±0.02
FP_TIGHT_ERR = 0.010         # microcontrol dentro de ±0.01
FP_EXCELLENT_ERR = 0.005     # diagnóstico

# ---------------- FG420 ----------------
AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FREC_NOMINAL = 60.0
FASE_MIN, FASE_MAX = -180.0, 180.0

# Ventana segura para la búsqueda. Se evita que BUSCAR recorra repetidamente
# casi todo el rango [-180,+180] como ocurrió en v11.28.
FASE_BUSCAR_MIN = -120.0
FASE_BUSCAR_MAX = +120.0

# ---------------- BUSCAR por FP ----------------
BUSCAR_STEP_INICIAL = 3.0
BUSCAR_STEP_MIN = 0.25
BUSCAR_STEP_MAX = 4.0
BUSCAR_REDUCE_BAD = 0.50
BUSCAR_INCREASE_GOOD = 1.05
BUSCAR_NEUTRO_FACTOR = 0.80
BUSCAR_EVAL_MARGIN_FP = 0.0020
BUSCAR_STEP_NEAR_1 = 1.50
BUSCAR_STEP_NEAR_2 = 0.80
BUSCAR_STEP_NEAR_3 = 0.45
N_SETTLE_BUSCAR = 3
BUSCAR_MAX_TIME_SEC = 45.0
MAX_BUSCAR_REVERSALS_STREAK = 4

# ---------------- ESTIMACIÓN dFP/dfase_cmd ----------------
FP_SLOPE_INIT = -0.012
FP_SLOPE_MIN_ABS = 0.004
FP_SLOPE_VALID_MIN = 0.0015
FP_SLOPE_VALID_MAX = 0.045
FP_SLOPE_CLIP_MIN = -0.060
FP_SLOPE_CLIP_MAX = +0.060
FP_SLOPE_ADAPT_W = 0.10
FP_SLOPE_SIGN_W = 0.35
FP_SLOPE_MIN_DFP = 0.0020
FP_SLOPE_MIN_DCMD = 0.35
SLOPE_SETTLE_SEC = 0.30

# ---------------- PLL directamente en FP ----------------
KP_FP = 0.45
KI_FP = 0.22
INTEGRAL_CMD_LIMIT = 0.80
INTEGRAL_ENABLE_ERR = 0.020
INTEGRAL_LEAK = 0.998
INTEGRAL_RESET_TRIM = 0.998

FP_TRIM_DEADBAND = 0.0010
FP_TRIM_MAX_DEG = 0.85
FP_TRIM_MIN_DEG = 0.025
N_FRESH_COOLDOWN_TRIM = 2

# ---------------- Filtros / calidad ----------------
FP_FILTER_ALPHA = 0.30
PHI_FILTER_ALPHA = 0.30
STALE_FP_TOL = 0.0005
STALE_PHI_TOL = 0.05
STALE_FORCE_SEC = 0.30
OUTLIER_JUMP = 30.0
MAX_OUTLIERS_CONSEC = 5
DELTA_MIN_MOVER = 0.05


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def error_fase(phi_deg):
    return envolver_fase(phi_deg - PHI_OBJETIVO)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS ====================
class Stats:
    BANDS = [0.005, 0.01, 0.02, 0.03]

    def __init__(self):
        self.fp_hist = []
        self.fp_err_hist = []
        self.phi_err_hist = []
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = 0
        self.n_stale = 0
        self.n_outliers = 0
        self.n_trims = 0
        self.trims_phi_delta = []
        self.n_buscar_moves = 0
        self.n_buscar_reverse = 0
        self.t_in_band = {b: 0.0 for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}

    def registrar(self, fp, phi_err, modo, dt):
        self.t_total += max(dt, 0.0)
        self.n_total += 1
        self.fp_hist.append(fp)
        fp_err = abs(fp - FP_OBJETIVO)
        self.fp_err_hist.append(fp_err)
        self.phi_err_hist.append(abs(phi_err))
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + max(dt, 0.0)
        for b in self.BANDS:
            if fp_err <= b:
                self.t_in_band[b] += max(dt, 0.0)
                if self.t_first_band[b] is None:
                    self.t_first_band[b] = self.t_total

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNÓSTICO v11.29-008 — Objetivo FP={FP_OBJETIVO} "
              f"(φ_obj={PHI_OBJETIVO:.4f}°)")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            print(f"[ Control FP ]  media={media_fp:.4f}  std={np.std(self.fp_hist):.4f}  "
                  f"sesgo (offset)={sesgo:+.4f}  |FP-{FP_OBJETIVO}| med="
                  f"{np.median(self.fp_err_hist):.4f}")

        print("\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            pct_m = 100.0 * t / max(self.t_total, 1e-6)
            print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100.0 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.3f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100.0 * self.t_in_band[0.02] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else
             "BUENO" if pct_t >= 50 else "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±0.02: {pct_t:.1f}%)")
        print(f"[ Trims aplicados ]  n={self.n_trims}  "
              f"|Δφ| medio={(np.mean(self.trims_phi_delta) if self.trims_phi_delta else 0.0):.2f}°")
        if ctrl is not None:
            print(f"[ BUSCAR FP ]       movimientos={ctrl.n_buscar_moves}  "
                  f"reversiones={ctrl.n_buscar_reverse}  paso={ctrl.buscar_step:.2f}°")
            print(f"[ Pendiente dFP/dφcmd ] {ctrl.fp_slope:+.5f} FP/°")
            print(f"[ Pendiente φ/φcmd ]    {ctrl.phi_slope:+.3f}  (diagnóstico)")
            print(f"[ Dirección BUSCAR ]    {ctrl.buscar_dir:+.0f}")
            print(f"[ Integrador fase ]      integral_cmd={ctrl.integral_cmd:+.3f}°")
            print(f"[ fase_cmd final ]       {ctrl.fase_cmd:+.3f}°")
            print(f"[ FP final ]             {ctrl.fp_fresh:.4f}  e={ctrl.fp_error:+.5f}")
            print(f"[ fase medida ]          {ctrl.phi_fresh:+.3f}°  eφ={ctrl.error_fresh:+.3f}°")
        print("=" * 78)


# ==================== CONTROLADOR ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.modo = "BUSCAR"
        self.cnt = 0

        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(0.0)
        self.error_filt = self.error_fresh
        self.phi_stale = True

        self.fp_prev_raw = None
        self.fp_fresh = 1.0
        self.fp_error = 1.0 - FP_OBJETIVO
        self.fp_error_filt = self.fp_error
        self.fp_stale = True

        self.last_fresh_time = time.time()
        self.dt_since_fresh = 0.0
        self.n_outliers_consec = 0

        self.move_pending = False
        self.fresh_after_move = False
        self.move_time = 0.0
        self.fase_prev = None
        self.phi_prev = None
        self.fp_prev = None

        # Pendiente directa de objetivo: dFP / dfase_cmd
        self.fp_slope = FP_SLOPE_INIT
        self.fp_slope_samples = 0

        # Pendiente angular solo para diagnóstico
        self.phi_slope = -0.80

        # PLL
        self.integral_cmd = 0.0
        self.cooldown_trim = 0

        # BUSCAR
        self.buscar_dir = +1.0
        self.buscar_step = BUSCAR_STEP_INICIAL
        self.buscar_err_before = None
        self.buscar_eval_pending = False
        self.buscar_start_time = time.time()
        self.buscar_reversals_streak = 0
        self.n_buscar_moves = 0
        self.n_buscar_reverse = 0

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {
            "accion": accion,
            "fase": self.fase_cmd,
            "delta_f": 0.0,
            "frecuencia": FREC_NOMINAL,
            "cambio_fase": cambio_fase,
            "modo": self.modo,
            "phi": self.phi_fresh,
            "error_fase": self.error_fresh,
            "fp": self.fp_fresh,
            "error_fp": self.fp_error,
            "fresh": fresh,
            "stale": self.phi_stale or self.fp_stale,
            "slope_fp": self.fp_slope,
        }

    def _mover_fase(self, delta, limitar_buscar=False):
        if abs(delta) < DELTA_MIN_MOVER:
            return False

        lo = FASE_BUSCAR_MIN if limitar_buscar else FASE_MIN
        hi = FASE_BUSCAR_MAX if limitar_buscar else FASE_MAX

        objetivo = clamp(self.fase_cmd + delta, lo, hi)
        delta_real = objetivo - self.fase_cmd
        if abs(delta_real) < DELTA_MIN_MOVER:
            return False

        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fp_prev = self.fp_fresh
        self.fase_cmd = objetivo
        self.move_pending = True
        self.fresh_after_move = False
        self.move_time = time.time()
        self.cnt = 0
        return True

    def _mediciones(self, fp_med, phi_raw, stats):
        if phi_raw is None:
            return False

        phi_w = envolver_fase(phi_raw)
        now = time.time()

        # Primera medición
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.error_fresh = error_fase(phi_w)
            self.error_filt = self.error_fresh
            self.phi_prev_raw = phi_w
            self.phi_stale = False

            if fp_med is not None and np.isfinite(fp_med):
                self.fp_fresh = abs(float(fp_med))
                self.fp_error = self.fp_fresh - FP_OBJETIVO
                self.fp_error_filt = self.fp_error
                self.fp_prev_raw = self.fp_fresh
                self.fp_stale = False
            else:
                self.fp_stale = True

            self.last_fresh_time = now
            return not self.fp_stale

        # Outlier de fase
        step_prev = abs(envolver_fase(phi_w - self.phi_prev_raw))
        if step_prev > OUTLIER_JUMP:
            stats.n_outliers += 1
            self.n_outliers_consec += 1
            self.phi_stale = True
            if self.n_outliers_consec >= MAX_OUTLIERS_CONSEC:
                self.phi_prev_raw = phi_w
                self.n_outliers_consec = 0
            return False
        self.n_outliers_consec = 0
        self.phi_prev_raw = phi_w

        # FP absoluto es la variable primaria
        fp_valid = fp_med is not None and np.isfinite(fp_med)
        fp_w = abs(float(fp_med)) if fp_valid else None

        phi_change = abs(envolver_fase(phi_w - self.phi_fresh))
        fp_change = abs(fp_w - self.fp_fresh) if fp_valid else 0.0
        force_fresh = (now - self.last_fresh_time) > STALE_FORCE_SEC

        if phi_change < STALE_PHI_TOL and fp_change < STALE_FP_TOL and not force_fresh:
            self.phi_stale = True
            self.fp_stale = True
            return False

        self.phi_fresh = phi_w
        self.error_fresh = error_fase(phi_w)
        self.error_filt = (
            PHI_FILTER_ALPHA * self.error_fresh
            + (1.0 - PHI_FILTER_ALPHA) * self.error_filt
        )
        self.phi_stale = False

        if fp_valid:
            self.fp_fresh = fp_w
            self.fp_error = fp_w - FP_OBJETIVO
            self.fp_error_filt = (
                FP_FILTER_ALPHA * self.fp_error
                + (1.0 - FP_FILTER_ALPHA) * self.fp_error_filt
            )
            self.fp_stale = False
        else:
            self.fp_stale = True

        self.last_fresh_time = now
        if self.move_pending and not self.phi_stale and not self.fp_stale:
            self.fresh_after_move = True
        return not self.phi_stale and not self.fp_stale

    def _actualizar_pendientes(self):
        if not self.move_pending:
            return
        if self.fase_prev is None or self.fp_prev is None:
            self.move_pending = False
            return
        if time.time() - self.move_time < SLOPE_SETTLE_SEC:
            return
        if not self.fresh_after_move:
            return

        dcmd = self.fase_cmd - self.fase_prev
        dfp = self.fp_fresh - self.fp_prev
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)

        # Aprender dFP/dfase_cmd de movimientos suficientemente grandes.
        if abs(dcmd) >= FP_SLOPE_MIN_DCMD and abs(dfp) >= FP_SLOPE_MIN_DFP:
            sn_fp = dfp / dcmd
            if FP_SLOPE_VALID_MIN <= abs(sn_fp) <= FP_SLOPE_VALID_MAX:
                sn_fp = clamp(sn_fp, FP_SLOPE_CLIP_MIN, FP_SLOPE_CLIP_MAX)
                if self.fp_slope * sn_fp < 0:
                    w = FP_SLOPE_SIGN_W
                else:
                    w = FP_SLOPE_ADAPT_W
                self.fp_slope = (1.0 - w) * self.fp_slope + w * sn_fp
                self.fp_slope_samples += 1
                if abs(self.fp_slope) < FP_SLOPE_MIN_ABS:
                    self.fp_slope = math.copysign(FP_SLOPE_MIN_ABS, sn_fp)

        # dphi/dfase_cmd solo como diagnóstico.
        if abs(dcmd) >= FP_SLOPE_MIN_DCMD and abs(dphi) >= 0.10:
            sn_phi = dphi / dcmd
            if 0.10 <= abs(sn_phi) <= 3.0:
                self.phi_slope = 0.9 * self.phi_slope + 0.1 * sn_phi

        self.move_pending = False

    def _cambiar(self, modo):
        if modo == self.modo:
            return

        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False

        if modo == "BUSCAR":
            # No perder totalmente la dirección previa: usar la pendiente FP local.
            if abs(self.fp_slope) >= FP_SLOPE_MIN_ABS:
                self.buscar_dir = -math.copysign(1.0, self.fp_error_filt / self.fp_slope)
            else:
                self.buscar_dir = +1.0
            self.buscar_step = BUSCAR_STEP_INICIAL
            self.buscar_err_before = None
            self.buscar_eval_pending = False
            self.buscar_start_time = time.time()
            self.buscar_reversals_streak = 0

        elif modo == "PLL":
            # La búsqueda ya nos entregó una zona cercana al objetivo FP.
            self.integral_cmd = 0.0
            self.fp_error_filt = self.fp_error

    def _ajustar_paso(self):
        err_abs = abs(self.fp_error)
        if err_abs <= FP_EXCELLENT_ERR:
            self.buscar_step = min(self.buscar_step, BUSCAR_STEP_NEAR_3)
        elif err_abs <= FP_TIGHT_ERR:
            self.buscar_step = min(self.buscar_step, BUSCAR_STEP_NEAR_2)
        elif err_abs <= FP_PLL_ENTRY_ERR:
            self.buscar_step = min(self.buscar_step, BUSCAR_STEP_NEAR_1)

    def actualizar(self, fp, phi_med, dt, stats):
        if phi_med is None:
            return self._r("SIN_PHI")

        fresh = self._mediciones(fp, phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh = 0.0 if fresh else self.dt_since_fresh + dt
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1

        if fresh:
            self.cooldown_trim = max(0, self.cooldown_trim - 1)

        self._actualizar_pendientes()

        # Transferencia BUSCAR -> PLL por objetivo real de FP.
        if self.modo == "BUSCAR" and fresh:
            if abs(self.fp_error_filt) <= FP_PLL_ENTRY_ERR and self.cnt >= 2:
                self._cambiar("PLL")
                return self._r(
                    f"BUSCAR→PLL FP={self.fp_fresh:.4f} e={self.fp_error:+.5f}",
                    fresh=True
                )

        # Salida PLL -> BUSCAR solo si FP se alejó de verdad.
        if self.modo == "PLL" and fresh:
            if abs(self.fp_error_filt) > FP_SEARCH_ENTRY:
                self._cambiar("BUSCAR")
                return self._r("PLL→BUSCAR FP fuera de rango", fresh=True)

        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh, stats)

    def _buscar(self, fresh):
        if not fresh:
            return self._r("BUSCAR-stale")

        elapsed = time.time() - self.buscar_start_time
        err_abs = abs(self.fp_error)

        # Timeout: NO pasar al PLL lejos del objetivo. Solo reiniciar búsqueda
        # con paso pequeño y dirección opuesta a la última exploración.
        if elapsed > BUSCAR_MAX_TIME_SEC and err_abs > FP_PLL_ENTRY_ERR:
            self.buscar_dir *= -1.0
            self.buscar_step = max(BUSCAR_STEP_MIN, self.buscar_step * 0.50)
            self.buscar_start_time = time.time()
            self.buscar_eval_pending = False
            self.buscar_err_before = None
            self.buscar_reversals_streak += 1
            self.n_buscar_reverse += 1
            return self._r(
                f"BUSCAR-reinicio d={self.buscar_dir:+.0f} paso={self.buscar_step:.2f} FPerr={err_abs:.4f}"
            )

        # Espera mínima tras un movimiento.
        if self.cnt < N_SETTLE_BUSCAR:
            return self._r("BUSCAR-settle")

        # Evaluar el último movimiento en términos de |FP - objetivo|.
        if self.buscar_eval_pending and self.fresh_after_move:
            before = self.buscar_err_before if self.buscar_err_before is not None else err_abs
            delta = err_abs - before

            if delta < -BUSCAR_EVAL_MARGIN_FP:
                # Mejoró: conservar dirección y permitir solo pequeño aumento.
                self.buscar_step = min(BUSCAR_STEP_MAX, self.buscar_step * BUSCAR_INCREASE_GOOD)
                self.buscar_reversals_streak = 0
                resultado = f"mejora Δ|eFP|={delta:+.5f}"
            elif delta > BUSCAR_EVAL_MARGIN_FP:
                # Empeoró: reversa inmediata y reducción fuerte.
                self.buscar_dir *= -1.0
                self.buscar_step = max(BUSCAR_STEP_MIN, self.buscar_step * BUSCAR_REDUCE_BAD)
                self.buscar_reversals_streak += 1
                self.n_buscar_reverse += 1
                resultado = f"reversa Δ|eFP|={delta:+.5f}"
            else:
                # Muy poca señal: reducir para aumentar resolución.
                self.buscar_step = max(BUSCAR_STEP_MIN, self.buscar_step * BUSCAR_NEUTRO_FACTOR)
                resultado = f"neutro Δ|eFP|={delta:+.5f}"

            self.buscar_eval_pending = False
            self.buscar_err_before = None
            self.fresh_after_move = False
            self._ajustar_paso()

            if abs(self.fp_error_filt) <= FP_PLL_ENTRY_ERR:
                self._cambiar("PLL")
                return self._r(
                    f"BUSCAR→PLL ({resultado}) FP={self.fp_fresh:.4f}",
                    fresh=True
                )

            # Cuatro reversiones consecutivas indican que el gradiente local no es
            # fiable; cambia de dirección y vuelve a un paso moderado sin ir al límite.
            if self.buscar_reversals_streak >= MAX_BUSCAR_REVERSALS_STREAK:
                self.buscar_dir *= -1.0
                self.buscar_step = max(BUSCAR_STEP_MIN, min(self.buscar_step, 1.0))
                self.buscar_reversals_streak = 0
                self.n_buscar_reverse += 1

        else:
            self._ajustar_paso()

        # La pendiente directa dFP/dfase_cmd propone una dirección local. Solo se
        # usa como guía cuando la magnitud es suficiente; BUSCAR la valida después.
        if abs(self.fp_slope) >= FP_SLOPE_MIN_ABS and err_abs < 0.12:
            dir_modelo = -math.copysign(1.0, self.fp_error_filt / self.fp_slope)
            if dir_modelo != self.buscar_dir:
                # No cambiar de rumbo gratuitamente si el último movimiento mejoró;
                # solo corregir la dirección cuando hay evidencia de empeoramiento.
                if self.buscar_reversals_streak > 0:
                    self.buscar_dir = dir_modelo

        d = self.buscar_dir * self.buscar_step
        objetivo = self.fase_cmd + d
        if objetivo < FASE_BUSCAR_MIN or objetivo > FASE_BUSCAR_MAX:
            self.buscar_dir *= -1.0
            self.buscar_step = max(BUSCAR_STEP_MIN, self.buscar_step * BUSCAR_REDUCE_BAD)
            self.n_buscar_reverse += 1
            d = self.buscar_dir * self.buscar_step

        self.buscar_err_before = err_abs
        cambio = self._mover_fase(d, limitar_buscar=True)
        if cambio:
            self.buscar_eval_pending = True
            self.n_buscar_moves += 1
            return self._r(
                f"BUSCAR φcmd {d:+.2f}° FP={self.fp_fresh:.4f} eFP={self.fp_error:+.5f}",
                cambio_fase=True,
                fresh=True,
            )

        self.buscar_dir *= -1.0
        self.buscar_step = max(BUSCAR_STEP_MIN, self.buscar_step * BUSCAR_REDUCE_BAD)
        self.n_buscar_reverse += 1
        return self._r("BUSCAR-límite/reversa", fresh=True)

    def _pll(self, fresh, stats):
        if not fresh:
            return self._r(
                f"PLL-stale FP={self.fp_fresh:.4f} e={self.fp_error_filt:+.5f}",
                fresh=False,
            )

        err = self.fp_error
        err_f = self.fp_error_filt
        slope = self.fp_slope
        if abs(slope) < FP_SLOPE_MIN_ABS:
            slope = math.copysign(FP_SLOPE_MIN_ABS, self.fp_slope if self.fp_slope else FP_SLOPE_INIT)

        # P: conversión directa de error FP a grados de fase.
        e_cmd = -err_f / slope
        dP = KP_FP * e_cmd

        # I: solo cerca del objetivo. Compensa sesgo estacionario sin acumular
        # rápidamente cuando el gradiente local todavía es incierto.
        if abs(err_f) <= INTEGRAL_ENABLE_ERR:
            self.integral_cmd += KI_FP * e_cmd * max(min(self.dt_since_fresh, 1.0), 0.05)
            self.integral_cmd = clamp(
                self.integral_cmd, -INTEGRAL_CMD_LIMIT, INTEGRAL_CMD_LIMIT
            )
        else:
            self.integral_cmd *= INTEGRAL_LEAK

        d = clamp(dP + self.integral_cmd, -FP_TRIM_MAX_DEG, FP_TRIM_MAX_DEG)

        if abs(err_f) <= FP_TRIM_DEADBAND:
            return self._r(
                f"PLL FP={self.fp_fresh:.5f} e={err:+.5f} (I={self.integral_cmd:+.3f})",
                fresh=True,
            )

        if self.cooldown_trim > 0 or abs(d) < FP_TRIM_MIN_DEG:
            return self._r(
                f"PLL espera FP={self.fp_fresh:.5f} e={err:+.5f}",
                fresh=True,
            )

        if self._mover_fase(d, limitar_buscar=False):
            stats.registrar_trim(d)
            # Fuga casi nula: conservar la compensación de sesgo.
            self.integral_cmd *= INTEGRAL_RESET_TRIM
            self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
            return self._r(
                f"PLL φ{d:+.3f}° FP={self.fp_fresh:.5f} e={err_f:+.5f} "
                f"sFP={slope:+.5f} I={self.integral_cmd:+.3f}",
                cambio_fase=True,
                fresh=True,
            )

        return self._r("PLL sin movimiento", fresh=True)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.29-008 — Objetivo FP={FP_OBJETIVO} "
          f"(φ_obj diagnóstico={PHI_OBJETIVO:.4f}°)")
    print("=" * 78)

    fg = None
    wt = None
    ctrl = ControladorFP()
    stats = Stats()
    total = 0
    t_ctrl = None

    try:
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        fg.extreme(
            canal=1,
            frecuencia_hz=FREC_NOMINAL,
            amplitud_vpp=AMPLITUD_FG,
            offset_v=OFFSET_V_FG,
            fase_grados=0.0,
            encender_salida=True,
        )
        wt.extreme(
            elemento_entrada=ELEMENTO_WT,
            incluir_potencias=True,
            configurar_salida=True,
        )

        print(
            f"\n{'t (s)':>6} | {'FP Medido':>10} | {'|FP-0.05|':>10} | "
            f"{'φ med':>9} | {'Modo':>7} | {'Acción':>34}"
        )
        print("-" * 105)

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

            try:
                if wt.is_outlier():
                    stats.n_outliers += 1
                    time.sleep(INTERVALO_MUESTREO)
                    continue
            except Exception:
                pass

            fp_med = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med = m.get("frecuencia")

            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None else abs(np.cos(np.radians(phi_med)))
            total += 1

            res = ctrl.actualizar(fp_abs, phi_med, dt, stats)

            # Frecuencia fija.
            try:
                fg.establecer_frecuencia_extreme(1, FREC_NOMINAL)
            except Exception:
                pass

            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt)

            print(
                f"{int(t_now - t_ctrl):>6} | {fp_abs:>10.5f} | "
                f"{abs(fp_abs - FP_OBJETIVO):>10.5f} | "
                f"{phi_med:>9.2f} | {res['modo']:>7} | {res['accion']:>34}"
            )

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        if total > 0 and t_ctrl is not None:
            stats.resumen(ctrl)
    finally:
        if fg:
            try:
                fg.desconectar()
            except Exception:
                pass
        if wt:
            try:
                wt.desconectar()
            except Exception:
                pass


if __name__ == "__main__":
    main()
