"""CONTROL DE FP v11.13 — Optimización basada en Estadísticas v11.12
=====================================================================
Cambios clave aplicados tras el análisis de la corrida v11.12:
  1. Corrección del bucle RECOVER: Se ajustan umbrales (60°/30°) y se limita
     el paso máximo de recuperación para evitar oscilación atrapada.
  2. Mitigación de cacería (67 salidas): Se aumenta el cooldown en FINE y LOCK 
     y se aplica una atenuación progresiva del trim cerca de cero.
  3. Tolerancia de Outliers Ampliada (45°): Evita ignorar mediciones válidas
     durante cambios legítimos de fase.
  4. Suavizado del Integrador en LOCK: Evita el windup dentro de la banda muerta
     reduciendo la derivación y deriva de fase residual.
"""
import sys
import time
from pathlib import Path
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

from estadisticas import solicitar_estadisticas, finalizar_seguro

# ==================== CONFIGURACIÓN GENERAL ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 100
INTERVALO_MUESTREO = 0.09

FP_OBJETIVO  = 1.0
PHI_OBJETIVO = 0.0
FP_OBJETICO  = FP_OBJETIVO

FP_MIN_RANGO, FP_MAX_RANGO = 0.95, 1.00
AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# ==================== ZONAS DE ERROR Y GANANCIAS ====================
ZONE_COARSE   = 25.0
ZONE_APPROACH = 8.0
ZONE_FINE     = 2.0

# --- Trims máximos por zona (Ajustados para prevenir sobreimpulso) ---
TRIM_MAX_COARSE   = 3.5
TRIM_MAX_APPROACH = 1.5
TRIM_MAX_FINE     = 0.8
TRIM_MAX_LOCK     = 0.20

# --- Cooldowns (Aumentados ligeramente para mejorar la estabilidad) ---
COOLDOWN_COARSE   = 3
COOLDOWN_APPROACH = 4
COOLDOWN_FINE     = 6
COOLDOWN_LOCK     = 10

# --- Deadband con histéresis ---
DEADBAND_IN_FINE  = 0.40
DEADBAND_OUT_FINE = 0.80
DEADBAND_IN_LOCK  = 0.20
DEADBAND_OUT_LOCK = 0.50

ALPHA_ERR = 0.30

# --- Integradores de frecuencia ---
KI_FREQ_APPROACH = 0.0030
KI_FREQ_FINE     = 0.0015
KI_FREQ_LOCK     = 0.0004

KP_FREQ_APPROACH = 0.0020
KP_FREQ_FINE     = 0.0005

DF_MAX_APPROACH = 0.06
DF_MAX_FINE     = 0.020
DF_MAX_LOCK     = 0.010

TRIM_MIN_APPLY = 0.05

# ==================== PROBE / CALIB ====================
CALIB_PASO = 4.0
CALIB_SETTLE_S = 0.5
SLOPE_INIT, SLOPE_MIN_ABS = -0.7, 0.20
SLOPE_ACCEPT_MIN, SLOPE_ACCEPT_MAX = 0.05, 4.0
SLOPE_EMA_W = 0.15

# ==================== BUSCAR ====================
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 30.0, 18.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 3, 3
PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 5.0, 1.5, 8.0
N_SETTLE_BUSCAR = 4

# ==================== RECOVER (Ajustado para evitar atrapamiento) ====================
PHI_RECOVER_ENTER = 60.0    # Incrementado para no entrar falsamente
PHI_RECOVER_EXIT  = 30.0
PASO_RECOVER_MAX  = 10.0    # Reducido para evitar oscilaciones descontroladas

# ==================== FILTRADO / OUTLIERS ====================
DF_LP = 0.25
STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.25
MAX_OUTLIERS_CONSEC = 5
OUTLIER_JUMP = 45.0         # Ampliado para capturar dinámicas rápidas
DELTA_MIN_MOVER = 0.05

# ==================== OUTLIERS DE FP ====================
FP_OUTLIER_JUMP = 0.25
N_FP_OUTLIER_CONSEC = 3


def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== RESUMEN ESTÁNDAR ====================
class Stats:
    BANDS = [0.01, 0.02, 0.03, 0.05]

    def __init__(self):
        self.fp_hist, self.fp_err_hist, self.phi_err_hist = [], [], []
        self.times_mode = {"BUSCAR": 0.0, "RECOVER": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = self.n_stale = self.n_outliers = 0
        self.n_fp_outliers = 0
        self.n_trims = 0
        self.trims_phi_delta = []
        self.t_in_band    = {b: 0.0  for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}
        self.t_zone = {"COARSE": 0.0, "APPROACH": 0.0,
                       "FINE": 0.0, "LOCK": 0.0}

    def registrar(self, fp, phi_err, modo, dt, zona=None):
        self.t_total += dt
        self.n_total += 1
        fp_abs = abs(fp)
        fp_err = abs(fp_abs - FP_OBJETIVO)
        self.fp_hist.append(fp_abs)
        self.fp_err_hist.append(fp_err)
        self.phi_err_hist.append(abs(phi_err))
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt
        if zona is not None:
            self.t_zone[zona] = self.t_zone.get(zona, 0.0) + dt
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
        print(f" RESUMEN v11.13 — Objetivo FP={FP_OBJETIVO}")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  "
              f"outliers={self.n_outliers}  fp_outliers={self.n_fp_outliers}")
        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            print(f"[ Control FP ]  media={media_fp:.4f}  std={np.std(self.fp_hist):.4f}  "
                  f"sesgo={media_fp - FP_OBJETIVO:+.4f}  "
                  f"|FP-{FP_OBJETIVO}| med={np.median(self.fp_err_hist):.4f}  "
                  f"trims={self.n_trims}")
            if self.trims_phi_delta:
                print(f"[ Trims ]  |dφ| medio={np.mean(self.trims_phi_delta):.3f}°  "
                      f"max={np.max(self.trims_phi_delta):.3f}°")

        print(f"\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            print(f"   {m:<8} : {t:6.1f}s ({(t/max(self.t_total,1e-6))*100:5.1f}%)")

        print(f"\n[ Distribución por Zona ]")
        for z, t in self.t_zone.items():
            print(f"   {z:<8} : {t:6.1f}s ({(t/max(self.t_total,1e-6))*100:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.2f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.02] / max(self.t_total, 1e-6)
        v = ("EXCELENTE" if pct_t >= 80 else "BUENO" if pct_t >= 50 else "AJUSTAR")
        print(f"\n[ Veredicto ]  {v}  (±0.02: {pct_t:.1f}%)")
        print("=" * 78)


# ==================== CONTROLADOR v11.13 ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.df_integral = 0.0
        self.modo = "CALIB"
        self.zona = "COARSE"
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(0.0)
        self.error_filt = self.error_fresh
        self.phi_stale = True
        self.slope = SLOPE_INIT
        self.slope_samples = 0
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
        self.in_deadband = False

        self.calib_state = 0
        self.calib_phi_base = 0.0
        self.calib_phi_pos = 0.0
        self.calib_phi_neg = 0.0
        self.calib_t_move = 0.0
        self.calib_baseline_done = False

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": FREC_NOMINAL + self.delta_f,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "zona": self.zona,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "error_filt": self.error_filt,
                "fresh": fresh, "stale": self.phi_stale,
                "slope": self.slope or 0.0}

    def _mover_fase(self, delta):
        if abs(delta) < DELTA_MIN_MOVER:
            return False
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
            self.phi_fresh = phi_w
            self.error_fresh = error_fase(phi_w)
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
        self.phi_fresh = phi_w
        self.error_fresh = error_fase(phi_w)
        self.error_filt = ((1 - ALPHA_ERR) * self.error_filt + ALPHA_ERR * self.error_fresh)
        self.phi_stale = False
        self.last_fresh_time = now
        if self.move_pending:
            self.fresh_after_move = True
        return True

    def _slope_teorico(self, err_deg):
        a = abs(err_deg)
        if a < 1e-9:
            return 1e-6
        return max(abs(np.sin(a * np.pi / 180.0)) * np.pi / 180.0, 1e-6)

    def _slope_efectivo(self, err=None):
        if err is None:
            err = self.error_filt
        st = self._slope_teorico(err) * (-1.0)
        if abs(self.slope) < 1e-6:
            return SLOPE_INIT
        if self.slope_samples < 5:
            return 0.5 * self.slope + 0.5 * SLOPE_INIT
        if abs(self.slope) < 2.0 * abs(st):
            return 0.5 * self.slope + 0.5 * st
        return self.slope

    def _slope(self):
        if not self.move_pending:
            return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False
            return
        if time.time() - self.move_time < SLOPE_SETTLE_SEC:
            return
        if not self.fresh_after_move:
            return
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 0.5 or abs(dphi) < 0.05:
            self.move_pending = False
            return
        sn = dphi / dfase
        if not (SLOPE_ACCEPT_MIN <= abs(sn) <= SLOPE_ACCEPT_MAX):
            self.move_pending = False
            return
        if self.slope_samples >= 3 and self.slope != 0:
            if (sn > 0) != (self.slope > 0):
                self.move_pending = False
                return
        self.slope = (1 - SLOPE_EMA_W) * self.slope + SLOPE_EMA_W * sn
        self.slope_samples += 1
        self.move_pending = False

    def _cambiar(self, modo):
        if modo == self.modo:
            return
        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

    def _calib(self, fresh):
        if not fresh:
            return self._r("CALIB-espera")

        if self.calib_state == 0:
            self.calib_phi_base = self.phi_fresh
            self.calib_state = 1
            self.calib_t_move = time.time()
            self._mover_fase(+CALIB_PASO)
            return self._r(f"CALIB +{CALIB_PASO:.0f}°")

        if self.calib_state == 1:
            if time.time() - self.calib_t_move < CALIB_SETTLE_S:
                return self._r("CALIB-settle+")
            self.calib_phi_pos = self.phi_fresh
            self.calib_t_move = time.time()
            self._mover_fase(-CALIB_PASO)
            self.calib_state = 2
            return self._r(f"CALIB -{CALIB_PASO:.0f}°")

        if self.calib_state == 2:
            if time.time() - self.calib_t_move < CALIB_SETTLE_S:
                return self._r("CALIB-settle-")
            self.calib_phi_neg = self.phi_fresh

            dphi = envolver_fase(self.calib_phi_pos - self.calib_phi_neg)
            dtheta = 2 * CALIB_PASO
            if abs(dphi) > 0.3:
                sn = dphi / dtheta
                if SLOPE_ACCEPT_MIN <= abs(sn) <= SLOPE_ACCEPT_MAX:
                    self.slope = sn
                    self.slope_samples = 10
                    self.calib_done = True
                    print(f"   [CALIB] slope = {sn:+.4f} "
                          f"(φ_base={self.calib_phi_base:+.2f}°, "
                          f"φ+={self.calib_phi_pos:+.2f}°, "
                          f"φ-={self.calib_phi_neg:+.2f}°)")
                else:
                    print(f"   [CALIB] slope fuera de rango: {sn:+.4f}, usando SLOPE_INIT")
            else:
                print(f"   [CALIB] sin señal suficiente (Δφ={dphi:+.3f}°), usando SLOPE_INIT")

            delta_back = self.calib_phi_base - self.phi_fresh
            self._mover_fase(delta_back)
            self.calib_state = 3
            self.calib_t_move = time.time()
            return self._r(f"CALIB-restore {delta_back:+.2f}°")

        if self.calib_state == 3:
            if time.time() - self.calib_t_move < CALIB_SETTLE_S:
                return self._r("CALIB-settle-final")
            aerr = abs(self.error_filt)
            if aerr > PHI_RECOVER_ENTER:
                self._cambiar("RECOVER")
            elif aerr > PHI_BUSCAR_ENTER:
                self._cambiar("BUSCAR")
            else:
                self._cambiar("PLL")
            self.calib_state = 4
            return self._r(f"CALIB done -> {self.modo}")

        return self._r("CALIB-ok")

    def actualizar(self, fp, phi_med, dt, stats):
        if phi_med is None:
            return self._r("SIN_PHI")
        fresh = self._phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh = 0.0 if fresh else self.dt_since_fresh + dt
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1

        if fresh:
            self.cooldown_trim = max(0, self.cooldown_trim - 1)

        self._slope()

        if self.modo == "CALIB":
            return self._calib(fresh)

        if fresh:
            self.fresh_enter_buscar = (self.fresh_enter_buscar + 1
                                       if abs(self.error_filt) > PHI_BUSCAR_ENTER else 0)
            self.fresh_exit_buscar = (self.fresh_exit_buscar + 1
                                      if abs(self.error_filt) < PHI_BUSCAR_EXIT else 0)

        if self.modo != "RECOVER" and abs(self.error_filt) > PHI_RECOVER_ENTER:
            self._cambiar("RECOVER")
        elif (self.modo == "RECOVER" and abs(self.error_filt) < PHI_RECOVER_EXIT):
            if abs(self.error_filt) > PHI_BUSCAR_ENTER:
                self._cambiar("BUSCAR")
            else:
                self._cambiar("PLL")

        if self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            self._cambiar("BUSCAR")
        elif self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            self._cambiar("PLL")

        if self.modo == "RECOVER":
            return self._recover(fresh)
        if self.modo == "BUSCAR":
            return self._buscar(fresh)
        return self._pll(fresh, stats)

    def _recover(self, fresh):
        if not fresh:
            return self._r("RECOVER-stale")
        if self.cnt < N_SETTLE_BUSCAR:
            return self._r("RECOVER-settle")
        self.delta_f = 0.0
        err = self.error_fresh
        s = self._slope_efectivo(err)
        d = -err / s if abs(s) > SLOPE_MIN_ABS else -np.sign(err) * PASO_RECOVER_MAX
        d = clamp(d, -PASO_RECOVER_MAX, PASO_RECOVER_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"RECOVER φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _buscar(self, fresh):
        if not fresh:
            return self._r("BUSCAR-stale")
        if self.cnt < N_SETTLE_BUSCAR:
            return self._r("BUSCAR-settle")
        self.delta_f = 0.0
        err = self.error_fresh
        s = self._slope_efectivo(err)
        d = -err / s if abs(s) > SLOPE_MIN_ABS else -np.sign(err) * self.paso_buscar
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _clasificar_zona(self, aerr):
        if aerr > ZONE_COARSE:
            return "COARSE"
        if aerr > ZONE_APPROACH:
            return "APPROACH"
        if aerr > ZONE_FINE:
            return "FINE"
        return "LOCK"

    def _parametros_zona(self, zona):
        return {
            "COARSE":   dict(trim=TRIM_MAX_COARSE,   cd=COOLDOWN_COARSE,
                             ki=KI_FREQ_APPROACH, kp_f=KP_FREQ_APPROACH,
                             dfm=DF_MAX_APPROACH, db_in=0.0, db_out=0.0),
            "APPROACH": dict(trim=TRIM_MAX_APPROACH, cd=COOLDOWN_APPROACH,
                             ki=KI_FREQ_APPROACH, kp_f=KP_FREQ_APPROACH,
                             dfm=DF_MAX_APPROACH, db_in=0.0, db_out=0.0),
            "FINE":     dict(trim=TRIM_MAX_FINE,     cd=COOLDOWN_FINE,
                             ki=KI_FREQ_FINE,     kp_f=KP_FREQ_FINE,
                             dfm=DF_MAX_FINE,     db_in=DEADBAND_IN_FINE,
                             db_out=DEADBAND_OUT_FINE),
            "LOCK":     dict(trim=TRIM_MAX_LOCK,     cd=COOLDOWN_LOCK,
                             ki=KI_FREQ_LOCK,     kp_f=0.0,
                             dfm=DF_MAX_LOCK,     db_in=DEADBAND_IN_LOCK,
                             db_out=DEADBAND_OUT_LOCK),
        }[zona]

    def _pll(self, fresh, stats):
        self.zona = self._clasificar_zona(abs(self.error_filt))
        P = self._parametros_zona(self.zona)

        if not fresh:
            return self._r(f"PLL-{self.zona}-stale df={self.delta_f:+.5f}")

        aerr = abs(self.error_filt)
        db_in, db_out = P["db_in"], P["db_out"]
        if db_in > 0:
            if self.in_deadband:
                if aerr > db_out:
                    self.in_deadband = False
            else:
                if aerr < db_in:
                    self.in_deadband = True

        s = self._slope_efectivo(self.error_filt)
        if abs(s) < 1e-6:
            s = SLOPE_INIT

        trim_applied = False
        d = 0.0
        if (not self.in_deadband) and self.cooldown_trim == 0:
            d_raw = -self.error_filt / s
            d = clamp(d_raw, -P["trim"], P["trim"])
            if abs(self.error_filt) < ZONE_FINE * 2:
                d *= 0.60  # Reducción suave para evitar rebotes en FINE
            if abs(d) > TRIM_MIN_APPLY:
                self._mover_fase(d)
                stats.registrar_trim(d)
                self.cooldown_trim = P["cd"]
                trim_applied = True
                self.df_integral *= 0.85

        if not trim_applied:
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            err_int = -self.error_filt / s if abs(s) > 1e-6 else 0.0
            if aerr > (db_in if db_in > 0 else 0.0):
                self.df_integral += P["ki"] * err_int * eff_dt
                self.df_integral = clamp(self.df_integral, -P["dfm"], P["dfm"])
            else:
                self.df_integral *= 0.95  # Drenaje más activo en deadband

            df_out = self.df_integral
            if P["kp_f"] > 0:
                df_out += P["kp_f"] * err_int

            self.delta_f = clamp(
                (1 - DF_LP) * self.delta_f + DF_LP * df_out,
                -P["dfm"], P["dfm"]
            )

        if trim_applied:
            return self._r(f"PLL-{self.zona}-trim φ{d:+.2f}°", cambio_fase=True, fresh=True)
        return self._r(f"PLL-{self.zona} df={self.delta_f:+.5f}", fresh=fresh)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.13 — Objetivo FP={FP_OBJETIVO} (Estabilidad optimizada)")
    print("=" * 78)

    stats_robustas = solicitar_estadisticas(
        fp_target=FP_OBJETIVO, tolerancia=0.03, tolerancia_fina=0.02
    )

    fg = wt = None
    ctrl = ControladorFP()
    stats = Stats()
    total = 0
    t_ctrl = None

    fp_prev_ok = None
    fp_outlier_consec = 0

    try:
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=0.0, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True,
                   configurar_salida=True)

        print(f"\n{'t (s)':>6} | {'FP':>8} | {'|FP-1|':>8} | {'Modo':>8} | "
              f"{'Zona':>9} | {'Err':>8} | {'Acción':>26}")
        print("-" * 92)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
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
            fp_med = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med = m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None \
                else abs(np.cos(np.radians(phi_med)))

            if fp_prev_ok is not None:
                if abs(fp_abs - fp_prev_ok) > FP_OUTLIER_JUMP:
                    fp_outlier_consec += 1
                    stats.n_fp_outliers += 1
                    if fp_outlier_consec < N_FP_OUTLIER_CONSEC:
                        time.sleep(INTERVALO_MUESTREO)
                        continue
                else:
                    fp_outlier_consec = 0
            fp_prev_ok = fp_abs

            total += 1
            res = ctrl.actualizar(fp_med, phi_med, dt, stats)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt,
                            zona=res.get("zona"))

            if stats_robustas is not None:
                stats_robustas.agregar(t_rel, fp_abs,
                                       res.get("error_fase", 0.0),
                                       modo=res["modo"])

            print(f"{int(t_rel):>6} | {fp_abs:>8.4f} | "
                  f"{abs(fp_abs - FP_OBJETICO):>8.4f} | {res['modo']:>8} | "
                  f"{res.get('zona',''):>9} | {res['error_fase']:>+8.4f} | "
                  f"{res['accion']:>26}")

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario (Ctrl+C).")
        try:
            stats.resumen(ctrl)
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