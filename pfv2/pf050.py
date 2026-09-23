"""CONTROL DE FP v11.11 — Optimización de sesgo y ajuste fino de amortiguamiento (Objetivo: PF = 0.5)"""
import sys
import time
from pathlib import Path
import numpy as np

# Agregar directorio padre al path (un nivel arriba del script)
sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==================== CONFIGURACIÓN Y CONSTANTES ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.09  # ~11 muestras/s para frescura óptima en GPIB

# Configuración optimizada para Factor de Potencia = 0.5
FP_OBJETIVO  = 0.5
PHI_OBJETIVO = 60.0   # cos(60°) = 0.5
FP_OBJETICO  = FP_OBJETIVO  # Alias preventivo por compatibilidad

FP_MIN_RANGO, FP_MAX_RANGO  = 0.45, 0.55
FP_TIGHT_LOW, FP_TIGHT_HIGH = 0.48, 0.52
PHI_TIGHT = 0.58

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# Umbrales y pasos optimizados para enganchar PLL en FP = 0.5 (φ = 60°)
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 25.0, 12.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 3, 3
PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 12.0, 3.0, 15.0
N_SETTLE_BUSCAR = 8

# Ganancias y parámetros dinámicos optimizados v11.11 (Mayor amortiguamiento)
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -0.7, 0.25, 25
KP_PLL, KI_PLL = 0.0032, 0.0038    # KP reducido para suavizar sobrepasos en φ=60°
DF_MAX, DF_LP  = 0.10, 0.20        # Mayor filtrado LP para compensar el retardo de lecturas GPIB

STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.25
MAX_OUTLIERS_CONSEC = 5

# Banda muerta ajustada v11.11 (Ajuste fino de sesgo)
PHI_TRIM_DEADBAND, PHI_TRIM_MAX, PHI_TRIM_MIN = 0.45, 7.0, 0.15  # Deadband y trim mínimo reducidos
N_FRESH_COOLDOWN_TRIM = 3                                         # Cooldown más ágil

CALIB_PASO = 5.0
OUTLIER_JUMP = 30.0
DELTA_MIN_MOVER = 0.15


def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS Y DIAGNÓSTICO ====================
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

    def resumen(self, ctrl):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNÓSTICO Y ESTADÍSTICAS v11.11 — Objetivo FP={FP_OBJETIVO}")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")
        
        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            print(f"[ Control FP ]  media={media_fp:.4f}  std={np.std(self.fp_hist):.4f}  "
                  f"sesgo (offset)={sesgo:+.4f}  |FP-{FP_OBJETIVO}| med={np.median(self.fp_err_hist):.4f}")

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
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else "BUENO" if pct_t >= 50 else "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±0.02: {pct_t:.1f}%)")
        print("=" * 78)


# ==================== CONTROLADOR v11.11 ====================
class ControladorFP:
    def __init__(self):
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
            self.fresh_enter_buscar = (self.fresh_enter_buscar + 1 if abs(self.error_fresh) > PHI_BUSCAR_ENTER else 0)
            self.fresh_exit_buscar = (self.fresh_exit_buscar + 1 if abs(self.error_fresh) < PHI_BUSCAR_EXIT else 0)
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
        d = -err / self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) else -np.sign(err) * self.paso_buscar
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, stats):
        if fresh:
            s = self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            err = self.error_fresh
            
            # Trim de fase para correcciones directas finas
            if abs(err) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
                d = clamp(-err / s, -PHI_TRIM_MAX, PHI_TRIM_MAX)
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    stats.registrar_trim(d)
                    self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                    return self._r(f"PLL-trim φ{d:+.1f}°", cambio_fase=True, fresh=True)
                    
            # Control por Frecuencia con Zona Muerta de Integración (Anti-windup)
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            err_int = -err / s
            
            # Congela la acumulación integral si el error de fase es menor a 0.2° para evitar sesgos
            if abs(err) > 0.2:
                self.df_integral = clamp(self.df_integral + KI_PLL * err_int * eff_dt, -DF_MAX, DF_MAX)
                
            df_out = KP_PLL * err_int + self.df_integral
            self.delta_f = clamp((1 - DF_LP) * self.delta_f + DF_LP * df_out, -DF_MAX, DF_MAX)
        return self._r(f"PLL df={self.delta_f:+.5f}", fresh=fresh)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.11 — Objetivo FP={FP_OBJETIVO} (φ_obj={PHI_OBJETIVO:.2f}°)")
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
        
        print(f"\n{'t (s)':>6} | {'FP Medido':>10} | {'|FP-0.5|':>10} | {'Modo':>7} | {'Acción':>20}")
        print("-" * 65)

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

            print(f"{int(t_now - t_ctrl):>6} | {fp_abs:>10.4f} | {abs(fp_abs - FP_OBJETICO):>10.4f} | {res['modo']:>7} | {res['accion']:>20}")

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