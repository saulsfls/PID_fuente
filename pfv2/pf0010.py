# ============================================================================
#  ControladorFP v11.24 — FP=0.01
#  Feedforward de frecuencia + filtro exponencial de fase + tuning intermedio
# ============================================================================

import sys
import time
from pathlib import Path
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000
from estadisticas import solicitar_estadisticas, finalizar_seguro

# ==================== CONFIGURACIÓN Y CONSTANTES v11.24 ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 100
INTERVALO_MUESTREO = 0.09

FP_OBJETIVO  = 0.01
PHI_OBJETIVO = 89.4271

FP_MIN_RANGO, FP_MAX_RANGO  = 0.005, 0.030
FP_TIGHT_LOW, FP_TIGHT_HIGH = 0.008, 0.012
PHI_TIGHT = 0.115

TOL_OBJETIVO = 0.002
TOL_FINA     = 0.001

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# --- Feedforward de frecuencia ---
FF_ENABLE     = True
FF_GAIN       = 1.00
FF_LIMIT_HZ   = 0.50
ALPHA_FREQ    = 0.10
N_FREQ_WARMUP = 5

# --- NUEVO v11.24: filtro exponencial de fase ---
PHI_FILTER_ALPHA = 0.40        # 0.4 → std reducida ~2×

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER = 2.50
PHI_BUSCAR_EXIT  = 0.80
N_FRESH_ENTER_BUSCAR = 3
N_FRESH_EXIT_BUSCAR  = 3

# --- Comportamiento del modo BUSCAR ---
PASO_BUSCAR_INICIAL = 4.0
PASO_BUSCAR_MIN     = 0.50
PASO_BUSCAR_MAX     = 2.00
N_SETTLE_BUSCAR     = 3

# --- Pendiente ---
SLOPE_FIXED_BUSCAR = -1.00
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -1.00, 0.30, 25
SLOPE_CLIP_LOW, SLOPE_CLIP_HIGH = -1.50, -0.50
SLOPE_ADAPT_W = 0.15

# --- Control PI de fase (v11.24: punto medio entre v11.22 y v11.23) ---
KP_PHI_VEL = 0.50              # v11.22=0.80, v11.23=0.35
KI_PHI_VEL = 0.40              # v11.22=0.80, v11.23=0.25
INTEGRAL_CMD_LIMIT   = 1.20    # v11.22=2.50, v11.23=0.80
INTEGRAL_ENABLE_BAND = 1.20    # v11.22=2.00, v11.23=0.80

# --- Trim de fase (v11.24: deadband > 2× ruido filtrado) ---
PHI_TRIM_DEADBAND     = 0.10   # v11.22=0.15, v11.23=0.08
PHI_TRIM_MAX          = 0.50   # v11.22=1.20, v11.23=0.35
PHI_TRIM_MIN          = 0.03   # v11.22=0.05, v11.23=0.02
N_FRESH_COOLDOWN_TRIM = 1      # v11.22=1,   v11.23=2

# --- Filtros y temporización ---
STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.25
MAX_OUTLIERS_CONSEC = 5

OUTLIER_JUMP    = 30.0
DELTA_MIN_MOVER = 0.03

def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS LOCALES ====================
class Stats:
    BANDS = [0.001, 0.002, 0.005, 0.010]

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
        if f_red is not None:
            self.f_red_hist.append(f_red)
        if delta_f is not None:
            self.delta_f_hist.append(delta_f)

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl=None):
        print("\n" + "=" * 78)
        print(f" RESUMEN v11.24 (filtro de fase + tuning intermedio) — FP={FP_OBJETIVO}")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            mediana_fp = np.median(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            print(f"[ Control FP ]  media={media_fp:.5f}  mediana={mediana_fp:.5f}  "
                  f"std={np.std(self.fp_hist):.5f}  sesgo={sesgo:+.5f}")
            print(f"                |FP-{FP_OBJETIVO}| med={np.median(self.fp_err_hist):.5f}  "
                  f"p90={np.percentile(self.fp_err_hist, 90):.5f}")

        if self.f_red_hist:
            print(f"[ Feedforward ]  f_red media={np.mean(self.f_red_hist):.4f} Hz  "
                  f"(std {np.std(self.f_red_hist)*1000:.1f} mHz)")
            print(f"                 Δf medio={np.mean(self.delta_f_hist)*1000:+.2f} mHz  "
                  f"(std {np.std(self.delta_f_hist)*1000:.1f} mHz)")

        print(f"\n[ Distribución de Modos ]")
        for m, t in self.times_mode.items():
            pct_m = (t / max(self.t_total, 1e-6)) * 100
            print(f"   {m:<8} : {t:6.1f}s ({pct_m:5.1f}%)")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   ±{b:.4f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.002] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else
             "BUENO" if pct_t >= 60 else
             "ACEPTABLE" if pct_t >= 40 else
             "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (±0.002: {pct_t:.1f}%)")
        print(f"[ Trims aplicados ]  n={self.n_trims}  "
              f"|Δφ| medio={(np.mean(self.trims_phi_delta) if self.trims_phi_delta else 0.0):.3f}°  "
              f"(tolerancia útil: {PHI_TIGHT:.3f}°)")
        if ctrl is not None:
            print(f"[ Pendiente final ]  s={ctrl.slope:+.3f}  (muestras={ctrl.slope_samples})")
            print(f"[ Integrador fase ]  integral_cmd={ctrl.integral_cmd:+.3f}°  "
                  f"(límite ±{INTEGRAL_CMD_LIMIT:.2f})")
            print(f"[ fase_cmd final ]  {ctrl.fase_cmd:+.3f}°")
            print(f"[ delta_f final ]   {ctrl.delta_f*1000:+.2f} mHz")
        print("=" * 78)


# ==================== CONTROLADOR v11.24 ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.f_filt = FREC_NOMINAL
        self.n_freq_actualizaciones = 0
        # --- NUEVO v11.24: filtro de fase ---
        self.phi_filtrado = None
        self.n_phi_filtradas = 0
        # ------------------------------------
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
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        self.move_time = 0.0
        self.last_fresh_time = time.time()
        self.n_outliers_consec = 0
        self.cooldown_trim = 0
        self.integral_cmd = 0.0

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": FREC_NOMINAL + self.delta_f,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale, "slope": self.slope or 0.0,
                "f_red": self.f_filt, "phi_filt": self.phi_filtrado}

    def _actualizar_feedforward(self, f_red):
        if not FF_ENABLE or f_red is None:
            return
        self.f_filt = (1 - ALPHA_FREQ) * self.f_filt + ALPHA_FREQ * f_red
        self.n_freq_actualizaciones += 1
        if self.n_freq_actualizaciones < N_FREQ_WARMUP:
            self.delta_f = 0.0
            return
        raw_delta = FF_GAIN * (self.f_filt - FREC_NOMINAL)
        self.delta_f = clamp(raw_delta, -FF_LIMIT_HZ, FF_LIMIT_HZ)

    def _filtrar_phi(self, phi_w):
        """NUEVO v11.24: filtro exponencial sobre la fase envuelta."""
        if self.phi_filtrado is None:
            self.phi_filtrado = phi_w
            self.n_phi_filtradas = 1
            return phi_w
        # Envolver la diferencia para no mezclar +180 con -180
        delta = envolver_fase(phi_w - self.phi_filtrado)
        self.phi_filtrado = envolver_fase(
            self.phi_filtrado + PHI_FILTER_ALPHA * delta
        )
        self.n_phi_filtradas += 1
        return self.phi_filtrado

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
            phi_filt = self._filtrar_phi(phi_w)
            self.phi_fresh, self.error_fresh = phi_filt, error_fase(phi_filt)
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

        # ---- v11.24: usar fase FILTRADA para el criterio de frescura ----
        phi_filt = self._filtrar_phi(phi_w)

        step_fresh = abs(envolver_fase(phi_filt - self.phi_fresh))
        force_fresh = (now - self.last_fresh_time) > STALE_FORCE_SEC
        if step_fresh < STALE_TOL and not force_fresh:
            self.phi_stale = True
            return False
        self.phi_fresh, self.error_fresh = phi_filt, error_fase(phi_filt)
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
        self.modo = modo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        if modo == "PLL":
            self.integral_cmd = 0.0

    def actualizar(self, fp, phi_med, dt, stats, f_red=None):
        self._actualizar_feedforward(f_red)

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

        err = self.error_fresh
        s = SLOPE_FIXED_BUSCAR
        d = -err / s
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.2f}°", cambio_fase=cambio, fresh=True)

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
                    return self._r(f"PLL φ{d:+.3f}° (e={err:+.3f})",
                                   cambio_fase=True, fresh=True)
        return self._r(f"PLL e={self.error_fresh:+.3f}° (I={self.integral_cmd:+.3f})",
                       fresh=fresh)


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.24 — Objetivo FP={FP_OBJETIVO} (φ={PHI_OBJETIVO:.4f}°)")
    print(f" Feedforward: {'ACTIVO' if FF_ENABLE else 'INACTIVO'}   "
          f"| Filtro φ: α={PHI_FILTER_ALPHA}   "
          f"| KP={KP_PHI_VEL} KI={KI_PHI_VEL}   DB={PHI_TRIM_DEADBAND}")
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

        print(f"\n{'t (s)':>6} | {'FP':>8} | {'|FP-0.01|':>10} | "
              f"{'f_red':>8} | {'Δf (mHz)':>9} | {'Modo':>7} | {'Acción':>32}")
        print("-" * 98)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            t_rel = t_now - t_ctrl

            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO); continue
            if wt.is_outlier():
                continue

            fp_med  = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med   = m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO); continue

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

            print(f"{int(t_rel):>6} | {fp_abs:>8.5f} | "
                  f"{abs(fp_abs - FP_OBJETIVO):>10.5f} | "
                  f"{f_med:>8.4f} | {res['delta_f']*1000:>+9.2f} | "
                  f"{res['modo']:>7} | {res['accion']:>32}")

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario (Ctrl+C).")
        if total > 0 and t_ctrl is not None:
            try:
                stats.resumen(ctrl)
            except Exception as e:
                print(f"[!] Error en resumen: {e}")

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback; traceback.print_exc()
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