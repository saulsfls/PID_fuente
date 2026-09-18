"""CONTROL DE FP v11.0-0.2 — Compacto, enfocado en alcanzar FP=0.2"""
import time
import numpy as np
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==================== CONFIG ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.05

PHI_OBJETIVO = 78.46
FP_OBJETIVO  = 0.2
FP_MIN_RANGO, FP_MAX_RANGO  = 0.17, 0.23
FP_TIGHT_LOW, FP_TIGHT_HIGH = 0.19, 0.21
PHI_TIGHT = 0.58

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 50.0, 15.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 3, 2
PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 20.0, 8.0, 30.0
N_SETTLE_BUSCAR = 20

KP_PLL, KI_PLL = 0.008, 0.0010
DF_MAX, DF_LP  = 0.15, 0.45

STALE_TOL, STALE_FORCE_SEC = 0.15, 0.6
EFF_DT_MAX, SLOPE_SETTLE_SEC = 1.5, 0.30
MAX_OUTLIERS_CONSEC = 5

PHI_TRIM_DEADBAND, PHI_TRIM_MAX, PHI_TRIM_MIN = 3.0, 12.0, 0.5
N_FRESH_COOLDOWN_TRIM = 3

CALIB_PASO = 6.0
SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -0.7, 0.20, 20
OUTLIER_JUMP = 35.0
MIN_FRESH_SINCE_LAST = 15.0
DELTA_MIN_MOVER = 0.2


def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS (foco FP=0.2) ====================
class Stats:
    BANDS = [0.01, 0.02, 0.03, 0.05]      # tolerancias |FP - 0.2|

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
        self.run_tight_cur = self.run_tight_max = 0.0
        self.runs_tight = []
        self.prev_sign = 0
        self.n_zerocross = 0

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
        # Racha en banda tight
        if FP_TIGHT_LOW <= fp_abs <= FP_TIGHT_HIGH:
            self.run_tight_cur += dt
            self.run_tight_max = max(self.run_tight_max, self.run_tight_cur)
        else:
            if self.run_tight_cur > 0:
                self.runs_tight.append(self.run_tight_cur)
                self.run_tight_cur = 0.0
        # Zero-crossings del error (indicador de oscilación)
        s = 1 if phi_err > 0 else (-1 if phi_err < 0 else 0)
        if s != 0 and self.prev_sign != 0 and s != self.prev_sign:
            self.n_zerocross += 1
        if s != 0:
            self.prev_sign = s

    def registrar_trim(self, dphi):
        self.n_trims += 1
        self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl):
        if self.run_tight_cur > 0:
            self.runs_tight.append(self.run_tight_cur)
        print("\n" + "=" * 78)
        print(f" RESUMEN FINAL v11.0 — Objetivo FP={FP_OBJETIVO}")
        print("=" * 78)

        print(f"\n[ Datos ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"tasa={self.n_total/max(self.t_total,1e-6):.1f}/s  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  "
              f"outliers={self.n_outliers}")
        if self.fp_hist:
            print(f"[ FP ]     media={np.mean(self.fp_hist):.4f}  "
                  f"std={np.std(self.fp_hist):.4f}  "
                  f"|FP-0.2| med={np.median(self.fp_err_hist):.4f}  "
                  f"p90={np.percentile(self.fp_err_hist, 90):.4f}")
        if self.phi_err_hist:
            print(f"[ φ err ]  |Δφ| media={np.mean(self.phi_err_hist):.2f}°  "
                  f"mediana={np.median(self.phi_err_hist):.2f}°")

        print(f"\n[ Tiempo en banda |FP-0.2| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"  ±{b:.2f}   {pct:5.1f}%  1er: {t1_str}   "
                  f"{'#' * int(pct / 2)}")

        med_tight = np.median(self.runs_tight) if self.runs_tight else 0.0
        print(f"\n[ Racha tight {FP_TIGHT_LOW}-{FP_TIGHT_HIGH} ]  "
              f"N={len(self.runs_tight)}  "
              f"max={self.run_tight_max:.2f}s  mediana={med_tight:.2f}s")

        print(f"\n[ Modo ]   BUSCAR={self.times_mode.get('BUSCAR', 0):.1f}s  "
              f"PLL={self.times_mode.get('PLL', 0):.1f}s")

        slope_str = f"{ctrl.slope:+.3f} ({ctrl.slope_samples})" if ctrl.slope else "N/A"
        print(f"\n[ Control ]  trims={self.n_trims}  "
              f"zerocross={self.n_zerocross}  slope={slope_str}")
        if self.trims_phi_delta:
            print(f"[ Trims ]   |Δφ| medio={np.mean(self.trims_phi_delta):.2f}°  "
                  f"máx={np.max(self.trims_phi_delta):.2f}°")

        if self.fp_hist:
            print(f"\n[ Distribución |FP| ]")
            edges = [0.0, 0.14, 0.16, 0.18, 0.19, 0.20, 0.21, 0.22, 0.24, 0.30, 1.01]
            counts = np.zeros(len(edges) - 1, dtype=int)
            for fp in self.fp_hist:
                for i in range(len(edges) - 1):
                    if edges[i] <= fp < edges[i + 1]:
                        counts[i] += 1
                        break
                else:
                    counts[-1] += 1
            for i in range(len(edges) - 1):
                pct = 100 * counts[i] / len(self.fp_hist)
                print(f"  [{edges[i]:.2f}-{edges[i+1]:.2f})  "
                      f"{counts[i]:5d} ({pct:5.1f}%)  {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.02] / max(self.t_total, 1e-6)
        pct_l = 100 * self.t_in_band[0.05] / max(self.t_total, 1e-6)
        v = ("EXCELENTE" if pct_t >= 70 else "BUENO"     if pct_t >= 50 else
             "ACEPTABLE" if pct_l >= 50 else "POBRE"     if pct_l >= 25 else "FALLO")
        print(f"\n[ Veredicto ]  {v}   (±0.02: {pct_t:.1f}% | ±0.05: {pct_l:.1f}%)")
        print("=" * 78)


# ==================== CONTROLADOR v10.6 (compacto) ====================
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
        self.mejor_phi_abs = 180.0
        self.fresh_since_trim = 999
        self.calib_done = False
        self.move_time = 0.0
        self.last_fresh_time = time.time()
        self.n_outliers_consec = 0

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": FREC_NOMINAL + self.delta_f,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale,
                "slope": self.slope or 0.0}

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
        is_outlier = step_prev > OUTLIER_JUMP
        if is_outlier:
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
        if not is_outlier and not force_fresh and step_fresh < STALE_TOL:
            self.phi_stale = True
            return False
        self.phi_fresh, self.error_fresh = phi_w, error_fase(phi_w)
        self.phi_stale = False
        self.last_fresh_time = now
        self.n_outliers_consec = 0
        if self.move_pending:
            self.fresh_after_move = True
        return True

    def _slope(self):
        if not self.move_pending: return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False; return
        if time.time() - self.move_time < SLOPE_SETTLE_SEC: return
        if not self.fresh_after_move: return
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 3.0 or abs(dphi) < 0.3:
            self.move_pending = False; return
        sn = dphi / dfase
        if not (0.05 <= abs(sn) <= 5.0):
            self.move_pending = False; return
        if self.slope is None:
            self.slope, self.slope_samples = sn, 1
        else:
            w = max(1.0 / (self.slope_samples + 1), 0.15)
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
        if phi_med is None:
            return self._r("SIN_PHI")
        fresh = self._phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh = 0.0 if fresh else self.dt_since_fresh + dt
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1
        if fresh and abs(self.error_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.error_fresh)
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
        if self.slope and abs(self.slope) > SLOPE_MIN_ABS:
            d = -err / self.slope
            if abs(err) > PHI_BUSCAR_EXIT:
                d = np.sign(d) * max(abs(d), PASO_BUSCAR_MIN)
        else:
            d = self.dir_buscar * self.paso_buscar
        d = clamp(d, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, stats):
        if fresh:
            s = self.slope if (self.slope and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            err = self.error_fresh
            self.fresh_since_trim += 1
            if (abs(err) > PHI_TRIM_DEADBAND
                    and self.fresh_since_trim >= N_FRESH_COOLDOWN_TRIM):
                d = clamp(-err / s, -PHI_TRIM_MAX, PHI_TRIM_MAX)
                if abs(d) > PHI_TRIM_MIN:
                    self._mover_fase(d)
                    self.fresh_since_trim = 0
                    stats.registrar_trim(d)
                    return self._r(f"PLL-trim φ{d:+.1f}°",
                                   cambio_fase=True, fresh=True)
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            err_int = -err / s
            self.df_integral = clamp(
                self.df_integral + KI_PLL * err_int * eff_dt, -DF_MAX, DF_MAX)
            df_p = clamp(KP_PLL * err_int, -DF_MAX, DF_MAX)
            df_out = df_p + self.df_integral
            self.delta_f = clamp((1 - DF_LP) * self.delta_f + DF_LP * df_out,
                                 -DF_MAX, DF_MAX)
        return self._r(f"PLL df={self.delta_f:+.5f}", fresh=fresh)


# ==================== LOGGING ====================
def print_row(t, fp_abs, res):
    mark = "*" if res.get("fresh") else ("-" if res.get("stale") else " ")
    print(f"{t:>4} | {fp_abs:>7.4f} | {res['phi']:>+8.2f} | "
          f"{res['error_fase']:>+7.2f} |{mark}| "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+9.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>7} | "
          f"{res['slope']:>+6.3f} | {res['accion']:>20}")


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v11.0 — Objetivo FP={FP_OBJETIVO} (φ_obj={PHI_OBJETIVO:.2f}°)")
    print("=" * 78)

    fg = wt = None
    ctrl = ControladorFP()
    stats = Stats()
    total = 0
    t_ctrl = None

    try:
        print("\n[1/3] Conectando equipos (modo EXTREME)...")
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()
        print(f"  FG420:  {fg.obtener_idn()[:60]}")
        print(f"  WT3000: {wt.obtener_idn()[:60]}")
        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=0.0, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True,
                   configurar_salida=True)
        print("  EXTREME OK. Estabilizando 5s...")
        for _ in range(10):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print(" OK")

        print("\n[3/3] INICIANDO CONTROL\n")
        print("-" * 122)
        print(f"{'t':>4} | {'|FP|':>7} | {'PHI':>8} | {'errφ':>7} |*| "
              f"{'Fase':>8} | {'Δf':>9} | {'FrecFG':>9} | {'Modo':>7} | "
              f"{'Slope':>6} | {'Acción':>20}")
        print("-" * 122)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            try:
                m = wt.leer_mediciones_minimas()
            except Exception as e:
                print(f"[X] {e}")
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
            fp_abs = (abs(fp_med) if fp_med is not None
                      else abs(np.cos(np.radians(phi_med))))
            total += 1
            res = ctrl.actualizar(fp_med, phi_med, dt, stats)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt)

            if res.get("fresh") or total % 8 == 0:
                print_row(int(t_now - t_ctrl), fp_abs, res)

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(ctrl)
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