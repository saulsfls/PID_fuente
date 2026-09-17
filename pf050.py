"""CONTROL DE FP v10.2 - PLL hacia FP=0.5 con trim, calibración y red de seguridad"""
import time
import numpy as np
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ------------------------------------------------------------------ CONFIG
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.05

# ---- Objetivo: FP = 0.5  ⇒  φ = ±60° ----
PHI_OBJETIVO   = 60.0
FP_OBJETIVO    = 0.5
FP_MIN_RANGO, FP_MAX_RANGO   = 0.47, 0.53
FP_TIGHT_LOW, FP_TIGHT_HIGH  = 0.49, 0.51

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 50.0, 25.0
N_FRESH_ENTER_BUSCAR, N_FRESH_EXIT_BUSCAR = 3, 4

PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 20.0, 8.0, 30.0
N_SETTLE_BUSCAR = 20                          # v10.2: era 6

KP_PLL, KI_PLL = 0.012, 0.0015
DF_MAX, DF_LP, EFF_DT_MAX = 0.30, 0.45, 0.5

PHI_TRIM_DEADBAND     = 3.5
PHI_TRIM_MAX          = 12.0
PHI_TRIM_MIN          = 0.5
N_FRESH_COOLDOWN_TRIM = 3
CALIB_PASO            = 6.0

SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = -0.8, 0.25, 20
STALE_TOL, OUTLIER_JUMP, PHI_LOCKED = 0.08, 35.0, 5.0

# ---- v10.2: red de seguridad ----
MIN_FRESH_SINCE_LAST  = 5.0     # s — si no hay fresca en este tiempo, congelar
STUCK_FREQ_WINDOW     = 100     # muestras
STUCK_FREQ_TOL        = 1e-5    # Hz — rango < esto ⇒ frecuencia "congelada"
HEALTH_PRINT_PERIOD   = 10.0    # s
HEALTH_WARMUP         = 15.0    # s — no penalizar al arranque


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def error_fase(phi_deg):
    return envolver_fase(phi_deg - PHI_OBJETIVO)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ------------------------------------------------------------- RUNTRACKER
class RunTracker:
    def __init__(self, name=""):
        self.name = name
        self.cur_n = self.cur_t = 0
        self.max_n = self.max_t = 0
        self.tot_in_n = self.tot_in_t = 0
        self.tot_n = 0
        self.tot_t = 0.0
        self.runs_t = []

    def update(self, cond, dt):
        self.tot_n += 1
        self.tot_t += dt
        if cond:
            self.cur_n += 1
            self.cur_t += dt
            self.tot_in_n += 1
            self.tot_in_t += dt
            self.max_n = max(self.max_n, self.cur_n)
            self.max_t = max(self.max_t, self.cur_t)
        else:
            self._close()

    def _close(self):
        if self.cur_n > 0:
            self.runs_t.append(self.cur_t)
            self.cur_n = self.cur_t = 0

    def close(self):
        self._close()

    def summary(self):
        n = len(self.runs_t)
        pct = 100 * self.tot_in_t / max(self.tot_t, 1e-6)
        return {"n": n, "max_time": self.max_t, "max_samples": self.max_n,
                "mean_time": float(np.mean(self.runs_t)) if n else 0.0,
                "median_time": float(np.median(self.runs_t)) if n else 0.0,
                "pct_time": pct, "total_time_in": self.tot_in_t}

    def histogram(self, bins):
        h = np.zeros(len(bins) - 1, dtype=int)
        for t in self.runs_t:
            for i in range(len(bins) - 1):
                if bins[i] <= t < bins[i + 1]:
                    h[i] += 1
                    break
            else:
                if t >= bins[-1]:
                    h[-1] += 1
        return h


# ---------------------------------------------------------- HEALTHMONITOR
class HealthMonitor:
    """Vigila la calidad de la cadena de medida.

    Detecta:
      - frescas demasiado escasas (la última fresca hace > MIN_FRESH_SINCE_LAST s)
      - frecuencia congelada (rango < STUCK_FREQ_TOL en las últimas STUCK_FREQ_WINDOW)
    """

    def __init__(self):
        self.t0 = time.time()
        self.last_fresh_t = None
        self.freq_window = []
        self.stuck_freq = False
        self.low_fresh = False
        self._last_print = 0.0

    def update(self, fresh, f_med):
        now = time.time()
        if fresh:
            self.last_fresh_t = now
        if f_med is not None:
            self.freq_window.append(float(f_med))
            if len(self.freq_window) > STUCK_FREQ_WINDOW:
                self.freq_window = self.freq_window[-STUCK_FREQ_WINDOW:]

    def status(self):
        now = time.time()
        reasons = []
        if self.last_fresh_t is None:
            if now - self.t0 > HEALTH_WARMUP:
                reasons.append("NUNCA_FRESCA")
        elif now - self.last_fresh_t > MIN_FRESH_SINCE_LAST:
            reasons.append(f"SIN_FRESCA({now-self.last_fresh_t:.1f}s)")
        self.low_fresh = any("FRESCA" in r for r in reasons)

        self.stuck_freq = False
        if len(self.freq_window) >= STUCK_FREQ_WINDOW:
            arr = np.array(self.freq_window[-STUCK_FREQ_WINDOW:])
            if (arr.max() - arr.min()) < STUCK_FREQ_TOL:
                self.stuck_freq = True
                reasons.append("FREC_CONGELADA")

        return (len(reasons) == 0, reasons)

    def maybe_print(self):
        now = time.time()
        if now - self._last_print < HEALTH_PRINT_PERIOD:
            return
        self._last_print = now
        healthy, reasons = self.status()
        if not healthy:
            print(f"  [!] SALUD: {' | '.join(reasons)}  "
                  f"(frescas_ult30s≈{self._count_recent_fresh():.1f})")
        else:
            print(f"  [✓] SALUD OK  (freq std últimos {len(self.freq_window)}: "
                  f"{np.std(self.freq_window):.5f} Hz)")

    def _count_recent_fresh(self):
        if self.last_fresh_t is None:
            return 0
        # No tenemos timestamps de todas las frescas, proxy: 1 si la última es reciente
        return 1.0 if (time.time() - self.last_fresh_t) < 5.0 else 0.0


# ------------------------------------------------------------ ESTADISTICAS
class Estadisticas:
    def __init__(self):
        self.fp_hist, self.phi_hist, self.phi_err_hist = [], [], []
        self.fase_hist, self.frec_hist, self.df_hist = [], [], []
        self.tiempos_modo = {"BUSCAR": 0.0, "PLL": 0.0}
        self.bins_fp = np.zeros(10)
        self.n_transiciones = self.n_stale = self.n_fresh = self.n_outliers = 0
        self.n_trims = 0
        self.n_congelados = 0
        self.run_fp_range = RunTracker(
            f"FP en rango [{FP_MIN_RANGO:.2f}, {FP_MAX_RANGO:.2f}]")
        self.run_fp_tight = RunTracker(
            f"FP tight [{FP_TIGHT_LOW:.2f}, {FP_TIGHT_HIGH:.2f}]")
        self.run_phi_lock = RunTracker(f"|φ - φ_obj| < {PHI_LOCKED:.0f}°")
        self.run_out = RunTracker("FP fuera de rango")
        self.tiempo_primer_lock = self.tiempo_primer_tight = None

    def registrar(self, fp, phi, phi_err, fase, frec, df, modo, dt):
        self.fp_hist.append(fp)
        self.phi_hist.append(phi)
        self.phi_err_hist.append(phi_err)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec)
        self.df_hist.append(df)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        self.bins_fp[min(int(abs(fp) * 10), 9)] += 1
        in_range = FP_MIN_RANGO <= abs(fp) <= FP_MAX_RANGO
        self.run_fp_range.update(in_range, dt)
        self.run_fp_tight.update(FP_TIGHT_LOW <= abs(fp) <= FP_TIGHT_HIGH, dt)
        self.run_phi_lock.update(abs(phi_err) < PHI_LOCKED, dt)
        self.run_out.update(not in_range, dt)

    def cerrar_rachas(self):
        for r in (self.run_fp_range, self.run_fp_tight,
                  self.run_phi_lock, self.run_out):
            r.close()

    def _imprimir_rachas(self, tr, bins):
        s = tr.summary()
        print(f"\n  ▸ {tr.name}")
        print(f"      Nº rachas:        {s['n']}")
        if s["n"] > 0:
            print(f"      Racha más larga:  {s['max_time']:.2f} s ({s['max_samples']} muestras)")
            print(f"      Media/mediana:    {s['mean_time']:.2f} / {s['median_time']:.2f} s")
        print(f"      Tiempo total:     {s['total_time_in']:.1f} s ({s['pct_time']:.1f}%)")
        if s["n"] > 0 and tr.runs_t:
            h = tr.histogram(bins)
            print("      Distribución:")
            for i in range(len(bins) - 1):
                lo, hi = bins[i], bins[i + 1]
                label = (f"      >{lo:>6.1f}s" if hi == np.inf else
                         f"      <{hi:>6.1f}s" if lo == 0 else
                         f"   {lo:>5.1f}-{hi:<5.1f}s")
                print(f"        {label} : {h[i]:3d}  {'#' * min(h[i], 40)}")

    def resumen(self, controlador, t_total, n_iter, en_rango):
        self.cerrar_rachas()
        print("\n" + "=" * 90)
        print(" RESUMEN FINAL v10.2 — Control hacia FP=0.5 (φ_obj = "
              f"{PHI_OBJETIVO:+.1f}°)")
        print("=" * 90)
        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:        {t_total:.1f} s")
        print(f"  Iteraciones:         {n_iter}")
        print(f"  Tasa efectiva:       {n_iter/max(t_total,1e-6):.1f} muestras/s")
        print(f"  Frescas / stale:     {self.n_fresh} / {self.n_stale}")
        print(f"  Outliers rechazados: {self.n_outliers}")
        print(f"  Iteraciones 'ciegas':{self.n_congelados}")
        if self.fp_hist:
            print(f"  FP prom / std:       {np.mean(self.fp_hist):.4f} / "
                  f"{np.std(self.fp_hist):.4f}")
        if self.phi_hist:
            print(f"  φ (crudo) prom/med:  {np.mean(self.phi_hist):+.2f}° / "
                  f"{np.median(self.phi_hist):+.2f}°")
        if self.phi_err_hist:
            print(f"  |φ-φ_obj| prom/med:  {np.mean(np.abs(self.phi_err_hist)):.2f}° / "
                  f"{np.median(np.abs(self.phi_err_hist)):.2f}°")
        if n_iter > 0:
            print(f"\n[ Efectividad ]")
            print(f"  En rango [{FP_MIN_RANGO:.2f}, {FP_MAX_RANGO:.2f}]: "
                  f"{en_rango} ({100*en_rango/n_iter:.1f}%)")
        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            print(f"  {m:<10} {t:7.1f}s ({pct:5.1f}%)  {'#' * int(pct / 2)}")
        n_tot = self.bins_fp.sum()
        if n_tot > 0:
            print(f"\n[ Distribución de |FP| ]")
            for i in range(9, -1, -1):
                pct = 100 * self.bins_fp[i] / n_tot
                lo = i * 0.1
                print(f"  [{lo:.1f}-{lo+0.1:.1f}] {int(self.bins_fp[i]):5d} "
                      f"({pct:5.1f}%)  {'#' * int(pct / 2)}")
        print(f"\n[ Rachas de permanencia ]")
        bins_t = [0, 0.5, 2.0, 5.0, 15.0, 30.0, 60.0, np.inf]
        for r in (self.run_fp_range, self.run_fp_tight,
                  self.run_phi_lock, self.run_out):
            self._imprimir_rachas(r, bins_t)
        print(f"\n[ Trazabilidad ]")
        print(f"  Mejor FP:            {controlador.mejor_fp:.4f}")
        print(f"  Mejor |φ-φ_obj|:     {controlador.mejor_phi_abs:.3f}°")
        print(f"  Fase en mejor punto: {controlador.mejor_fase:+.2f}°")
        if self.tiempo_primer_lock is not None:
            print(f"  1er lock (|Δφ|<{PHI_LOCKED:.0f}°): {self.tiempo_primer_lock:.2f} s")
        if self.tiempo_primer_tight is not None:
            print(f"  1er FP tight:        {self.tiempo_primer_tight:.2f} s")
        print(f"\n[ PLL ]")
        print(f"  Kp={KP_PLL:.5f}  Ki={KI_PLL:.6f}  |Δf|max={DF_MAX:.3f} Hz")
        if self.df_hist:
            print(f"  Δf prom/std/max:     {np.mean(self.df_hist):+.5f} / "
                  f"{np.std(self.df_hist):.5f} / "
                  f"{np.max(np.abs(self.df_hist)):.5f} Hz")
        print(f"  Cambios de fase:     {controlador.n_cambios_fase}")
        print(f"  Trims de fase:       {self.n_trims}")
        print(f"  Transiciones:        {self.n_transiciones}")
        if controlador.slope is not None:
            print(f"  Slope (dφ/dφ_cmd):   {controlador.slope:+.3f} "
                  f"({controlador.slope_samples} act.)")
        if self.frec_hist:
            uf = self.frec_hist[-100:]
            print(f"\n[ Frecuencia de red medida ]")
            print(f"  Promedio (últimas):  {np.mean(uf):.4f} Hz")
            print(f"  Desviación:          {np.std(uf):.5f} Hz")
            print(f"  Rango global:        {np.min(self.frec_hist):.4f} - "
                  f"{np.max(self.frec_hist):.4f} Hz")
        print(f"\n[ Veredicto ]")
        if n_iter > 0:
            ef = 100 * en_rango / n_iter
            v = ("EXCELENTE" if ef >= 80 else "ACEPTABLE" if ef >= 50 else
                 "POBRE" if ef >= 20 else "FALLO")
            print(f"  {v} ({ef:.1f}% en banda FP≈0.5)")
        print("=" * 90)


# ------------------------------------------------------- CONTROLADOR v10.2
class ControladorFPv10:
    """v10.2: como v10.1 + parámetro `healthy` para congelar si la medida no es fiable."""

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
        self.stale_run = 0
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
        self.mejor_fase = 0.0
        self.mejor_fp = 0.0
        self.fresh_since_trim = 999
        self.calib_done = False

    def _resp(self, accion, f_fg, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": f_fg, "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale,
                "mejor_fp": self.mejor_fp, "mejor_fase": self.mejor_fase,
                "mejor_phi_abs": self.mejor_phi_abs,
                "slope": self.slope if self.slope is not None else 0.0,
                "paso": self.paso_buscar}

    def _mover_fase(self, delta):
        if abs(delta) < 0.3:
            return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = clamp(envolver_fase(self.fase_cmd + delta),
                              FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        self.move_pending = True
        self.fresh_after_move = False
        self.cnt = 0
        return True

    def _actualizar_phi(self, phi_raw_deg, stats=None):
        phi_w = envolver_fase(phi_raw_deg)
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.error_fresh = error_fase(phi_w)
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.stale_run = 0
            return True
        if abs(phi_w - self.phi_prev_raw) < STALE_TOL:
            self.stale_run += 1
            self.phi_stale = True
            return False
        jump = abs(envolver_fase(phi_w - self.phi_fresh))
        if jump > OUTLIER_JUMP:
            if stats is not None:
                stats.n_outliers += 1
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            return False
        self.phi_fresh = phi_w
        self.error_fresh = error_fase(phi_w)
        self.phi_prev_raw = phi_w
        self.phi_stale = False
        self.stale_run = 0
        if self.move_pending:
            self.fresh_after_move = True
        return True

    def _actualizar_slope(self):
        if not (self.move_pending and self.fresh_after_move):
            return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False
            return
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 3.0 or abs(dphi) < 0.3:
            self.move_pending = False
            return
        slope_new = dphi / dfase
        if abs(slope_new) > 5.0 or abs(slope_new) < 0.05:
            self.move_pending = False
            return
        if self.slope is None:
            self.slope, self.slope_samples = slope_new, 1
        else:
            w = max(1.0 / (self.slope_samples + 1), 0.15)
            self.slope = (1 - w) * self.slope + w * slope_new
            self.slope_samples += 1
        self.move_pending = False

    def _cambiar_modo(self, nuevo, stats):
        if nuevo == self.modo:
            return
        self.modo = nuevo
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        if stats is not None:
            stats.n_transiciones += 1

    def actualizar(self, fp_med, phi_med, f_med, dt, stats=None, healthy=True):
        if phi_med is None:
            return self._resp("SIN_PHI", FREC_NOMINAL + self.delta_f)
        fresh = self._actualizar_phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh += dt
        if fresh:
            self.dt_since_fresh = 0.0
            if stats is not None:
                stats.n_fresh += 1
        elif stats is not None:
            stats.n_stale += 1
        fp_abs = abs(fp_med) if fp_med is not None \
            else abs(np.cos(np.radians(self.phi_fresh)))
        if fresh and abs(self.error_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.error_fresh)
            self.mejor_fase = self.fase_cmd
            self.mejor_fp = fp_abs
        self._actualizar_slope()
        if fresh:
            self.fresh_enter_buscar = (
                self.fresh_enter_buscar + 1
                if abs(self.error_fresh) > PHI_BUSCAR_ENTER else 0)
            self.fresh_exit_buscar = (
                self.fresh_exit_buscar + 1
                if abs(self.error_fresh) < PHI_BUSCAR_EXIT else 0)
        if self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            self._cambiar_modo("BUSCAR", stats)
        elif self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            self._cambiar_modo("PLL", stats)

        # v10.2: si el sistema no está sano, no comandamos movimientos
        if not healthy:
            if stats is not None:
                stats.n_congelados += 1
            return self._resp(f"FREEZE ({self.modo})",
                              FREC_NOMINAL + self.delta_f, fresh=fresh)
        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh, stats)

    def _buscar(self, fresh):
        if not fresh:
            return self._resp("BUSCAR-stale", FREC_NOMINAL + self.delta_f)
        if self.cnt < N_SETTLE_BUSCAR:
            return self._resp("BUSCAR-settle", FREC_NOMINAL + self.delta_f)
        self.delta_f = 0.0
        if not self.calib_done:
            delta = self.dir_buscar * CALIB_PASO
            cambio = self._mover_fase(delta)
            if cambio:
                self.calib_done = True
            return self._resp(f"CALIB φ{delta:+.0f}°", FREC_NOMINAL,
                              cambio_fase=cambio, fresh=True)
        err = self.error_fresh
        if self.slope is not None and abs(self.slope) > SLOPE_MIN_ABS:
            delta = -err / self.slope
            delta = np.sign(delta) * max(abs(delta), self.paso_buscar)
        else:
            delta = self.dir_buscar * self.paso_buscar
        delta = clamp(delta, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(delta)
        return self._resp(f"BUSCAR φ{delta:+.0f}°", FREC_NOMINAL,
                          cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, stats=None):
        if fresh:
            s = self.slope if (self.slope is not None
                               and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            error = self.error_fresh
            self.fresh_since_trim += 1
            if (abs(error) > PHI_TRIM_DEADBAND
                    and self.fresh_since_trim >= N_FRESH_COOLDOWN_TRIM):
                delta_fase = clamp(-error / s, -PHI_TRIM_MAX, PHI_TRIM_MAX)
                if abs(delta_fase) > PHI_TRIM_MIN:
                    self._mover_fase(delta_fase)
                    self.fresh_since_trim = 0
                    self.df_integral = 0.0
                    self.delta_f = 0.0
                    if stats is not None:
                        stats.n_trims += 1
                    return self._resp(f"PLL-trim φ{delta_fase:+.1f}°",
                                      FREC_NOMINAL, cambio_fase=True, fresh=True)
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            err = -error / s
            self.df_integral = clamp(
                self.df_integral + KI_PLL * err * eff_dt, -DF_MAX, DF_MAX)
            df_p = clamp(KP_PLL * err, -DF_MAX, DF_MAX)
            df_out = df_p + self.df_integral
            self.delta_f = clamp((1 - DF_LP) * self.delta_f + DF_LP * df_out,
                                 -DF_MAX, DF_MAX)
        return self._resp(f"PLL df={self.delta_f:+.5f}",
                          FREC_NOMINAL + self.delta_f, fresh=fresh)


# ------------------------------------------------------------------ LOGGING
def encabezado():
    print("-" * 175)
    print(f"{'t':>4} | {'|FP|':>7} | {'PHI':>8} | {'errφ':>7} | {'*':>1} | "
          f"{'Fase':>8} | {'Δf':>10} | {'FrecFG':>9} | {'Modo':>7} | "
          f"{'Slope':>7} | {'dfint':>10} | {'err':>8} | {'MejorFP':>7} | "
          f"{'Acción':>18}")
    print("-" * 175)


def fila(t, fp_med, res, df_integral=0.0, err_val=0.0):
    mark = "*" if res.get("fresh") else ("-" if res.get("stale") else " ")
    print(f"{t:>4} | {fp_med:>7.4f} | {res['phi']:>+8.2f} | "
          f"{res['error_fase']:>+7.2f} | {mark:>1} | "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+10.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>7} | "
          f"{res['slope']:>+7.3f} | {df_integral:>+10.5f} | "
          f"{err_val:>+8.2f} | {res['mejor_fp']:>7.4f} | {res['accion']:>18}")


# --------------------------------------------------------------------- MAIN
def main():
    print("=" * 175)
    print(" CONTROL FP v10.2 - FP=0.5 + trim + calibración + red de seguridad  "
          f"(φ_obj = {PHI_OBJETIVO:+.1f}°)")
    print("=" * 175)
    print(f" Muestreo objetivo: {1/INTERVALO_MUESTREO:.0f} Hz")
    print(f" Kp={KP_PLL:.5f} Ki={KI_PLL:.6f} |Δf|max={DF_MAX:.3f} Hz  DF_LP={DF_LP}")
    print(f" φ_objetivo={PHI_OBJETIVO:+.1f}°  ⇒  FP_objetivo≈{FP_OBJETIVO:.3f}")
    print(f" Banda FP: [{FP_MIN_RANGO:.2f}, {FP_MAX_RANGO:.2f}]  "
          f"tight [{FP_TIGHT_LOW:.2f}, {FP_TIGHT_HIGH:.2f}]")
    print(f" Trim: deadband={PHI_TRIM_DEADBAND}°  max={PHI_TRIM_MAX}°  "
          f"cooldown={N_FRESH_COOLDOWN_TRIM}")
    print(f" N_SETTLE_BUSCAR={N_SETTLE_BUSCAR}  Calibración inicial={CALIB_PASO}°")
    print(f" Red de seguridad: congelar si no hay fresca en {MIN_FRESH_SINCE_LAST}s "
          f"o frecuencia congelada (rango<{STUCK_FREQ_TOL} Hz en {STUCK_FREQ_WINDOW} m)")
    print("=" * 175)

    fg = wt = None
    controlador = ControladorFPv10()
    stats = Estadisticas()
    health = HealthMonitor()
    total = en_rango = 0
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
        print("  Modo EXTREME aplicado.")

        print("\n[2/3] Estabilizando 5 s...")
        for _ in range(10):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print(" OK")

        print("\n[3/3] INICIANDO CONTROL\n")
        encabezado()
        t_ctrl = t_prev = time.time()

        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
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
            en_rango_local = FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO
            if en_rango_local:
                en_rango += 1

            # v10.2: evaluar salud ANTES de decidir si mover
            # (usamos el estado de la última lectura fresca conocida)
            healthy, reasons = health.status()

            res = controlador.actualizar(fp_med, phi_med, f_med, dt, stats,
                                         healthy=healthy)

            # v10.2: actualizar health monitor con esta iteración
            health.update(res.get("fresh", False), f_med)
            health.maybe_print()

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])
            stats.registrar(fp_abs, res["phi"], res["error_fase"], res["fase"],
                            res["frecuencia"], res["delta_f"], res["modo"], dt)
            t_rel = t_now - t_ctrl
            if stats.tiempo_primer_lock is None and \
               abs(res["error_fase"]) < PHI_LOCKED:
                stats.tiempo_primer_lock = t_rel
            if stats.tiempo_primer_tight is None and \
               FP_TIGHT_LOW <= fp_abs <= FP_TIGHT_HIGH:
                stats.tiempo_primer_tight = t_rel
            if res.get("fresh") or total % 8 == 0:
                s = controlador.slope if controlador.slope else SLOPE_INIT
                err_val = -res["error_fase"] / s if abs(s) > SLOPE_MIN_ABS else 0.0
                fila(int(t_rel), fp_abs, res,
                     controlador.df_integral, err_val)
            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)
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