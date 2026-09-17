"""
CONTROL DE FP v9.6 - Target FP = 0.50 (PHI_TARGET = 60.0°)
Optimizaciones v9.6:
  1. Conmutación BUSCAR -> PLL con 1 sola lectura fresca en ventana (|PHI - 60°| < 8.0°).
  2. Soft-landing en modo BUSCAR para evitar overshooting cerca de 60°.
  3. Muestreo ajustado a 10 Hz (100 ms) para reducir lecturas stale del WT3000.
"""

import time
import numpy as np
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ------------------------------------------------------------------ CONFIGURACIÓN
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.10  # 10 Hz: Optimizado para la tasa de refresco del WT3000

# TARGET FP = 0.50 -> arccos(0.50) = 60.0°
PHI_TARGET = 60.0  # Ángulo de fase objetivo (+60.0° inductivo)
FP_TARGET = abs(np.cos(np.radians(PHI_TARGET)))  # 0.5000
FP_MIN_RANGO, FP_MAX_RANGO = 0.470, 0.530
FP_TIGHT_TOL = 0.010  # Banda estrecha: [0.490, 0.510]

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# Umbrales para conmutación de modos
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 15.0, 8.0
N_FRESH_ENTER_BUSCAR = 2  # Muestras fuera de tolerancia para volver a BUSCAR
N_FRESH_EXIT_BUSCAR = 1   # 1 lectura fresca en ventana alcanza para ENGANCHAR a PLL

PASO_BUSCAR_INICIAL, PASO_BUSCAR_MIN, PASO_BUSCAR_MAX = 10.0, 1.5, 20.0
N_SETTLE_BUSCAR = 2  # Reducido para acelerar reacción

KP_PLL, KI_PLL = 0.0025, 0.00025
DF_MAX, DF_LP, EFF_DT_MAX = 0.15, 0.25, 0.6
DEADBAND_PHI = 0.6  # Banda muerta angular: ±0.6° en torno a 60°

SLOPE_INIT, SLOPE_MIN_ABS, SLOPE_SAMPLES_INIT = +1.0, 0.20, 20
STALE_TOL, OUTLIER_JUMP, PHI_LOCKED = 0.05, 35.0, 2.5


def envolver_fase(a):
    """Mantiene el ángulo dentro del intervalo [-180.0, +180.0]°"""
    return ((a + 180.0) % 360.0) - 180.0


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
        return {
            "n": n, "max_time": self.max_t, "max_samples": self.max_n,
            "mean_time": float(np.mean(self.runs_t)) if n else 0.0,
            "median_time": float(np.median(self.runs_t)) if n else 0.0,
            "pct_time": pct, "total_time_in": self.tot_in_t
        }

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


# ------------------------------------------------------------ ESTADÍSTICAS
class Estadisticas:
    def __init__(self):
        self.fp_hist, self.phi_hist = [], []
        self.fase_hist, self.frec_hist, self.df_hist = [], [], []
        self.tiempos_modo = {"BUSCAR": 0.0, "PLL": 0.0}
        self.bins_fp = np.zeros(10)
        self.n_transiciones = self.n_stale = self.n_fresh = self.n_outliers = 0
        self.run_fp_range = RunTracker("FP en rango [0.47, 0.53]")
        self.run_fp_tight = RunTracker("FP en rango tight [0.49, 0.51]")
        self.run_phi_lock = RunTracker(f"|PHI - {PHI_TARGET}°| < {PHI_LOCKED}°")
        self.run_out = RunTracker("FP fuera de rango")
        self.tiempo_primer_lock = self.tiempo_primer_tight = None

    def registrar(self, fp, phi, fase, frec, df, modo, dt):
        self.fp_hist.append(fp)
        self.phi_hist.append(phi)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec)
        self.df_hist.append(df)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        self.bins_fp[min(int(abs(fp) * 10), 9)] += 1
        
        in_range = FP_MIN_RANGO <= abs(fp) <= FP_MAX_RANGO
        in_tight = abs(abs(fp) - FP_TARGET) <= FP_TIGHT_TOL
        err_phi = abs(envolver_fase(phi - PHI_TARGET))
        
        self.run_fp_range.update(in_range, dt)
        self.run_fp_tight.update(in_tight, dt)
        self.run_phi_lock.update(err_phi < PHI_LOCKED, dt)
        self.run_out.update(not in_range, dt)

    def cerrar_rachas(self):
        for r in (self.run_fp_range, self.run_fp_tight, self.run_phi_lock, self.run_out):
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
        print(f" RESUMEN FINAL v9.6  [OBJETIVO FP = {FP_TARGET:.2f} / PHI = {PHI_TARGET:.1f}°]")
        print("=" * 90)
        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:        {t_total:.1f} s")
        print(f"  Iteraciones:         {n_iter}")
        print(f"  Tasa efectiva:       {n_iter/max(t_total,1e-6):.1f} muestras/s")
        print(f"  Frescas / stale:     {self.n_fresh} / {self.n_stale}")
        print(f"  Outliers rechazados: {self.n_outliers}")
        if self.fp_hist:
            print(f"  FP prom / std:       {np.mean(self.fp_hist):.4f} / {np.std(self.fp_hist):.4f}")
        if self.phi_hist:
            errs = [abs(envolver_fase(p - PHI_TARGET)) for p in self.phi_hist]
            print(f"  |Error PHI| prom/med:{np.mean(errs):.2f}° / {np.median(errs):.2f}°")
        if n_iter > 0:
            print(f"\n[ Efectividad ]\n  En rango FP ({FP_MIN_RANGO:.2f}-{FP_MAX_RANGO:.2f}): {en_rango} "
                  f"({100*en_rango/n_iter:.1f}%)")
        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            print(f"  {m:<10} {t:7.1f}s ({pct:5.1f}%)  {'#' * int(pct / 2)}")
        print(f"\n[ Rachas de permanencia ]")
        bins_t = [0, 0.5, 2.0, 5.0, 15.0, 30.0, 60.0, np.inf]
        for r in (self.run_fp_range, self.run_fp_tight, self.run_phi_lock, self.run_out):
            self._imprimir_rachas(r, bins_t)
        print(f"\n[ Trazabilidad ]")
        print(f"  Mejor FP (más cercano a {FP_TARGET:.2f}): {controlador.mejor_fp:.4f}")
        print(f"  Mejor |Error PHI|:    {controlador.mejor_phi_err:.3f}°")
        print(f"  Fase en mejor punto: {controlador.mejor_fase:+.2f}°")
        if self.tiempo_primer_lock is not None:
            print(f"  1er lock (|ΔPHI|<{PHI_LOCKED}°): {self.tiempo_primer_lock:.2f} s")
        if self.tiempo_primer_tight is not None:
            print(f"  1er FP en banda tight: {self.tiempo_primer_tight:.2f} s")
        print("=" * 90)


# ------------------------------------------------------- CONTROLADOR v9.6
class ControladorFPv9:
    def __init__(self):
        self.fase_cmd = 60.0
        self.delta_f = 0.0
        self.df_integral = 0.0
        self.modo = "BUSCAR"
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.phi_stale = True
        self.stale_run = 0
        self.slope = SLOPE_INIT
        self.slope_samples = SLOPE_SAMPLES_INIT
        self.fase_prev = None
        self.phi_prev = None
        self.paso_buscar = PASO_BUSCAR_INICIAL
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0
        self.mejor_phi_err = 180.0
        self.mejor_fase = 60.0
        self.mejor_fp = 0.0

    def _resp(self, accion, f_fg, cambio_fase=False, fresh=False):
        return {
            "accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
            "frecuencia": f_fg, "cambio_fase": cambio_fase, "modo": self.modo,
            "phi": self.phi_fresh, "fresh": fresh, "stale": self.phi_stale,
            "mejor_fp": self.mejor_fp, "mejor_fase": self.mejor_fase,
            "mejor_phi_err": self.mejor_phi_err,
            "slope": self.slope if self.slope is not None else 0.0,
            "paso": self.paso_buscar
        }

    def _mover_fase(self, delta):
        if abs(delta) < 0.15:
            return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = clamp(envolver_fase(self.fase_cmd + delta), FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        self.move_pending = True
        self.fresh_after_move = False
        self.cnt = 0
        return True

    def _actualizar_phi(self, phi_raw_deg, stats=None):
        phi_w = envolver_fase(phi_raw_deg)
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
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
        if abs(dfase) < 1.0 or abs(dphi) < 0.2:
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

    def actualizar(self, fp_med, phi_med, f_med, dt, stats=None):
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

        fp_abs = abs(fp_med) if fp_med is not None else abs(np.cos(np.radians(self.phi_fresh)))
        err_phi_actual = abs(envolver_fase(self.phi_fresh - PHI_TARGET))
        
        if fresh and err_phi_actual < self.mejor_phi_err:
            self.mejor_phi_err = err_phi_actual
            self.mejor_fase = self.fase_cmd
            self.mejor_fp = fp_abs

        self._actualizar_slope()

        # Evaluación de conmutación de modos ajustada para lecturas asíncronas
        if fresh:
            if err_phi_actual < PHI_BUSCAR_EXIT:
                self.fresh_exit_buscar += 1
            else:
                self.fresh_exit_buscar = 0

            if err_phi_actual > PHI_BUSCAR_ENTER:
                self.fresh_enter_buscar += 1
            else:
                self.fresh_enter_buscar = 0

        # Conmutación acelerada
        if self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            self._cambiar_modo("PLL", stats)
        elif self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            self._cambiar_modo("BUSCAR", stats)

        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh)

    def _buscar(self, fresh):
        if not fresh:
            return self._resp("BUSCAR-stale", FREC_NOMINAL + self.delta_f)
        if self.cnt < N_SETTLE_BUSCAR:
            return self._resp("BUSCAR-settle", FREC_NOMINAL + self.delta_f)
        self.delta_f = 0.0

        phi_err = envolver_fase(PHI_TARGET - self.phi_fresh)
        s = self.slope if (self.slope is not None and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT

        # Soft-landing: Reduce el paso a medida que se acerca al objetivo
        factor_cercania = clamp(abs(phi_err) / 25.0, 0.1, 1.0)
        delta = (phi_err / s) * factor_cercania

        if abs(delta) < PASO_BUSCAR_MIN and abs(phi_err) > 2.0:
            delta = np.sign(phi_err) * PASO_BUSCAR_MIN

        delta = clamp(delta, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)
        cambio = self._mover_fase(delta)
        return self._resp(f"BUSCAR φ{delta:+.1f}°", FREC_NOMINAL, cambio_fase=cambio, fresh=True)

    def _pll(self, fresh):
        if fresh:
            eff_dt = max(min(self.dt_since_fresh, EFF_DT_MAX), 0.05)
            s = self.slope if (self.slope is not None and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            
            phi_err = envolver_fase(PHI_TARGET - self.phi_fresh)
            
            if abs(phi_err) < DEADBAND_PHI:
                err = 0.0
            else:
                err = phi_err / s

            self.df_integral = clamp(self.df_integral + KI_PLL * err * eff_dt, -DF_MAX, DF_MAX)
            df_p = clamp(KP_PLL * err, -DF_MAX, DF_MAX)
            df_out = df_p + self.df_integral
            
            self.delta_f = clamp((1 - DF_LP) * self.delta_f + DF_LP * df_out, -DF_MAX, DF_MAX)

        return self._resp(f"PLL df={self.delta_f:+.5f}", FREC_NOMINAL + self.delta_f, fresh=fresh)


# ------------------------------------------------------------------ LOGGING
def encabezado():
    print("-" * 160)
    print(f"{'t':>4} | {'FP':>7} | {'PHI':>8} | {'*':>1} | {'Fase':>8} | "
          f"{'Δf':>10} | {'FrecFG':>9} | {'Modo':>7} | {'Slope':>7} | "
          f"{'dfint':>10} | {'err':>8} | {'MejorFP':>7} | {'Acción':>18}")
    print("-" * 160)


def fila(t, fp_med, res, en_rango, df_integral=0.0, err_val=0.0):
    mark = "*" if res.get("fresh") else ("-" if res.get("stale") else " ")
    print(f"{t:>4} | {fp_med:>7.4f} | {res['phi']:>+8.2f} | {mark:>1} | "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+10.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>7} | "
          f"{res['slope']:>+7.3f} | {df_integral:>+10.5f} | "
          f"{err_val:>+8.2f} | {res['mejor_fp']:>7.4f} | {res['accion']:>18}")


# --------------------------------------------------------------------- MAIN
def main():
    print("=" * 160)
    print(f" CONTROL FP v9.6 - BÚSQUEDA Y ESTABILIZACIÓN FP = {FP_TARGET:.2f} (PHI_TARGET = {PHI_TARGET:.1f}°)")
    print("=" * 160)
    print(f" Muestreo objetivo: {1/INTERVALO_MUESTREO:.0f} Hz (100 ms)")
    print(f" Kp={KP_PLL:.5f} Ki={KI_PLL:.6f} |Δf|max={DF_MAX:.3f} Hz | Deadband={DEADBAND_PHI}°")
    print(f" BUSCAR entra si |PHI-{PHI_TARGET}°|>{PHI_BUSCAR_ENTER}°; engancha a PLL si |PHI-{PHI_TARGET}°|<{PHI_BUSCAR_EXIT}°")
    print("=" * 160)

    fg = wt = None
    controlador = ControladorFPv9()
    stats = Estadisticas()
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
                   offset_v=OFFSET_V_FG, fase_grados=60.0, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True, configurar_salida=True)
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
                
            fp_abs = abs(fp_med) if fp_med is not None else abs(np.cos(np.radians(phi_med)))
            total += 1
            
            en_rango_local = FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO
            if en_rango_local:
                en_rango += 1
                
            res = controlador.actualizar(fp_med, phi_med, f_med, dt, stats)
            
            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])
                
            stats.registrar(fp_abs, res["phi"], res["fase"], res["frecuencia"],
                            res["delta_f"], res["modo"], dt)
            t_rel = t_now - t_ctrl
            
            err_phi_act = abs(envolver_fase(res["phi"] - PHI_TARGET))
            if stats.tiempo_primer_lock is None and err_phi_act < PHI_LOCKED:
                stats.tiempo_primer_lock = t_rel
            if stats.tiempo_primer_tight is None and abs(fp_abs - FP_TARGET) <= FP_TIGHT_TOL:
                stats.tiempo_primer_tight = t_rel

            if res.get("fresh") or total % 5 == 0:
                s = controlador.slope if controlador.slope else SLOPE_INIT
                phi_err = envolver_fase(PHI_TARGET - res["phi"])
                err_val = phi_err / s if abs(s) > SLOPE_MIN_ABS else 0.0
                fila(int(t_rel), fp_abs, res, en_rango_local, controlador.df_integral, err_val)
                
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