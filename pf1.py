"""
CONTROL DE FP v9.3 - PLL con err = -PHI/slope (signo corregido)
================================================================

Fix crítico respecto a v9.2:
  El error del PLL debe ser err = -PHI/slope (con slope < 0).
  El código anterior usaba err = -PHI, lo que invertía la realimentación:
  el PLL empujaba PHI LEJOS de 0 en lugar de hacia 0.

Otras mejoras:
 - slope se inicializa en -0.8 para arrancar sin necesidad de aprendizaje.
 - eff_dt cap a 0.6 s para evitar sobrecorrección en saltos largos.
 - Rechazo de outliers: saltos de PHI >35° en una fresca se descartan.
 - df_integral persiste entre BUSCAR↔PLL.
 - PHI_BUSCAR_ENTER sube a 50° (más margen para el PLL).
"""
import time
import numpy as np
from collections import deque
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================
DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.05

FP_MIN_RANGO = 0.970
FP_MAX_RANGO = 1.030
FP_TIGHT = 0.990

AMPLITUD_FG = 5.0
OFFSET_V_FG = 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

# BUSCAR ↔ PLL
PHI_BUSCAR_ENTER = 50.0       # |PHI| > esto → BUSCAR (3 frescas)
PHI_BUSCAR_EXIT  = 25.0       # |PHI| < esto → PLL (2 frescas)
N_FRESH_ENTER_BUSCAR = 3
N_FRESH_EXIT_BUSCAR  = 2

# BUSCAR
PASO_BUSCAR_INICIAL = 20.0
PASO_BUSCAR_MIN     = 8.0
PASO_BUSCAR_MAX     = 30.0
N_SETTLE_BUSCAR = 6

# PLL
KP_PLL = 0.0025                # adimensional (ver derivación)
KI_PLL = 0.00030               # Hz/(°·s) aprox tras división por slope
DF_MAX = 0.15                  # Hz
DF_LP  = 0.30
EFF_DT_MAX = 0.6               # s — cap del dt efectivo

# Slope: inicialización
SLOPE_INIT = -0.8
SLOPE_MIN_ABS = 0.25           # si |slope| < esto, no se usa para control
SLOPE_SAMPLES_INIT = 20        # peso inicial alto

# Detección de staleness y outliers
STALE_TOL = 0.05
OUTLIER_JUMP = 35.0            # °

PHI_LOCKED = 5.0


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==============================================================================
# RUNTRACKER
# ==============================================================================
class RunTracker:
    def __init__(self, name=""):
        self.name = name
        self.current_samples = 0
        self.current_time = 0.0
        self.max_samples = 0
        self.max_time = 0.0
        self.total_in_run_samples = 0
        self.total_in_run_time = 0.0
        self.total_samples = 0
        self.total_time = 0.0
        self.run_lengths = []
        self.run_times = []

    def update(self, cond, dt):
        self.total_samples += 1
        self.total_time += dt
        if cond:
            self.current_samples += 1
            self.current_time += dt
            self.total_in_run_samples += 1
            self.total_in_run_time += dt
            if self.current_samples > self.max_samples:
                self.max_samples = self.current_samples
            if self.current_time > self.max_time:
                self.max_time = self.current_time
        else:
            self._close_run()

    def _close_run(self):
        if self.current_samples > 0:
            self.run_lengths.append(self.current_samples)
            self.run_times.append(self.current_time)
            self.current_samples = 0
            self.current_time = 0.0

    def close(self):
        self._close_run()

    def summary(self):
        n = len(self.run_lengths)
        pct_time = 100 * self.total_in_run_time / max(self.total_time, 1e-6)
        if n == 0:
            return {"n": 0, "max_time": 0.0, "max_samples": 0,
                    "mean_time": 0.0, "median_time": 0.0,
                    "pct_time": pct_time,
                    "total_time_in": self.total_in_run_time}
        return {"n": n, "max_time": self.max_time, "max_samples": self.max_samples,
                "mean_time": float(np.mean(self.run_times)),
                "median_time": float(np.median(self.run_times)),
                "pct_time": pct_time,
                "total_time_in": self.total_in_run_time}

    def histogram(self, bins):
        hist = np.zeros(len(bins) - 1, dtype=int)
        for t in self.run_times:
            placed = False
            for i in range(len(bins) - 1):
                if bins[i] <= t < bins[i + 1]:
                    hist[i] += 1
                    placed = True
                    break
            if not placed and t >= bins[-1]:
                hist[-1] += 1
        return hist


# ==============================================================================
# ESTADÍSTICAS
# ==============================================================================
class Estadisticas:
    def __init__(self):
        self.fp_hist = []
        self.phi_hist = []
        self.fase_hist = []
        self.frec_hist = []
        self.df_hist = []
        self.tiempos_modo = {"BUSCAR": 0.0, "PLL": 0.0}
        self.bins_fp = np.zeros(10)
        self.n_transiciones = 0
        self.n_stale = 0
        self.n_fresh = 0
        self.n_outliers = 0

        self.run_fp_range = RunTracker("FP en rango [0.97, 1.03]")
        self.run_fp_tight = RunTracker("FP>=0.99")
        self.run_phi_lock = RunTracker("|PHI| < 5°")
        self.run_out = RunTracker("FP fuera rango")

        self.tiempo_primer_lock = None
        self.tiempo_primer_tight = None

    def registrar(self, fp, phi, fase, frec, df, modo, dt):
        self.fp_hist.append(fp)
        self.phi_hist.append(phi)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec)
        self.df_hist.append(df)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        self.bins_fp[min(int(abs(fp) * 10), 9)] += 1

        in_range = FP_MIN_RANGO <= abs(fp) <= FP_MAX_RANGO
        self.run_fp_range.update(in_range, dt)
        self.run_fp_tight.update(abs(fp) >= FP_TIGHT, dt)
        self.run_phi_lock.update(abs(phi) < PHI_LOCKED, dt)
        self.run_out.update(not in_range, dt)

    def cerrar_rachas(self):
        self.run_fp_range.close()
        self.run_fp_tight.close()
        self.run_phi_lock.close()
        self.run_out.close()

    def _imprimir_rachas(self, tracker, bins):
        s = tracker.summary()
        print(f"\n  ▸ {tracker.name}")
        print(f"      Nº rachas:           {s['n']}")
        if s["n"] > 0:
            print(f"      Racha más larga:     {s['max_time']:.2f} s "
                  f"({s['max_samples']} muestras)")
            print(f"      Duración media:      {s['mean_time']:.2f} s")
            print(f"      Duración mediana:    {s['median_time']:.2f} s")
        print(f"      Tiempo total:        {s['total_time_in']:.1f} s "
              f"({s['pct_time']:.1f}%)")
        if s["n"] > 0 and tracker.run_times:
            hist = tracker.histogram(bins)
            print(f"      Distribución:")
            for i in range(len(bins) - 1):
                lo, hi = bins[i], bins[i + 1]
                if hi == np.inf:
                    label = f"      >{lo:>6.1f}s"
                elif lo == 0:
                    label = f"      <{hi:>6.1f}s"
                else:
                    label = f"   {lo:>5.1f}-{hi:<5.1f}s"
                n = hist[i]
                barra = "#" * min(n, 40)
                print(f"        {label} : {n:3d}  {barra}")

    def resumen(self, controlador, t_total, n_iter, en_rango):
        self.cerrar_rachas()
        print("\n" + "=" * 90)
        print(" RESUMEN FINAL v9.3  [MODO EXTREME] — PLL signo corregido")
        print("=" * 90)

        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:            {t_total:.1f} s")
        print(f"  Iteraciones:             {n_iter}")
        print(f"  Tasa efectiva:           {n_iter/max(t_total,1e-6):.1f} muestras/s")
        print(f"  Lecturas frescas:        {self.n_fresh}")
        print(f"  Lecturas stale:          {self.n_stale}")
        print(f"  Outliers rechazados:     {self.n_outliers}")
        if self.fp_hist:
            print(f"  FP promedio:             {np.mean(self.fp_hist):.4f}")
            print(f"  FP desviación:           {np.std(self.fp_hist):.4f}")
        if self.phi_hist:
            print(f"  |PHI| promedio:          {np.mean(np.abs(self.phi_hist)):.2f}°")
            print(f"  |PHI| mediana:           {np.median(np.abs(self.phi_hist)):.2f}°")

        if n_iter > 0:
            ef = 100 * en_rango / n_iter
            print(f"\n[ Efectividad ]")
            print(f"  En rango (1±3%):         {en_rango} ({ef:.1f}%)")

        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            barra = "#" * int(pct / 2)
            print(f"  {m:<10} {t:7.1f}s ({pct:5.1f}%)  {barra}")

        n_tot = self.bins_fp.sum()
        if n_tot > 0:
            print(f"\n[ Distribución de FP ]")
            for i in range(9, -1, -1):
                pct = 100 * self.bins_fp[i] / n_tot
                barra = "#" * int(pct / 2)
                lo = i * 0.1
                print(f"  [{lo:.1f}-{lo+0.1:.1f}] {int(self.bins_fp[i]):5d} "
                      f"({pct:5.1f}%)  {barra}")

        print(f"\n[ Rachas de permanencia ]")
        bins_t = [0, 0.5, 2.0, 5.0, 15.0, 30.0, 60.0, np.inf]
        self._imprimir_rachas(self.run_fp_range, bins_t)
        self._imprimir_rachas(self.run_fp_tight, bins_t)
        self._imprimir_rachas(self.run_phi_lock, bins_t)
        self._imprimir_rachas(self.run_out, bins_t)

        print(f"\n[ Trazabilidad ]")
        print(f"  Mejor FP:                {controlador.mejor_fp:.4f}")
        print(f"  Mejor |PHI|:             {controlador.mejor_phi_abs:.3f}°")
        print(f"  Fase en mejor punto:     {controlador.mejor_fase:+.2f}°")
        if self.tiempo_primer_lock is not None:
            print(f"  1er lock (|PHI|<5°):     {self.tiempo_primer_lock:.2f} s")
        if self.tiempo_primer_tight is not None:
            print(f"  1er FP>=0.99:            {self.tiempo_primer_tight:.2f} s")

        print(f"\n[ PLL ]")
        print(f"  Kp = {KP_PLL:.5f}  Ki = {KI_PLL:.6f}  |Δf|max = {DF_MAX:.3f} Hz")
        if self.df_hist:
            print(f"  Δf promedio:             {np.mean(self.df_hist):+.5f} Hz")
            print(f"  Δf desviación:           {np.std(self.df_hist):.5f} Hz")
            print(f"  Δf máximo abs:           {np.max(np.abs(self.df_hist)):.5f} Hz")
        print(f"  Cambios de fase:         {controlador.n_cambios_fase}")
        print(f"  Transiciones BUSCAR↔PLL: {self.n_transiciones}")
        if controlador.slope is not None:
            print(f"  Slope (dPHI/dφ):         {controlador.slope:+.3f} "
                  f"({controlador.slope_samples} act.)")

        if self.frec_hist:
            uf = self.frec_hist[-100:] if len(self.frec_hist) >= 100 else self.frec_hist
            print(f"\n[ Frecuencia de red medida ]")
            print(f"  Promedio (últimas):      {np.mean(uf):.4f} Hz")
            print(f"  Desviación:              {np.std(uf):.5f} Hz")
            print(f"  Rango global:            {np.min(self.frec_hist):.4f} - "
                  f"{np.max(self.frec_hist):.4f} Hz")

        print(f"\n[ Veredicto ]")
        if n_iter > 0:
            ef = 100 * en_rango / n_iter
            if ef >= 80:
                print(f"  EXCELENTE ({ef:.1f}% en rango)")
            elif ef >= 50:
                print(f"  ACEPTABLE ({ef:.1f}% en rango)")
            elif ef >= 20:
                print(f"  POBRE ({ef:.1f}% en rango)")
            else:
                print(f"  FALLO ({ef:.1f}% en rango)")
        print("=" * 90)


# ==============================================================================
# CONTROLADOR v9.3
# ==============================================================================
class ControladorFPv9:
    def __init__(self):
        # Actuadores
        self.fase_cmd = 0.0
        self.delta_f = 0.0
        self.df_integral = 0.0

        # Estado
        self.modo = "BUSCAR"
        self.cnt = 0
        self.move_pending = False
        self.fresh_after_move = False

        # PHI fresco
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.phi_stale = True
        self.stale_run = 0

        # Slope (se inicializa para arrancar sin aprendizaje)
        self.slope = SLOPE_INIT
        self.slope_samples = SLOPE_SAMPLES_INIT
        self.fase_prev = None
        self.phi_prev = None

        # Búsqueda
        self.paso_buscar = PASO_BUSCAR_INICIAL
        self.dir_buscar = +1.0

        # Contadores
        self.n_cambios_fase = 0
        self.dt_since_fresh = 0.0

        # Histéresis BUSCAR↔PLL
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

        # Mejor histórico
        self.mejor_phi_abs = 180.0
        self.mejor_fase = 0.0
        self.mejor_fp = 0.0

    # ------------------------------------------------------------------
    def _resp(self, accion, f_fg, cambio_fase=False, fresh=False):
        return {
            "accion": accion,
            "fase": self.fase_cmd,
            "delta_f": self.delta_f,
            "frecuencia": f_fg,
            "cambio_fase": cambio_fase,
            "modo": self.modo,
            "phi": self.phi_fresh,
            "fresh": fresh,
            "stale": self.phi_stale,
            "mejor_fp": self.mejor_fp,
            "mejor_fase": self.mejor_fase,
            "mejor_phi_abs": self.mejor_phi_abs,
            "slope": self.slope if self.slope is not None else 0.0,
            "paso": self.paso_buscar,
        }

    def _mover_fase(self, delta):
        if abs(delta) < 0.3:
            return False
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        self.fase_cmd = envolver_fase(self.fase_cmd + delta)
        self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        self.move_pending = True
        self.fresh_after_move = False
        self.cnt = 0
        return True

    def _actualizar_phi(self, phi_raw_deg, stats=None):
        """Devuelve True si la lectura es fresca y válida (no outlier)."""
        phi_w = envolver_fase(phi_raw_deg)
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.stale_run = 0
            return True

        # Repetido → stale
        if abs(phi_w - self.phi_prev_raw) < STALE_TOL:
            self.stale_run += 1
            self.phi_stale = True
            return False

        # Diferente → candidato fresco. Verificar si es outlier
        jump = abs(envolver_fase(phi_w - self.phi_fresh))
        if jump > OUTLIER_JUMP:
            if stats is not None:
                stats.n_outliers += 1
            # No actualizamos phi_fresh (rechazamos el valor)
            # pero tampoco es stale "duro" (el WT sí actualizó)
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            return False

        # Aceptado
        self.phi_fresh = phi_w
        self.phi_prev_raw = phi_w
        self.phi_stale = False
        self.stale_run = 0
        if self.move_pending:
            self.fresh_after_move = True
        return True

    def _actualizar_slope(self):
        if not self.move_pending or not self.fresh_after_move:
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
            self.slope = slope_new
            self.slope_samples = 1
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

    # ------------------------------------------------------------------
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

        fp_abs = abs(fp_med) if fp_med is not None \
            else abs(np.cos(np.radians(self.phi_fresh)))

        if fresh and abs(self.phi_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.phi_fresh)
            self.mejor_fase = self.fase_cmd
            self.mejor_fp = fp_abs

        self._actualizar_slope()

        # --- Histéresis BUSCAR↔PLL (sólo frescas) ---
        if fresh:
            if abs(self.phi_fresh) > PHI_BUSCAR_ENTER:
                self.fresh_enter_buscar += 1
            else:
                self.fresh_enter_buscar = 0

            if abs(self.phi_fresh) < PHI_BUSCAR_EXIT:
                self.fresh_exit_buscar += 1
            else:
                self.fresh_exit_buscar = 0

        if self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            self._cambiar_modo("BUSCAR", stats)
        elif self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            self._cambiar_modo("PLL", stats)

        # --- Ejecución ---
        if self.modo == "BUSCAR":
            return self._buscar(fresh)
        else:
            return self._pll(fresh)

    # ------------------------------------------------------------------
    def _buscar(self, fresh):
        if not fresh:
            return self._resp("BUSCAR-stale", FREC_NOMINAL + self.delta_f)

        if self.cnt < N_SETTLE_BUSCAR:
            return self._resp("BUSCAR-settle", FREC_NOMINAL + self.delta_f)

        # En BUSCAR, Δf=0 (fase del FG libre)
        self.delta_f = 0.0

        if self.slope is not None and abs(self.slope) > SLOPE_MIN_ABS:
            delta = -self.phi_fresh / self.slope
            delta = np.sign(delta) * max(abs(delta), self.paso_buscar)
        else:
            delta = self.dir_buscar * self.paso_buscar
        delta = clamp(delta, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)

        cambio = self._mover_fase(delta)
        return self._resp(f"BUSCAR φ{delta:+.0f}°",
                          FREC_NOMINAL, cambio_fase=cambio, fresh=True)

    # ------------------------------------------------------------------
    def _pll(self, fresh):
        """PLL: Δf = Kp·err + Ki·∫err, con err = -PHI/slope."""
        if fresh:
            eff_dt = min(self.dt_since_fresh, EFF_DT_MAX)
            if eff_dt < 0.05:
                eff_dt = 0.05

            # --- err con signo correcto ---
            s = self.slope if (self.slope is not None
                               and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            err = -self.phi_fresh / s       # clave: dividir por slope

            # Integral
            self.df_integral += KI_PLL * err * eff_dt
            self.df_integral = clamp(self.df_integral, -DF_MAX, DF_MAX)
            # Proporcional
            df_p = clamp(KP_PLL * err, -DF_MAX, DF_MAX)
            # Salida
            df_out = df_p + self.df_integral
            self.delta_f = (1 - DF_LP) * self.delta_f + DF_LP * df_out
            self.delta_f = clamp(self.delta_f, -DF_MAX, DF_MAX)

        f_fg = FREC_NOMINAL + self.delta_f
        return self._resp(f"PLL df={self.delta_f:+.5f}", f_fg, fresh=fresh)


# ==============================================================================
# LOGGING
# ==============================================================================
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


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    print("=" * 160)
    print(" CONTROL FP v9.3 - PLL CON SIGNO CORREGIDO (err = -PHI/slope)  [MODO EXTREME]")
    print("=" * 160)
    print(f" Muestreo objetivo: {1/INTERVALO_MUESTREO:.0f} Hz")
    print(f" Kp = {KP_PLL:.5f}  Ki = {KI_PLL:.6f}  |Δf|max = {DF_MAX:.3f} Hz")
    print(f" BUSCAR entra si |PHI| > {PHI_BUSCAR_ENTER}° (3 frescas); "
          f"sale si |PHI| < {PHI_BUSCAR_EXIT}° (2 frescas)")
    print(f" Slope inicializado en {SLOPE_INIT}. Outliers: saltos > {OUTLIER_JUMP}°")
    print("=" * 160)

    fg = None
    wt = None
    controlador = ControladorFPv9()
    stats = Estadisticas()

    total = 0
    en_rango = 0
    t_ctrl = None

    try:
        print("\n[1/3] Conectando equipos (modo EXTREME)...")
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()
        print(f"  FG420:  {fg.obtener_idn()[:60]}")
        print(f"  WT3000: {wt.obtener_idn()[:60]}")

        fg.extreme(
            canal=1, frecuencia_hz=FREC_NOMINAL,
            amplitud_vpp=AMPLITUD_FG, offset_v=OFFSET_V_FG,
            fase_grados=0.0, encender_salida=True,
        )
        wt.extreme(
            elemento_entrada=ELEMENTO_WT,
            incluir_potencias=True, configurar_salida=True,
        )
        print("  Modo EXTREME aplicado.")

        print("\n[2/3] Estabilizando 5 s...")
        for _ in range(10):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print(" OK")

        print("\n[3/3] INICIANDO CONTROL\n")
        encabezado()

        t_ctrl = time.time()
        t_prev = t_ctrl

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

            res = controlador.actualizar(fp_med, phi_med, f_med, dt, stats)

            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["phi"], res["fase"],
                            res["frecuencia"], res["delta_f"],
                            res["modo"], dt)

            t_rel = t_now - t_ctrl
            if stats.tiempo_primer_lock is None and abs(res["phi"]) < PHI_LOCKED:
                stats.tiempo_primer_lock = t_rel
            if stats.tiempo_primer_tight is None and fp_abs >= FP_TIGHT:
                stats.tiempo_primer_tight = t_rel

            # Log: frescas o cada 8 muestras
            if res.get("fresh") or total % 8 == 0:
                s = controlador.slope if controlador.slope else SLOPE_INIT
                err_val = -res["phi"] / s if abs(s) > SLOPE_MIN_ABS else 0.0
                fila(int(t_rel), fp_abs, res, en_rango_local,
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