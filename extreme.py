"""
CONTROL DE FP v9.7 - Hill-climb decisivo sobre FASE en FIJAR
=============================================================================
Fixes v9.7 (basados en v9.6 que mostró 34.0% con mejor FP +0.9995 en +58°):

 BUG J: FIJAR casi no actuaba (78 cambios de fase en 135 s ≈ 1 cada 1.7 s).
        FIX:  dead-zone 0.015 → 0.003, cooldown 3 → 1, eval 4 → 3,
              paso inicial 2° → 3°, paso máx 12° → 15°.

 BUG K: PI de frecuencia en FIJAR saturaba a ±0.010 Hz sin mover el FP.
        FIX:  KP_FIJAR = KI_FIJAR = DF_FIJAR_MAX = 0. FIJAR solo mueve fase.

 BUG L: f_filt se actualizaba en FIJAR, metiendo ruido del WT en el FG420.
        FIX:  FIJAR_FREEZE_FREQ = True. Frecuencia congelada en FIJAR.

 BUG M: no había mecanismo de return-to-best dentro de FIJAR.
        FIX:  running-best + salto de una sola escritura si dist > 3×dist_min.

 BUG N: se escribía la frecuencia en cada iteración aunque no cambiara.
        FIX:  umbral FREQ_WRITE_EPS = 0.2 mHz. Menos writes → más tasa.

La máquina de estados BUSCAR/AJUSTAR/FIJAR y el hill-climb sobre fase
son los mismos conceptos que v9.6, pero con parámetros sintonizados para
que FIJAR reaccione en ~150-200 ms en lugar de ~350 ms.
"""
import time
import numpy as np
from collections import deque
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

# --- Selector de modo de I/O ---
MODO_EXTREME = True
INTERVALO_EXTREME = 0.0
INTERVALO_MUESTREO = 0.05

DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 600

# --- Objetivos ---
FP_OBJETIVO_MAG = 1.0
FP_OBJETIVO     = 1.0
TIPO_CARGA      = "RESISTIVO"
TOLERANCIA_FP   = 0.03
FP_MIN_RANGO = FP_OBJETIVO - TOLERANCIA_FP
FP_MAX_RANGO = FP_OBJETIVO + TOLERANCIA_FP

AMPLITUD_FG = 5.0
OFFSET_V_FG = 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0
FREC_MIN, FREC_MAX = 55.0, 65.0

ALPHA_F = 0.08
BETA_F = 0.002
FP_BUF_LEN = 3

# --- Transiciones de modo ---
DELTA_BUSCAR_AJUSTAR = 0.100
DELTA_AJUSTAR_FIJAR  = 0.045
DELTA_SALIR_FIJAR    = 0.15       # antes 0.100
DELTA_REBUSCAR       = 0.300

N_CONSEC_BUSCAR_OK = 2
N_CONSEC_FIJAR_OK  = 3
N_CONSEC_FIJAR_FAIL = 30          # antes 40

N_WARMUP_AJUSTAR = 8
N_WARMUP_FIJAR   = 6              # antes 4

# --- BUSCAR ---
PASO_FASE_INICIAL = 25.0
PASO_FASE_MIN = 2.0
N_ESPERA_BUSCAR = 5
N_MOVES_SIN_MEJORA = 20
N_MOVES_TOTAL_MAX  = 60

# --- AJUSTAR ---
PASO_DF_INICIAL = 0.010
PASO_DF_MIN = 0.0012
N_APLICAR_DF = 3
N_ESPERA_DF = 5

# --- FIJAR v9.7: hill-climb decisivo sobre FASE ---
FIJAR_ACTIVO_UMBRAL  = 0.003      # antes 0.015
FIJAR_COOLDOWN       = 1          # antes 3
FIJAR_EVAL_ITER      = 3          # antes 4
FIJAR_PASO_INICIAL   = 3.0        # antes 2.0
FIJAR_PASO_MIN       = 0.3        # antes 0.5
FIJAR_PASO_MAX       = 15.0       # antes 12.0
FIJAR_UMBRAL_MEJORA  = 0.002      # antes 0.004
FIJAR_RETURN_BEST    = 3.0        # nuevo
FIJAR_FREEZE_FREQ    = True       # nuevo

# PI de frecuencia DESACTIVADO en FIJAR
KP_FIJAR      = 0.0
KI_FIJAR      = 0.0
DF_FIJAR_MAX  = 0.0

# --- Anti-oscilación ---
N_EMPEORAS_SEGUIDAS = 2
UMBRAL_MEJORA = 0.003
UMBRAL_EMPEORA = 0.003
UMBRAL_EFECTO = 0.004
UMBRAL_STREAK_1 = 10.0
UMBRAL_STREAK_2 = 30.0
UMBRAL_STREAK_3 = 60.0

# --- Umbral de escritura de frecuencia ---
FREQ_WRITE_EPS = 2e-4   # 0.2 mHz


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==============================================================================
# PREGUNTA DE OBJETIVOS
# ==============================================================================

def preguntar_objetivos():
    global FP_OBJETIVO_MAG, FP_OBJETIVO, TIPO_CARGA, FP_MIN_RANGO, FP_MAX_RANGO

    print("\n" + "=" * 90)
    print(" CONFIGURACIÓN DE OBJETIVOS DE PRUEBA")
    print("=" * 90)
    print("\n Convención de signos:")
    print("   • RESISTIVO  → FP ≈ +1.00")
    print("   • INDUCTIVO  → FP positivo (0 < FP < 1)")
    print("   • CAPACITIVO → FP negativo (-1 < FP < 0)")
    print("-" * 90)

    while True:
        try:
            s = input("\n  ► Factor de potencia objetivo (magnitud 0.0 - 1.0): ").strip()
            fp_mag = float(s)
            if 0.0 <= fp_mag <= 1.0:
                break
            print("    [!] Debe estar entre 0.0 y 1.0")
        except (ValueError, EOFError):
            print("    [!] Entrada inválida")

    print("\n  ► Tipo de carga:")
    print("      0 = RESISTIVO  (FP ~unitario)")
    print("      1 = INDUCTIVO  (FP positivo < 1)")
    print("      2 = CAPACITIVO (FP negativo)")
    while True:
        try:
            s = input("    Seleccione (0/1/2) [por defecto 0]: ").strip() or "0"
            tipo = int(s)
            if tipo in (0, 1, 2):
                break
            print("    [!] Debe ser 0, 1 o 2")
        except (ValueError, EOFError):
            print("    [!] Entrada inválida")

    if tipo == 0:
        tipo_str = "RESISTIVO"; fp_signed = +fp_mag
    elif tipo == 1:
        tipo_str = "INDUCTIVO"; fp_signed = +fp_mag
    else:
        tipo_str = "CAPACITIVO"; fp_signed = -fp_mag

    FP_OBJETIVO_MAG = fp_mag
    FP_OBJETIVO = fp_signed
    TIPO_CARGA = tipo_str
    FP_MIN_RANGO = fp_signed - TOLERANCIA_FP
    FP_MAX_RANGO = fp_signed + TOLERANCIA_FP

    print("-" * 90)
    print(f"  ✓ FP objetivo (magnitud) : {fp_mag:.4f}")
    print(f"  ✓ Tipo de carga          : {tipo_str}")
    print(f"  ✓ FP objetivo firmado    : {fp_signed:+.4f}")
    print(f"  ✓ Banda de aceptación    : [{FP_MIN_RANGO:+.4f}, {FP_MAX_RANGO:+.4f}]")
    print("=" * 90)
    input("\n  Presione ENTER para iniciar la prueba...")


# ==============================================================================
# FILTRO α-β
# ==============================================================================

class FrecuenciaTracker:
    def __init__(self, f0=60.0, alpha=0.08, beta=0.002):
        self.f = f0; self.df = 0.0
        self.alpha = alpha; self.beta = beta

    def update(self, f_meas, dt):
        if dt <= 1e-6:
            return self.f, self.df
        dt = clamp(dt, 0.005, 0.3)   # v9.7: mínimo 5 ms (antes 10 ms)
        f_pred = self.f + self.df * dt
        e = f_meas - f_pred
        self.f = f_pred + self.alpha * e
        self.df = self.df + self.beta * e / dt
        return self.f, self.df


# ==============================================================================
# ESTADÍSTICAS
# ==============================================================================

class Estadisticas:
    def __init__(self):
        self.fp_hist = []
        self.fase_hist = []
        self.frec_hist = []
        self.frec_filt_hist = []
        self.df_hist = []
        self.tiempos_modo = {"BUSCAR": 0.0, "AJUSTAR": 0.0, "FIJAR": 0.0}
        self.bins_fp = np.zeros(10)

        self.n_transiciones = 0
        self.n_rebusquedas = 0
        self.efecto_pruebas = 0
        self.efecto_exitosas = 0

        self.t_in_range = 0.0
        self.t_out_range = 0.0

        self.streak_actual = 0.0
        self.streak_max = 0.0
        self.streak_list = []
        self.streak_inicio_t = None
        self.n_streaks_10 = 0
        self.n_streaks_30 = 0
        self.n_streaks_60 = 0
        self.hitos_log = []

        self.integral_fijar_hist = []
        self.n_micro_fase = 0
        self.n_anti_oscilacion = 0
        self.n_return_to_best = 0

    def registrar(self, fp, fase, frec_raw, frec_filt, df, modo, dt, integral=0.0):
        self.fp_hist.append(fp)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec_raw)
        self.frec_filt_hist.append(frec_filt)
        self.df_hist.append(df)
        self.integral_fijar_hist.append(integral)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        idx = min(int(abs(fp) * 10), 9)
        self.bins_fp[idx] += 1

    def update_streak(self, en_rango, dt, t_actual):
        evento = None
        if en_rango:
            self.t_in_range += dt
            if self.streak_actual == 0.0:
                self.streak_inicio_t = t_actual
            prev = self.streak_actual
            self.streak_actual += dt
            self.streak_max = max(self.streak_max, self.streak_actual)
            for umbral, nombre, attr in [
                (UMBRAL_STREAK_1, "10s", "n_streaks_10"),
                (UMBRAL_STREAK_2, "30s", "n_streaks_30"),
                (UMBRAL_STREAK_3, "60s", "n_streaks_60"),
            ]:
                if prev < umbral <= self.streak_actual:
                    setattr(self, attr, getattr(self, attr) + 1)
                    self.hitos_log.append((t_actual, nombre))
                    evento = nombre
        else:
            self.t_out_range += dt
            if self.streak_actual > 0.5:
                self.streak_list.append(self.streak_actual)
            self.streak_actual = 0.0
            self.streak_inicio_t = None
        return evento

    def resumen(self, controlador, t_total, n_iter, en_rango, fg=None, wt=None):
        titulo = "RESUMEN FINAL v9.7 (FASE decisiva + sin PI freq)" \
                 if MODO_EXTREME else "RESUMEN FINAL v9.7 (streaming)"
        print("\n" + "=" * 90)
        print(f" {titulo}")
        print("=" * 90)

        print(f"\n[ Modo de I/O ]")
        print(f"  Selector:                {'EXTREME' if MODO_EXTREME else 'streaming'}")
        print(f"  Intervalo objetivo:      "
              f"{INTERVALO_EXTREME if MODO_EXTREME else INTERVALO_MUESTREO:.4f} s")

        print(f"\n[ Objetivo ]")
        print(f"  Tipo de carga:           {TIPO_CARGA}")
        print(f"  FP objetivo (firmado):   {FP_OBJETIVO:+.4f}")
        print(f"  Banda aceptación:        [{FP_MIN_RANGO:+.4f}, {FP_MAX_RANGO:+.4f}]")

        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:            {t_total:.1f} s")
        print(f"  Iteraciones:             {n_iter}")
        print(f"  Tasa efectiva:           {n_iter/max(t_total,1e-6):.1f} muestras/s")

        if n_iter > 0:
            efectividad = 100 * en_rango / n_iter
            print(f"\n[ Efectividad ]")
            print(f"  En rango ({FP_OBJETIVO:+.3f}±{TOLERANCIA_FP:.2f}):  "
                  f"{en_rango} ({efectividad:.1f}%)")

        t_total_medido = self.t_in_range + self.t_out_range
        if t_total_medido > 0:
            pct_in = 100 * self.t_in_range / t_total_medido
            print(f"\n[ Tiempo en rango (segundos) ]")
            print(f"  En rango:                {self.t_in_range:8.1f} s  ({pct_in:5.1f}%)")
            print(f"  Fuera de rango:          {self.t_out_range:8.1f} s  "
                  f"({100-pct_in:5.1f}%)")
            barra_in = int(pct_in / 2)
            print(f"  [{('#'*barra_in):<50}] {pct_in:.1f}%")

        print(f"\n[ Rachas en rango ]")
        if self.streak_list:
            arr = np.array(self.streak_list)
            print(f"  Número de rachas:        {len(arr)}")
            print(f"  Duración media:          {np.mean(arr):7.2f} s")
            print(f"  Duración mediana:        {np.median(arr):7.2f} s")
            print(f"  Duración mínima:         {np.min(arr):7.2f} s")
            print(f"  Duración máxima:         {np.max(arr):7.2f} s")
            print(f"  Desviación estándar:     {np.std(arr):7.2f} s")

            dur = np.array(self.streak_list)
            b_0_5 = int(np.sum((dur >= 0)  & (dur < 5)))
            b_5_10 = int(np.sum((dur >= 5) & (dur < 10)))
            b_10_30 = int(np.sum((dur >= 10) & (dur < 30)))
            b_30_60 = int(np.sum((dur >= 30) & (dur < 60)))
            b_60 = int(np.sum(dur >= 60))
            total_b = max(len(dur), 1)
            print(f"\n  Distribución de duraciones:")
            for lbl, cnt in [("0-5s", b_0_5), ("5-10s", b_5_10),
                             ("10-30s", b_10_30), ("30-60s", b_30_60),
                             (">=60s", b_60)]:
                pct = 100 * cnt / total_b
                print(f"    {lbl:<7} {cnt:5d} ({pct:5.1f}%)  "
                      f"{'#' * int(pct/2)}")
        if self.streak_max > 0:
            print(f"  Racha máxima:            {self.streak_max:.1f} s")
        print(f"  Rachas ≥ 10 s:           {self.n_streaks_10}")
        print(f"  Rachas ≥ 30 s:           {self.n_streaks_30}")
        print(f"  Rachas ≥ 60 s:           {self.n_streaks_60}")

        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            barra = "#" * int(pct / 2)
            print(f"  {m:<10} {t:7.1f}s ({pct:5.1f}%)  {barra}")

        n_tot = self.bins_fp.sum()
        if n_tot > 0:
            print(f"\n[ Distribución de |FP| ]")
            for i in range(9, -1, -1):
                pct = 100 * self.bins_fp[i] / n_tot
                barra = "#" * int(pct / 2)
                lo = i * 0.1
                print(f"  [{lo:.1f}-{lo+0.1:.1f}] {int(self.bins_fp[i]):5d} "
                      f"({pct:5.1f}%)  {barra}")

        print(f"\n[ Mejor punto ]")
        print(f"  Mejor FP (más cercano):  {controlador.mejor_fp:+.4f}  "
              f"(objetivo {FP_OBJETIVO:+.4f})")
        print(f"  Fase en mejor FP:        {controlador.mejor_fase:+.2f}°")

        if self.frec_hist:
            uf = self.frec_hist[-400:] if len(self.frec_hist) >= 400 else self.frec_hist
            uff = self.frec_filt_hist[-400:] if len(self.frec_filt_hist) >= 400 else self.frec_filt_hist
            print(f"\n[ Frecuencia de red ]")
            print(f"  Raw prom:                {np.mean(uf):.5f} Hz")
            print(f"  Raw std:                 {np.std(uf):.6f} Hz")
            print(f"  Filt prom:               {np.mean(uff):.5f} Hz")
            print(f"  Filt std:                {np.std(uff):.6f} Hz")

        if self.df_hist:
            dfs = np.array(self.df_hist)
            print(f"\n[ Δf ]")
            print(f"  Media:                   {np.mean(dfs):+.6f} Hz")
            print(f"  Máx abs:                 {np.max(np.abs(dfs)):.6f} Hz")
            print(f"  Fracción no-cero:        "
                  f"{100*np.mean(np.abs(dfs)>1e-5):.1f}%")

        if self.integral_fijar_hist:
            ii = np.array(self.integral_fijar_hist)
            ii_nz = ii[np.abs(ii) > 1e-9]
            if len(ii_nz) > 0:
                print(f"\n[ Integrador en FIJAR ]")
                print(f"  Valor final:             {ii[-1]:+.6f} Hz")
                print(f"  Valor medio (no-cero):   {np.mean(ii_nz):+.6f} Hz")
                print(f"  Rango:                   {np.min(ii_nz):+.6f} a "
                      f"{np.max(ii_nz):+.6f} Hz")
            else:
                print(f"\n[ Integrador en FIJAR ]")
                print(f"  Desactivado (v9.7)")

        print(f"\n[ Comportamiento ]")
        print(f"  Transiciones de modo:    {self.n_transiciones}")
        print(f"  Re-búsquedas:            {self.n_rebusquedas}")
        print(f"  Cambios de fase:         {controlador.n_cambios_fase}")
        print(f"  Micro-ajustes de fase:   {self.n_micro_fase}")
        print(f"  Anti-oscilación:         {self.n_anti_oscilacion}")
        print(f"  Return-to-best:          {self.n_return_to_best}")
        print(f"  Return-best en FIJAR:    {getattr(controlador,'n_return_best_fijar',0)}")
        print(f"  HOLD en FIJAR:           {getattr(controlador,'n_hold_fijar',0)}")

        if fg is not None:
            try:
                s = fg.stats
                if s.get("n", 0) > 0:
                    print(f"\n[ Latencia I/O — FG420 (write) ]")
                    print(f"  n={s['n']}  "
                          f"mean={s['mean_ms']:.2f}ms  "
                          f"p50={s['p50_ms']:.2f}ms  "
                          f"p95={s['p95_ms']:.2f}ms  "
                          f"p99={s['p99_ms']:.2f}ms  "
                          f"max={s['max_ms']:.2f}ms")
            except Exception:
                pass
        if wt is not None:
            try:
                s = wt.stats
                if s.get("n", 0) > 0:
                    print(f"\n[ Latencia I/O — WT3000 (query) ]")
                    print(f"  n={s['n']}  "
                          f"mean={s['mean_ms']:.2f}ms  "
                          f"p50={s['p50_ms']:.2f}ms  "
                          f"p95={s['p95_ms']:.2f}ms  "
                          f"p99={s['p99_ms']:.2f}ms  "
                          f"max={s['max_ms']:.2f}ms")
            except Exception:
                pass

        print(f"\n[ Veredicto ]")
        if n_iter > 0:
            ef = 100 * en_rango / n_iter
            if ef >= 80:
                print(f"  EXCELENTE ({ef:.1f}% en rango, racha máx {self.streak_max:.1f}s)")
            elif ef >= 70:
                print(f"  BUENO ({ef:.1f}% en rango, racha máx {self.streak_max:.1f}s)")
            elif ef >= 50:
                print(f"  ACEPTABLE ({ef:.1f}% en rango)")
            else:
                print(f"  POBRE ({ef:.1f}% en rango)")
        print("=" * 90)


# ==============================================================================
# CONTROLADOR v9.7
# ==============================================================================

class ControladorFPv9:
    def __init__(self):
        self.filtro_f = FrecuenciaTracker(FREC_NOMINAL, ALPHA_F, BETA_F)

        self.fase_cmd = 0.0
        self.delta_f = 0.0

        self.modo = "BUSCAR"
        self.sub_estado = "MOVER"
        self.cnt = 0
        self.fp_ref = None

        self.signo = +1.0
        self.paso_fase = PASO_FASE_INICIAL
        self.paso_df = PASO_DF_INICIAL

        self.fp_buf = deque(maxlen=FP_BUF_LEN)
        self.fp_suav = None

        self.mejor_fp = None
        self.mejor_fase = 0.0

        self.cnt_consec_buscar_ok = 0
        self.cnt_consec_fijar_ok = 0
        self.cnt_consec_fijar_fail = 0

        self.integral_fijar = 0.0

        self.n_cambios_fase = 0
        self.n_cont_malo = 0

        self.empeoras_seguidas = 0

        self.n_moves_total = 0
        self.n_moves_sin_mejorar = 0

        self.cnt_ajustar = 0
        self.cnt_fijar_warmup = 0
        self.samples_since_fase = 0
        self.warned_unreachable = False

        self.cnt_fuera_fijar = 0
        self.cnt_desde_micro = 0

        self.f_filt = FREC_NOMINAL

        # v9.7: hill-climb de fase en FIJAR
        self.fase_paso_fijar     = FIJAR_PASO_INICIAL
        self.signo_fase_fijar    = +1.0
        self.fp_antes_paso       = None
        self.dist_min_fijar      = 1e9
        self.fase_mejor_fijar    = 0.0
        self.fase_fp_hist_fijar  = deque(maxlen=10)
        self.n_hold_fijar        = 0
        self.n_return_best_fijar = 0

    # ------------------------------------------------------------------
    def _dist(self):
        if self.fp_suav is None:
            return 1e9
        return abs(self.fp_suav - FP_OBJETIVO)

    def _paso_fase_adaptativo(self):
        d = self._dist()
        if d > 0.70:  return 45.0
        if d > 0.40:  return 30.0
        if d > 0.20:  return 20.0
        if d > 0.10:  return 12.0
        if d > 0.05:  return 6.0
        if d > 0.025: return 3.0
        return PASO_FASE_MIN

    def _resp(self, accion, cambio_fase=False):
        return {
            "accion": accion,
            "fase": self.fase_cmd,
            "delta_f": self.delta_f,
            "frecuencia": self.f_filt + self.delta_f,
            "frecuencia_filt": self.f_filt,
            "cambio_fase": cambio_fase,
            "modo": self.modo,
            "fp": self.fp_suav if self.fp_suav is not None else 0.0,
            "mejor_fp": self.mejor_fp if self.mejor_fp is not None else 0.0,
            "mejor_fase": self.mejor_fase,
            "paso_fase": self.paso_fase,
            "paso_df": self.paso_df,
            "signo": self.signo,
            "integral_fijar": self.integral_fijar,
        }

    def _transicionar(self, nuevo_modo, stats):
        if nuevo_modo != self.modo:
            stats.n_transiciones += 1
            self.modo = nuevo_modo
            self.sub_estado = "MOVER"
            self.cnt = 0
            self.fp_ref = self.fp_suav
            self.cnt_consec_buscar_ok = 0
            self.cnt_consec_fijar_ok = 0
            self.cnt_consec_fijar_fail = 0
            self.empeoras_seguidas = 0
            self.cnt_ajustar = 0
            self.cnt_fijar_warmup = 0
            if nuevo_modo == "BUSCAR":
                self.integral_fijar = 0.0
                self.n_moves_total = 0
                self.n_moves_sin_mejorar = 0
            if nuevo_modo == "FIJAR":
                self.cnt_fuera_fijar = 0
                self.cnt_desde_micro = 0
                self.integral_fijar = 0.0
                # v9.7: reset del hill-climb
                self.fase_paso_fijar     = FIJAR_PASO_INICIAL
                self.signo_fase_fijar    = +1.0
                self.fp_antes_paso       = (self.fp_suav
                                            if self.fp_suav is not None
                                            else FP_OBJETIVO)
                self.dist_min_fijar      = self._dist()
                self.fase_mejor_fijar    = self.fase_cmd
                self.fase_fp_hist_fijar.clear()
                self.n_hold_fijar        = 0
                self.n_return_best_fijar = 0

    # ------------------------------------------------------------------
    def actualizar(self, fp_med, f_med, dt, stats):
        # v9.7: en FIJAR congelamos la frecuencia base
        if not (FIJAR_FREEZE_FREQ and self.modo == "FIJAR"):
            if f_med is not None and FREC_MIN < f_med < FREC_MAX:
                self.f_filt, _ = self.filtro_f.update(f_med, dt)

        if fp_med is None:
            return self._resp("SIN_FP")

        fp = float(fp_med)
        self.fp_buf.append(fp)
        self.fp_suav = float(np.mean(self.fp_buf))

        self.samples_since_fase += 1
        if self.samples_since_fase >= FP_BUF_LEN:
            dist_actual = abs(self.fp_suav - FP_OBJETIVO)
            if self.mejor_fp is None or dist_actual < abs(self.mejor_fp - FP_OBJETIVO):
                self.mejor_fp = self.fp_suav
                self.mejor_fase = self.fase_cmd
                self.n_moves_sin_mejorar = 0

        dist_actual = self._dist()
        if dist_actual > DELTA_REBUSCAR:
            self.n_cont_malo += 1
        else:
            self.n_cont_malo = 0

        if self.n_cont_malo >= 15 and self.modo != "BUSCAR":
            stats.n_rebusquedas += 1
            self._transicionar("BUSCAR", stats)
            self.signo = +1.0

        if (not self.warned_unreachable and self.mejor_fp is not None and
                self.n_moves_total >= N_MOVES_TOTAL_MAX * 2):
            dist_mejor = abs(self.mejor_fp - FP_OBJETIVO)
            if dist_mejor > DELTA_BUSCAR_AJUSTAR:
                print(f"\n[!] AVISO: tras {self.n_moves_total} movimientos el mejor FP "
                      f"está a {dist_mejor:.4f} del objetivo.\n")
                self.warned_unreachable = True

        if self.modo == "BUSCAR":
            return self._buscar(stats)
        elif self.modo == "AJUSTAR":
            return self._ajustar(stats)
        else:
            return self._fijar(stats, dt)

    # ------------------------------------------------------------------
    def _buscar(self, stats):
        if self._dist() <= DELTA_BUSCAR_AJUSTAR:
            self.cnt_consec_buscar_ok += 1
            if self.cnt_consec_buscar_ok >= N_CONSEC_BUSCAR_OK:
                self._transicionar("AJUSTAR", stats)
                self.signo = +1.0
                self.paso_df = PASO_DF_INICIAL
                self.delta_f = 0.0
                return self._resp("BUSCAR→AJUSTAR")
        else:
            self.cnt_consec_buscar_ok = 0

        if self.sub_estado == "MOVER":
            paso_max = self._paso_fase_adaptativo()
            if self.paso_fase < paso_max * 0.4:
                self.paso_fase = min(self.paso_fase * 1.5, paso_max)
                self.paso_fase = max(self.paso_fase, PASO_FASE_MIN)
            paso = min(self.paso_fase, paso_max)
            self.paso_fase = paso

            self.fase_cmd = envolver_fase(self.fase_cmd + self.signo * self.paso_fase)
            self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
            self.sub_estado = "MEDIR"
            self.cnt = 0
            self.fp_ref = self.fp_suav
            self.n_cambios_fase += 1
            self.n_moves_total += 1
            self.n_moves_sin_mejorar += 1
            self.samples_since_fase = 0
            self.delta_f = 0.0
            return self._resp(f"FASE{self.signo*self.paso_fase:+.0f}",
                              cambio_fase=True)

        self.cnt += 1
        if self.cnt >= N_ESPERA_BUSCAR:
            fp_ref_val = self.fp_ref if self.fp_ref is not None else FP_OBJETIVO
            mejora = abs(fp_ref_val - FP_OBJETIVO) - self._dist()

            stats.efecto_pruebas += 1
            if abs(mejora) > UMBRAL_EFECTO:
                stats.efecto_exitosas += 1

            if mejora > UMBRAL_MEJORA:
                self.empeoras_seguidas = 0
            elif mejora < -UMBRAL_EMPEORA:
                self.empeoras_seguidas += 1
                if self.empeoras_seguidas >= N_EMPEORAS_SEGUIDAS:
                    self.signo *= -1
                    self.paso_fase = max(self.paso_fase * 0.6, PASO_FASE_MIN)
                    self.empeoras_seguidas = 0
                    stats.n_anti_oscilacion += 1
                else:
                    self.signo *= -1
                    self.paso_fase = max(self.paso_fase * 0.8, PASO_FASE_MIN)
            else:
                self.paso_fase = max(self.paso_fase * 0.9, PASO_FASE_MIN)

            if self.n_moves_sin_mejorar >= N_MOVES_SIN_MEJORA:
                dist_mejor = abs(self.mejor_fp - FP_OBJETIVO)
                if dist_mejor <= DELTA_BUSCAR_AJUSTAR:
                    self.fase_cmd = self.mejor_fase
                    self.n_cambios_fase += 1
                    self.samples_since_fase = 0
                    stats.n_return_to_best += 1
                    self._transicionar("AJUSTAR", stats)
                    self.signo = +1.0
                    self.paso_df = PASO_DF_INICIAL
                    self.delta_f = 0.0
                    return self._resp("RETURN_TO_BEST→AJUSTAR",
                                      cambio_fase=True)
                else:
                    self.paso_fase = max(self.paso_fase * 0.5, PASO_FASE_MIN)
                    self.n_moves_sin_mejorar = 0
                    self.empeoras_seguidas = 0

            if self.n_moves_total >= N_MOVES_TOTAL_MAX:
                dist_mejor = abs(self.mejor_fp - FP_OBJETIVO)
                if dist_mejor <= DELTA_BUSCAR_AJUSTAR * 1.5:
                    self.fase_cmd = self.mejor_fase
                    self.n_cambios_fase += 1
                    self.samples_since_fase = 0
                    stats.n_return_to_best += 1
                    self._transicionar("AJUSTAR", stats)
                    self.signo = +1.0
                    self.paso_df = PASO_DF_INICIAL
                    self.delta_f = 0.0
                    return self._resp("FORCE_BEST→AJUSTAR",
                                      cambio_fase=True)
                else:
                    self.n_moves_total = 0
                    self.n_moves_sin_mejorar = 0

            self.sub_estado = "MOVER"
            self.cnt = 0
        return self._resp("MEDIR_BUSCAR")

    # ------------------------------------------------------------------
    def _ajustar(self, stats):
        self.cnt_ajustar += 1

        if self.cnt_ajustar < N_WARMUP_AJUSTAR:
            if self.sub_estado == "MOVER":
                self.sub_estado = "MEDIR"
                self.fp_ref = self.fp_suav
                self.cnt = 0
            return self._resp(f"WARMUP_AJUSTAR({self.cnt_ajustar})")

        if self._dist() <= DELTA_AJUSTAR_FIJAR:
            self.cnt_consec_fijar_ok += 1
            if self.cnt_consec_fijar_ok >= N_CONSEC_FIJAR_OK:
                self._transicionar("FIJAR", stats)
                self.integral_fijar = 0.0
                self.delta_f = 0.0
                return self._resp("AJUSTAR→FIJAR")
        else:
            self.cnt_consec_fijar_ok = 0

        if self._dist() > DELTA_REBUSCAR:
            self._transicionar("BUSCAR", stats)
            self.delta_f = 0.0
            return self._resp("AJUSTAR→BUSCAR")

        if self.sub_estado == "MOVER":
            self.delta_f = self.signo * self.paso_df
            self.cnt += 1
            if self.cnt >= N_APLICAR_DF:
                self.sub_estado = "MEDIR"
                self.delta_f = 0.0
                self.cnt = 0
                self.fp_ref = self.fp_suav
            return self._resp(f"DF{self.delta_f:+.4f}")

        self.cnt += 1
        if self.cnt >= N_ESPERA_DF:
            fp_ref_val = self.fp_ref if self.fp_ref is not None else FP_OBJETIVO
            mejora = abs(fp_ref_val - FP_OBJETIVO) - self._dist()

            stats.efecto_pruebas += 1
            if abs(mejora) > UMBRAL_EFECTO:
                stats.efecto_exitosas += 1

            if mejora > UMBRAL_MEJORA:
                pass
            elif mejora < -UMBRAL_EMPEORA:
                self.signo *= -1
                self.paso_df = max(self.paso_df * 0.7, PASO_DF_MIN)
            else:
                self.paso_df = max(self.paso_df * 0.85, PASO_DF_MIN)

            self.sub_estado = "MOVER"
            self.cnt = 0
        return self._resp("MEDIR_AJUSTAR")

    # ------------------------------------------------------------------
    def _fijar(self, stats, dt):
        """
        FIJAR v9.7: hill-climb decisivo sobre FASE.
        - Sin PI de frecuencia (no tiene autoridad).
        - Dead-zone muy estrecha (0.003).
        - Cooldown mínimo (1 iteración).
        - Running-best + return-to-best si nos perdemos.
        """
        # Warmup
        if self.cnt_fijar_warmup < N_WARMUP_FIJAR:
            self.cnt_fijar_warmup += 1
            self.delta_f = 0.0
            return self._resp(f"WARMUP_FIJAR({self.cnt_fijar_warmup})")

        self.delta_f = 0.0  # sin PI de frecuencia
        dist = self._dist()

        # Actualizar mejor observado
        if dist < self.dist_min_fijar:
            self.dist_min_fijar   = dist
            self.fase_mejor_fijar = self.fase_cmd

        # Salida de emergencia
        if dist > DELTA_SALIR_FIJAR:
            self.cnt_consec_fijar_fail += 1
            if self.cnt_consec_fijar_fail >= N_CONSEC_FIJAR_FAIL:
                self._transicionar("AJUSTAR", stats)
                self.sub_estado = "MOVER"
                self.cnt = 0
                self.fp_ref = self.fp_suav
                return self._resp("FIJAR→AJUSTAR")
        else:
            self.cnt_consec_fijar_fail = 0

        # === EVAL del último paso ===
        if self.sub_estado == "EVAL_FASE":
            self.cnt += 1
            if self.cnt < FIJAR_EVAL_ITER:
                return self._resp(f"EVAL_FIJAR({self.cnt})")

            dist_antes = abs(self.fp_antes_paso - FP_OBJETIVO)
            mejora = dist_antes - dist

            stats.efecto_pruebas += 1
            if abs(mejora) > UMBRAL_EFECTO:
                stats.efecto_exitosas += 1

            self.fase_fp_hist_fijar.append((self.fase_cmd, self.fp_suav))

            if mejora > FIJAR_UMBRAL_MEJORA:
                self.fase_paso_fijar = min(self.fase_paso_fijar * 1.4,
                                           FIJAR_PASO_MAX)
            elif mejora < -FIJAR_UMBRAL_MEJORA:
                self.signo_fase_fijar *= -1.0
                self.fase_paso_fijar = max(self.fase_paso_fijar * 0.6,
                                           FIJAR_PASO_MIN)
                stats.n_anti_oscilacion += 1
            else:
                self.fase_paso_fijar = min(self.fase_paso_fijar * 1.25,
                                           FIJAR_PASO_MAX)

            self.sub_estado = "COOLDOWN"
            self.cnt = 0
            return self._resp(f"EVAL_FIJAR(m={mejora:+.4f})")

        # === COOLDOWN corto ===
        if self.sub_estado == "COOLDOWN":
            self.cnt += 1
            if self.cnt >= FIJAR_COOLDOWN:
                self.sub_estado = "MOVER"
                self.cnt = 0
            return self._resp("COOL_FIJAR")

        # === MOVER ===
        if self.sub_estado == "MOVER":
            # Return-to-best si estamos perdidos y teníamos un buen punto
            if (self.dist_min_fijar < 0.005
                    and dist > self.dist_min_fijar * FIJAR_RETURN_BEST
                    and abs(self.fase_cmd - self.fase_mejor_fijar) > 0.5):
                self.fase_cmd = self.fase_mejor_fijar
                self.n_cambios_fase += 1
                self.samples_since_fase = 0
                self.n_return_best_fijar += 1
                stats.n_return_to_best += 1
                self.sub_estado = "COOLDOWN"
                self.cnt = 0
                return self._resp("RETURN_BEST_FIJAR", cambio_fase=True)

            if dist > FIJAR_ACTIVO_UMBRAL:
                self.fp_antes_paso = self.fp_suav
                self.fase_cmd = envolver_fase(
                    self.fase_cmd
                    + self.signo_fase_fijar * self.fase_paso_fijar)
                self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
                self.n_cambios_fase += 1
                self.samples_since_fase = 0
                stats.n_micro_fase += 1
                self.sub_estado = "EVAL_FASE"
                self.cnt = 0
                return self._resp(
                    f"FIJAR_MOVE{self.signo_fase_fijar*self.fase_paso_fijar:+.2f}",
                    cambio_fase=True)

            self.n_hold_fijar += 1
            return self._resp("FIJAR_HOLD")

        return self._resp("FIJAR_IDLE")


# ==============================================================================
# LOGGING
# ==============================================================================

def encabezado():
    print("-" * 170)
    print(f"{'t':>5} | {'FP':>8} | {'FPflt':>8} | {'Fase':>8} | "
          f"{'Δf':>9} | {'FrecFlt':>9} | "
          f"{'Modo':>8} | {'T.Range':>8} | {'Streak':>7} | "
          f"{'Dist':>6} | {'Acción':>24} | Estado")
    print("-" * 170)


def fila(t, fp_med, res, en_rango, streak, t_in_range, t_total):
    estado = "OK" if en_rango else "OUT"
    if res["modo"] == "FIJAR":
        estado = "LOCK" if en_rango else "LOCK-OUT"
    if res["modo"] == "BUSCAR":
        estado = "SRCH"
    if streak >= UMBRAL_STREAK_1:
        estado = f"★{streak:.0f}s"
    pct_in = 100 * t_in_range / max(t_total, 0.1)
    dist = abs(res["fp"] - FP_OBJETIVO)
    print(f"{t:>5} | {fp_med:>+8.4f} | {res['fp']:>+8.4f} | "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+9.5f} | "
          f"{res['frecuencia_filt']:>9.4f} | "
          f"{res['modo']:>8} | {pct_in:>6.1f}% | {streak:>6.1f}s | "
          f"{dist:>6.4f} | {res['accion']:>24} | {estado}")


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 170)
    titulo = "CONTROL DE FP v9.7 — HILL-CLIMB DECISIVO SOBRE FASE" if MODO_EXTREME \
             else "CONTROL DE FP v9.7 — STREAMING"
    print(f" {titulo}")
    print("=" * 170)

    preguntar_objetivos()

    modo_io = "EXTREME" if MODO_EXTREME else "streaming"
    print(f"\n Modo I/O: {modo_io}")
    if MODO_EXTREME:
        print(f" Intervalo EXTREME:       {INTERVALO_EXTREME:.4f} s  "
              f"({'sin sleep' if INTERVALO_EXTREME <= 0 else 'con sleep'})")
    else:
        print(f" Muestreo:                {1/INTERVALO_MUESTREO:.0f} Hz")
    print(f" FIJAR: hill-climb fase (KP={KP_FIJAR}, KI={KI_FIJAR}, "
          f"FREEZE_FREQ={FIJAR_FREEZE_FREQ})")
    print(f" FIJAR parámetros: dead-zone={FIJAR_ACTIVO_UMBRAL}, "
          f"cooldown={FIJAR_COOLDOWN}, eval={FIJAR_EVAL_ITER}, "
          f"paso {FIJAR_PASO_MIN}°-{FIJAR_PASO_MAX}°")
    print(f" Objetivo: FP={FP_OBJETIVO:+.4f} ({TIPO_CARGA}) | "
          f"Banda: [{FP_MIN_RANGO:+.4f}, {FP_MAX_RANGO:+.4f}]")
    print("=" * 170)

    fg = None
    wt = None
    controlador = ControladorFPv9()
    stats = Estadisticas()

    total = 0
    en_rango = 0
    t_ctrl = None
    last_freq_written = None

    try:
        if MODO_EXTREME:
            print("\n[1/3] Conectando equipos (EXTREME)...")
            fg = YokogawaFG420(DIR_FG, mode='extreme')
            wt = YokogawaWT3000(DIR_WT, mode='extreme')
            fg.conectar()
            wt.conectar()
            print(f"  FG420:  {fg.obtener_idn()[:60]}")
            print(f"  WT3000: {wt.obtener_idn()[:60]}")

            print("  ► Aplicando extreme() en FG420...")
            fg.extreme(
                canal=1,
                frecuencia_hz=FREC_NOMINAL,
                amplitud_vpp=AMPLITUD_FG,
                offset_v=OFFSET_V_FG,
                fase_grados=0.0,
                encender_salida=True,
            )
            print("  ► Aplicando extreme() en WT3000...")
            wt.extreme(
                elemento_entrada=ELEMENTO_WT,
                incluir_potencias=True,
                configurar_salida=True,
            )
        else:
            print("\n[1/3] Conectando equipos (streaming)...")
            fg = YokogawaFG420(DIR_FG, mode='streaming')
            wt = YokogawaWT3000(DIR_WT, mode='streaming')
            fg.conectar()
            wt.conectar()
            print(f"  FG420:  {fg.obtener_idn()[:60]}")
            print(f"  WT3000: {wt.obtener_idn()[:60]}")

            wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)
            fg.configurar_canal_rapido(
                canal=1, funcion="SIN",
                frecuencia_hz=FREC_NOMINAL,
                amplitud_vpp=AMPLITUD_FG,
                offset_v=OFFSET_V_FG,
                fase_grados=0.0,
                activar_salida=True,
            )

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
                if MODO_EXTREME:
                    m = wt.leer_mediciones_minimas()
                else:
                    m = wt.leer_mediciones_estandar()
            except Exception as e:
                print(f"[X] Error lectura: {e}")
                time.sleep(0.001)
                continue

            if wt.is_outlier():
                continue

            fp_med = m.get("factor_potencia")
            f_med = m.get("frecuencia")
            if fp_med is None or f_med is None:
                time.sleep(0.001)
                continue

            fp_val = float(fp_med)
            total += 1
            en_rango_local = FP_MIN_RANGO <= fp_val <= FP_MAX_RANGO
            if en_rango_local:
                en_rango += 1

            res = controlador.actualizar(fp_val, f_med, dt, stats)

            # v9.7: escribir frecuencia solo si cambia de verdad
            f_out = res["frecuencia"]
            if (last_freq_written is None
                    or abs(f_out - last_freq_written) > FREQ_WRITE_EPS):
                if MODO_EXTREME:
                    fg.establecer_frecuencia_extreme(1, f_out)
                else:
                    fg.establecer_frecuencia_streaming(1, f_out)
                last_freq_written = f_out

            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_val, res["fase"], f_med,
                            res["frecuencia_filt"], res["delta_f"],
                            res["modo"], dt,
                            integral=res.get("integral_fijar", 0.0))

            t_rel = t_now - t_ctrl
            evento_streak = stats.update_streak(en_rango_local, dt, t_rel)
            if evento_streak:
                print(f"\n★ ★ ★  RACHA DE {evento_streak} ALCANZADA EN "
                      f"t={t_rel:.1f}s  ★ ★ ★\n")

            if total % 3 == 0 or res["cambio_fase"]:
                fila(int(t_rel), fp_val, res, en_rango_local,
                     stats.streak_actual, stats.t_in_range, t_rel)

            if MODO_EXTREME:
                if INTERVALO_EXTREME > 0:
                    elapsed = time.time() - t_now
                    if elapsed < INTERVALO_EXTREME:
                        time.sleep(INTERVALO_EXTREME - elapsed)
            else:
                elapsed = time.time() - t_now
                if elapsed < INTERVALO_MUESTREO:
                    time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(controlador, time.time() - t_ctrl,
                      total, en_rango, fg=fg, wt=wt)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(controlador, time.time() - t_ctrl,
                          total, en_rango, fg=fg, wt=wt)

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