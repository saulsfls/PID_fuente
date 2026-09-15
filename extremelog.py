"""
CONTROL DE FP v9.3 — PLL CON SIGNO CORREGIDO  [MODO EXTREME]
================================================================================

OBJETIVO
--------
Mantener el factor de potencia (FP) del sistema lo más cerca posible de 1.0,
ajustando continuamente la señal del generador Yokogawa FG420 para que su fase
respecto a la red quede alineada.

¿QUÉ MIDE EL WT3000?
--------------------
El analizador de potencias WT3000 nos entrega dos cantidades clave cada
muestra:
    - PHI (ángulo V-I): fase entre tensión y corriente en grados.
    - FP   (factor de potencia): cos(PHI). Está directamente relacionado.

Cuando PHI ≈ 0 → FP ≈ 1.0 → fase alineada → objetivo cumplido.
Cuando PHI ≠ 0 → FP < 1.0 → hay desalineación → hay que corregir.

¿CÓMO SE CORRIGE?
-----------------
Hay dos actuadores disponibles en el FG420:
    1. FASE_Cmd (φ): cambia la fase del FG instantáneamente. Es un actuador
       DISCRETO, perturba el sistema (cada salto se nota en PHI).
    2. Δf (Hz): cambia la frecuencia del FG respecto a la nominal 60 Hz.
       Es un actuador CONTINUO: con Δf ≠ 0 la fase RELATIVA FG↔red deriva
       de forma controlada a razón de 360·Δf grados por segundo.

La estrategia v9.3 usa AMBOS:
    - En BUSCAR (|PHI| grande, > 50°): se mueve la FASE a saltos para
      llegar rápido a la zona buena. Δf = 0 para no perturbar.
    - En PLL  (|PHI| pequeña, < 25°): la FASE queda CONGELADA y todo el
      control lo hace Δf mediante un lazo tipo PLL (Phase-Locked Loop).

FÍSICA DEL MODELO (por qué el PLL funciona)
-------------------------------------------
La fase relativa φ_rel(t) entre el FG y la red evoluciona así:

    dφ_rel/dt = 360 · (f_FG - f_red) = 360 · Δf     [grados/segundo]

Como la fase del FG está congelada durante el PLL, mover Δf es equivalente
a mover la fase relativa a una velocidad proporcional.

A su vez, el PHI medido por el WT3000 responde al cambio de φ_rel con una
ganancia llamada "slope" (pendiente):

    dPHI/dφ_rel = slope     (típicamente slope ∈ [-1.5, -0.3])

Es decir: aumentar la fase del FG reduce el PHI (por la convención física
del sistema). Slope es NEGATIVO.

Entonces, para llevar PHI → 0 mediante Δf:

    dPHI/dt = slope · 360 · Δf

Queremos que dPHI/dt = -k·PHI (realimentación proporcional negativa).
Despejando:

    Δf = -k · PHI / slope

El signo es CLAVE: dividir por slope (no multiplicar). Con slope<0 y PHI<0,
Δf resulta NEGATIVO, lo que hace dPHI/dt POSITIVO y sube PHI hacia 0.
Este fue el bug que se corrigió respecto a v9.2.

CONTROLADOR PI SOBRE EL ERROR
-----------------------------
El PLL implementa un controlador PI clásico:

    err     = -PHI / slope        ← error en unidades de "Δf necesario"
    df_p    = Kp · err            ← término proporcional (reacción rápida)
    df_i   += Ki · err · dt       ← término integral (elimina offset persistente)
    Δf      = df_p + df_i         ← salida del controlador
    f_FG    = 60.0 + Δf           ← frecuencia que se envía al FG420

El término P reacciona al instante al PHI medido.
El término I integra el error en el tiempo y corrige la diferencia entre
60.0 Hz nominal y la frecuencia real de la red (que puede estar a 59.98
o 60.02 Hz, etc.). Sin el término I, el PLL tendría un offset permanente.

¿POR QUÉ HAY QUE DETECTAR "LECTURAS FRESCAS" vs "STALE"?
--------------------------------------------------------
El WT3000 no actualiza sus mediciones a 20 Hz. Internamente promedia
durante ~250-500 ms y entrega el MISMO valor varias veces consecutivas.
Eso significa que si tomamos 20 muestras por segundo, sólo 2-4 son
realmente nuevas; el resto son copias de la anterior.

Integrar Δf sobre datos "stale" duplicaría el efecto del error (porque
el dt transcurre pero el PHI no cambia). Por eso:
    - Sólo actualizamos φ_fresh cuando el PHI medido difiere del anterior.
    - El integrador sólo avanza en muestras frescas.
    - El dt efectivo es el tiempo transcurrido desde la última fresca
      (con un tope de 0.6 s para evitar sobrecorrecciones).

DETECCIÓN DE OUTLIERS
---------------------
Ocasionalmente el WT3000 da un valor espurio (glitch de comunicación o
transitorio). Si el salto entre dos lecturas frescas consecutivas supera
OUTLIER_JUMP (=35°), se descarta y no se propaga al controlador.
"""
import time
import numpy as np
from collections import deque
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000


# ==============================================================================
# CONFIGURACIÓN GLOBAL
# ==============================================================================
# Direcciones GPIB de los instrumentos
DIR_FG = "GPIB1::2::INSTR"          # Generador de funciones Yokogawa FG420
DIR_WT = "GPIB0::1::INSTR"          # Analizador de potencias Yokogawa WT3000

ELEMENTO_WT = 1                     # Canal de entrada del WT a medir (1=primario)

TIEMPO_PRUEBA_SEG = 300             # Duración total del lazo de control (segundos)
INTERVALO_MUESTREO = 0.05           # Tiempo mínimo entre iteraciones (20 Hz)

# Rango de FP considerado "aceptable" para reportar efectividad
FP_MIN_RANGO = 0.970                # Límite inferior de FP en rango
FP_MAX_RANGO = 1.030                # Límite superior (por si el WT reporta >1)
FP_TIGHT = 0.990                    # Umbral "muy bueno" para reportar

# Parámetros de la señal de salida del FG420
AMPLITUD_FG = 5.0                   # Amplitud Vpp de la senoidal
OFFSET_V_FG = 0.0                   # Offset DC de la señal
FASE_MIN, FASE_MAX = -180.0, 180.0  # Rango permitido para fase del FG
FREC_NOMINAL = 60.0                 # Frecuencia nominal de la red (Hz)

# ----- Umbrales de transición BUSCAR ↔ PLL (histéresis) -----------------------
# Se usan DOS umbrales para evitar oscilar entre modos:
#   - Para ENTRAR a BUSCAR:  |PHI| > PHI_BUSCAR_ENTER (valor grande)
#   - Para SALIR de BUSCAR:  |PHI| < PHI_BUSCAR_EXIT  (valor más pequeño)
PHI_BUSCAR_ENTER = 50.0             # |PHI| > 50° durante 3 frescas → BUSCAR
PHI_BUSCAR_EXIT  = 25.0             # |PHI| < 25° durante 2 frescas → PLL
N_FRESH_ENTER_BUSCAR = 3            # Nº de frescas consecutivas para entrar
N_FRESH_EXIT_BUSCAR  = 2            # Nº de frescas consecutivas para salir

# ----- Parámetros del modo BUSCAR (búsqueda discreta de fase) ------------------
PASO_BUSCAR_INICIAL = 20.0          # Tamaño inicial de paso (°)
PASO_BUSCAR_MIN     = 8.0           # Paso mínimo permitido
PASO_BUSCAR_MAX     = 30.0          # Paso máximo permitido (evita saltos grandes)
N_SETTLE_BUSCAR = 6                 # Muestras a esperar tras mover fase antes de evaluar

# ----- Parámetros del PLL (control continuo por frecuencia) --------------------
# Kp: ganancia proporcional. Multiplica directamente al error.
#     Más Kp → reacción más rápida pero puede oscilar.
# Ki: ganancia integral. Acumula error en el tiempo.
#     Más Ki → elimina offset persistente más rápido pero puede sobrecorregir.
KP_PLL = 0.0025                     # Ganancia proporcional (Hz por °/slope)
KI_PLL = 0.00030                    # Ganancia integral (Hz por °·s/…)
DF_MAX = 0.15                       # Saturación de |Δf| (Hz). ±150 mHz suficiente
                                    # para cubrir variaciones normales de red.
DF_LP  = 0.30                       # Suavizado exponencial de Δf (α del filtro)
                                    # α pequeño = más suave, α grande = más reactivo.
EFF_DT_MAX = 0.6                    # Cap del dt efectivo (s) para no sobreintegrar
                                    # cuando hay huecos largos entre frescas.

# ----- Slope (dPHI/dφ_rel): relación lineal entre fase del FG y PHI medido -----
# Se inicializa con un valor negativo típico para que el PLL arranque sin
# necesidad de aprender primero. Luego se actualiza con EMA conforme
# observamos movimientos de fase.
SLOPE_INIT = -0.8                   # Valor inicial (negativo: subir φ_FG baja PHI)
SLOPE_MIN_ABS = 0.25                # |slope| mínimo para usarlo en el control
                                    # (por debajo de esto, slope es ruido)
SLOPE_SAMPLES_INIT = 20             # Peso inicial alto = confianza media-baja
                                    # en el valor por defecto.

# ----- Detección de staleness y outliers -------------------------------------
STALE_TOL = 0.05                    # ° — si PHI cambia menos que esto, es stale
OUTLIER_JUMP = 35.0                 # ° — saltos mayores se descartan como glitch

# ----- Umbral de "lock" (para métricas de tiempo) ----------------------------
PHI_LOCKED = 5.0                    # |PHI| < 5° se considera "bloqueado"


# ==============================================================================
# FUNCIONES AUXILIARES
# ==============================================================================
def envolver_fase(a):
    """Envuelve un ángulo a [-180°, +180°]. Esencial porque PHI del WT3000
    viene ocasionalmente en [0°, 360°] y hay que normalizarlo."""
    return ((a + 180.0) % 360.0) - 180.0


def clamp(v, lo, hi):
    """Satura v al rango [lo, hi]."""
    return max(lo, min(hi, v))


# ==============================================================================
# RUNTRACKER — mide rachas de permanencia en un estado
# ==============================================================================
class RunTracker:
    """
    Rastrea rachas (runs) consecutivas en las que una condición es verdadera.

    Ejemplo: "FP estuvo entre 0.97 y 1.03 durante 45 s seguidos antes de salir".
    Útil para reportar cuánto dura el sistema en cada estado (bueno/malo),
    y no sólo el % de muestras — que sería engañoso (podría ser 50% en rango
    pero con rachas de 1 muestra cada una, que es inútil).

    Acumula:
        - Duración en tiempo y muestras de cada racha, tanto de la actual
          como de todas las cerradas.
        - Estadísticas: máxima, media, mediana, total.
    """
    def __init__(self, name=""):
        self.name = name
        # Racha actual (aún abierta)
        self.current_samples = 0
        self.current_time = 0.0
        # Récord de todas las rachas
        self.max_samples = 0
        self.max_time = 0.0
        # Acumuladores globales
        self.total_in_run_samples = 0
        self.total_in_run_time = 0.0
        self.total_samples = 0
        self.total_time = 0.0
        # Historial de rachas cerradas
        self.run_lengths = []       # en muestras
        self.run_times = []         # en segundos

    def update(self, cond, dt):
        """Llamar en cada muestra. `cond` es True si estamos en racha."""
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
        """Cierra la racha actual y la guarda en el historial."""
        if self.current_samples > 0:
            self.run_lengths.append(self.current_samples)
            self.run_times.append(self.current_time)
            self.current_samples = 0
            self.current_time = 0.0

    def close(self):
        """Cierra la última racha al terminar la prueba."""
        self._close_run()

    def summary(self):
        """Resumen agregado para imprimir al final."""
        n = len(self.run_lengths)
        pct_time = 100 * self.total_in_run_time / max(self.total_time, 1e-6)
        if n == 0:
            return {"n": 0, "max_time": 0.0, "max_samples": 0,
                    "mean_time": 0.0, "median_time": 0.0,
                    "pct_time": pct_time,
                    "total_time_in": self.total_in_run_time}
        return {
            "n": n,
            "max_time": self.max_time,
            "max_samples": self.max_samples,
            "mean_time": float(np.mean(self.run_times)),
            "median_time": float(np.median(self.run_times)),
            "pct_time": pct_time,
            "total_time_in": self.total_in_run_time,
        }

    def histogram(self, bins):
        """Histograma de duraciones de racha con los límites dados en `bins`."""
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
# ESTADÍSTICAS — acumula métricas y genera el reporte final
# ==============================================================================
class Estadisticas:
    """
    Acumula todo lo relevante de la corrida:
        - Series temporales de FP, PHI, fase, frecuencia, Δf.
        - Contadores de frescas/stale/outliers.
        - Tiempo acumulado en cada modo.
        - Rachas de permanencia (4 condiciones monitoreadas).
        - Instantes de trazabilidad (primer lock, primer FP≥0.99).
    """
    def __init__(self):
        # Series temporales (para estadística final)
        self.fp_hist = []
        self.phi_hist = []
        self.fase_hist = []
        self.frec_hist = []
        self.df_hist = []
        # Tiempo total acumulado en cada modo del controlador
        self.tiempos_modo = {"BUSCAR": 0.0, "PLL": 0.0}
        # Histograma de FP (10 bins de 0.1)
        self.bins_fp = np.zeros(10)
        # Contadores
        self.n_transiciones = 0
        self.n_stale = 0
        self.n_fresh = 0
        self.n_outliers = 0

        # Rastreadores de racha — 4 condiciones de interés
        self.run_fp_range = RunTracker("FP en rango [0.97, 1.03]")
        self.run_fp_tight = RunTracker("FP>=0.99")
        self.run_phi_lock = RunTracker("|PHI| < 5°")
        self.run_out = RunTracker("FP fuera rango")

        # Trazabilidad temporal
        self.tiempo_primer_lock = None
        self.tiempo_primer_tight = None

    def registrar(self, fp, phi, fase, frec, df, modo, dt):
        """Registrar una muestra completa."""
        self.fp_hist.append(fp)
        self.phi_hist.append(phi)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec)
        self.df_hist.append(df)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        # Bin del histograma de FP (0.0-0.1 → bin 0, ..., 0.9-1.0 → bin 9)
        self.bins_fp[min(int(abs(fp) * 10), 9)] += 1

        # Actualizar los rastreadores de racha
        in_range = FP_MIN_RANGO <= abs(fp) <= FP_MAX_RANGO
        self.run_fp_range.update(in_range, dt)
        self.run_fp_tight.update(abs(fp) >= FP_TIGHT, dt)
        self.run_phi_lock.update(abs(phi) < PHI_LOCKED, dt)
        self.run_out.update(not in_range, dt)

    def cerrar_rachas(self):
        """Cerrar todas las rachas al final."""
        self.run_fp_range.close()
        self.run_fp_tight.close()
        self.run_phi_lock.close()
        self.run_out.close()

    def _imprimir_rachas(self, tracker, bins):
        """Imprime un bloque de información para un rastreador de racha."""
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
        """Genera el reporte final completo."""
        self.cerrar_rachas()
        print("\n" + "=" * 90)
        print(" RESUMEN FINAL v9.3  [MODO EXTREME] — PLL signo corregido")
        print("=" * 90)

        # -- Tiempo y muestras --
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

        # -- Efectividad --
        if n_iter > 0:
            ef = 100 * en_rango / n_iter
            print(f"\n[ Efectividad ]")
            print(f"  En rango (1±3%):         {en_rango} ({ef:.1f}%)")

        # -- Distribución de tiempo por modo --
        print(f"\n[ Tiempo por modo ]")
        tot = sum(self.tiempos_modo.values()) or 1.0
        for m, t in self.tiempos_modo.items():
            pct = 100 * t / tot
            barra = "#" * int(pct / 2)
            print(f"  {m:<10} {t:7.1f}s ({pct:5.1f}%)  {barra}")

        # -- Distribución de FP en 10 bins --
        n_tot = self.bins_fp.sum()
        if n_tot > 0:
            print(f"\n[ Distribución de FP ]")
            for i in range(9, -1, -1):
                pct = 100 * self.bins_fp[i] / n_tot
                barra = "#" * int(pct / 2)
                lo = i * 0.1
                print(f"  [{lo:.1f}-{lo+0.1:.1f}] {int(self.bins_fp[i]):5d} "
                      f"({pct:5.1f}%)  {barra}")

        # -- Rachas de permanencia --
        print(f"\n[ Rachas de permanencia ]")
        bins_t = [0, 0.5, 2.0, 5.0, 15.0, 30.0, 60.0, np.inf]
        self._imprimir_rachas(self.run_fp_range, bins_t)
        self._imprimir_rachas(self.run_fp_tight, bins_t)
        self._imprimir_rachas(self.run_phi_lock, bins_t)
        self._imprimir_rachas(self.run_out, bins_t)

        # -- Trazabilidad --
        print(f"\n[ Trazabilidad ]")
        print(f"  Mejor FP:                {controlador.mejor_fp:.4f}")
        print(f"  Mejor |PHI|:             {controlador.mejor_phi_abs:.3f}°")
        print(f"  Fase en mejor punto:     {controlador.mejor_fase:+.2f}°")
        if self.tiempo_primer_lock is not None:
            print(f"  1er lock (|PHI|<5°):     {self.tiempo_primer_lock:.2f} s")
        if self.tiempo_primer_tight is not None:
            print(f"  1er FP>=0.99:            {self.tiempo_primer_tight:.2f} s")

        # -- PLL: parámetros y comportamiento observado --
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

        # -- Frecuencia de red observada --
        if self.frec_hist:
            uf = self.frec_hist[-100:] if len(self.frec_hist) >= 100 else self.frec_hist
            print(f"\n[ Frecuencia de red medida ]")
            print(f"  Promedio (últimas):      {np.mean(uf):.4f} Hz")
            print(f"  Desviación:              {np.std(uf):.5f} Hz")
            print(f"  Rango global:            {np.min(self.frec_hist):.4f} - "
                  f"{np.max(self.frec_hist):.4f} Hz")

        # -- Veredicto final --
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
    """
    Máquina de estados con dos modos:

        BUSCAR:  |PHI| > 50°  →  mueve la fase del FG a saltos de ~20°.
                 Objetivo: acercarse rápido a la zona |PHI|<25°.
                 Δf = 0 (no perturbar con frecuencia).

        PLL:     |PHI| < 25°  →  congela la fase del FG y controla Δf
                 con un PI sobre err = -PHI/slope.
                 Objetivo: llevar PHI → 0 suavemente y mantenerlo.

    Variables internas clave:
        fase_cmd        : fase actual enviada al FG (grados)
        delta_f         : Δf actual (Hz) — salida del PI en modo PLL
        df_integral     : acumulador del término I
        phi_fresh       : última lectura válida de PHI (no stale, no outlier)
        slope           : dPHI/dφ_rel aprendido (o el valor por defecto)
        dt_since_fresh  : tiempo desde la última lectura fresca (para integrar)
    """

    def __init__(self):
        # ----- Actuadores -----
        self.fase_cmd = 0.0             # fase actual del FG (grados)
        self.delta_f = 0.0              # Δf actual (Hz) — salida total
        self.df_integral = 0.0          # acumulador del término integral del PI

        # ----- Estado de la máquina -----
        self.modo = "BUSCAR"            # modo actual: "BUSCAR" o "PLL"
        self.cnt = 0                    # contador de muestras desde último evento
        self.move_pending = False       # True si acabamos de mover la fase
        self.fresh_after_move = False   # True si ya llegó una fresca tras mover

        # ----- Estado de PHI -----
        self.phi_prev_raw = None        # PHI crudo de la muestra anterior (para
                                        # detectar staleness: si el nuevo es igual
                                        # al anterior, es stale)
        self.phi_fresh = 0.0            # última lectura fresca y válida
        self.phi_stale = True           # True si la lectura actual es stale
        self.stale_run = 0              # nº consecutivo de muestras idénticas

        # ----- Slope (pendiente dPHI/dφ_rel) -----
        # Se inicializa con valor negativo típico para que el PLL arranque
        # sin necesidad de aprender primero (si no, los primeros segundos
        # el PLL no sabría en qué dirección empujar).
        self.slope = SLOPE_INIT         # empieza en -0.8
        self.slope_samples = SLOPE_SAMPLES_INIT  # peso inicial (baja confianza)
        self.fase_prev = None           # fase antes del último movimiento
        self.phi_prev = None            # PHI antes del último movimiento

        # ----- Búsqueda (modo BUSCAR) -----
        self.paso_buscar = PASO_BUSCAR_INICIAL  # tamaño actual de paso (°)
        self.dir_buscar = +1.0                  # dirección actual de búsqueda

        # ----- Contadores -----
        self.n_cambios_fase = 0         # total de movimientos de fase
        self.dt_since_fresh = 0.0       # acumulador de tiempo desde última fresca

        # ----- Histéresis BUSCAR↔PLL -----
        # Necesitamos N frescas consecutivas para cambiar de modo. Evita
        # oscilar cuando PHI está en el límite.
        self.fresh_enter_buscar = 0
        self.fresh_exit_buscar = 0

        # ----- Mejor histórico (menor |PHI|) -----
        self.mejor_phi_abs = 180.0
        self.mejor_fase = 0.0
        self.mejor_fp = 0.0

    # ------------------------------------------------------------------
    def _resp(self, accion, f_fg, cambio_fase=False, fresh=False):
        """Construye el diccionario de respuesta que consume el main()."""
        return {
            "accion": accion,               # etiqueta legible de lo que se hizo
            "fase": self.fase_cmd,          # fase actual a enviar al FG
            "delta_f": self.delta_f,        # Δf actual (a sumar a 60 Hz)
            "frecuencia": f_fg,             # frecuencia total a enviar al FG
            "cambio_fase": cambio_fase,     # si hay que escribir fase al FG
            "modo": self.modo,              # modo actual (BUSCAR o PLL)
            "phi": self.phi_fresh,          # último PHI fresco
            "fresh": fresh,                 # si la lectura actual es fresca
            "stale": self.phi_stale,        # si la lectura actual es stale
            "mejor_fp": self.mejor_fp,      # mejor FP alcanzado
            "mejor_fase": self.mejor_fase,  # fase en el mejor FP
            "mejor_phi_abs": self.mejor_phi_abs,
            "slope": self.slope if self.slope is not None else 0.0,
            "paso": self.paso_buscar,
        }

    # ------------------------------------------------------------------
    def _mover_fase(self, delta):
        """
        Aplica un movimiento de fase al FG. Registra el estado previo para
        que, cuando llegue una lectura FRESCA, podamos aprender el slope
        (cuánto cambió PHI por unidad de cambio de fase).

        Devuelve True si hubo movimiento efectivo.
        """
        if abs(delta) < 0.3:
            return False  # ignorar movimientos microscópicos
        # Guardar estado ANTES de mover (para el aprendizaje de slope)
        self.fase_prev = self.fase_cmd
        self.phi_prev = self.phi_fresh
        # Aplicar movimiento
        self.fase_cmd = envolver_fase(self.fase_cmd + delta)
        self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
        self.n_cambios_fase += 1
        # Marcar que estamos esperando una lectura fresca post-movimiento
        self.move_pending = True
        self.fresh_after_move = False
        self.cnt = 0
        return True

    # ------------------------------------------------------------------
    def _actualizar_phi(self, phi_raw_deg, stats=None):
        """
        Actualiza el estado interno de PHI.

        Lógica:
            1. Envolver el PHI crudo a [-180, 180].
            2. Si es idéntico al anterior (dentro de STALE_TOL), marcar stale.
            3. Si cambió mucho más de OUTLIER_JUMP, rechazar como outlier.
            4. Si cambió razonablemente, ACEPTAR como fresca.

        Devuelve True si la lectura es fresca y válida (apta para control).
        """
        phi_w = envolver_fase(phi_raw_deg)

        # Primera lectura: aceptar sin más
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.stale_run = 0
            return True

        # ¿Es idéntico al anterior? → stale (el WT no actualizó)
        if abs(phi_w - self.phi_prev_raw) < STALE_TOL:
            self.stale_run += 1
            self.phi_stale = True
            return False

        # Cambió algo. ¿Es un salto razonable o un outlier?
        jump = abs(envolver_fase(phi_w - self.phi_fresh))
        if jump > OUTLIER_JUMP:
            if stats is not None:
                stats.n_outliers += 1
            # Rechazamos el valor: NO actualizamos phi_fresh.
            # Pero tampoco es stale duro (el WT sí actualizó). Lo marcamos
            # como "no fresh" para que no afecte al control.
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            return False

        # Cambio razonable → aceptar como fresca
        self.phi_fresh = phi_w
        self.phi_prev_raw = phi_w
        self.phi_stale = False
        self.stale_run = 0
        if self.move_pending:
            # ¡Ya llegó la fresca después del último movimiento de fase!
            # Esto habilita el aprendizaje de slope.
            self.fresh_after_move = True
        return True

    # ------------------------------------------------------------------
    def _actualizar_slope(self):
        """
        Aprende la pendiente dPHI/dφ_rel comparando el PHI antes y después
        del último movimiento de fase.

        Sólo se ejecuta cuando:
            - Hicimos un movimiento (move_pending=True)
            - Ya llegó una lectura FRESCA después del movimiento
            - Tenemos referencias de antes del movimiento

        Filtra con EMA (media exponencial) para suavizar el ruido.
        """
        if not self.move_pending or not self.fresh_after_move:
            return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False
            return

        # Diferencia de PHI (envuelta por si cruzó ±180°)
        dphi = envolver_fase(self.phi_fresh - self.phi_prev)
        # Diferencia de fase comandada (envuelta)
        dfase = envolver_fase(self.fase_cmd - self.fase_prev)

        # Requerir cambios mínimos para que la medición sea significativa
        if abs(dfase) < 3.0 or abs(dphi) < 0.3:
            self.move_pending = False
            return

        slope_new = dphi / dfase
        # Rechazar slopes absurdas (fuera del rango físico plausible)
        if abs(slope_new) > 5.0 or abs(slope_new) < 0.05:
            self.move_pending = False
            return

        # Actualizar con EMA (más peso a los valores nuevos cuando hay pocas
        # muestras; menos peso cuando ya tenemos confianza).
        if self.slope is None:
            self.slope = slope_new
            self.slope_samples = 1
        else:
            w = max(1.0 / (self.slope_samples + 1), 0.15)
            self.slope = (1 - w) * self.slope + w * slope_new
            self.slope_samples += 1
        self.move_pending = False

    # ------------------------------------------------------------------
    def _cambiar_modo(self, nuevo, stats):
        """Cambia de modo y resetea contadores relevantes."""
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
        """
        Punto de entrada del controlador. Se llama UNA vez por iteración
        del lazo, con la última medición del WT3000.

        Parámetros:
            fp_med: factor de potencia medido (dimensionless)
            phi_med: ángulo V-I medido (grados, del WT3000)
            f_med: frecuencia de la red medida (Hz)
            dt: tiempo transcurrido desde la última llamada (s)
            stats: objeto Estadisticas para acumular métricas

        Devuelve: diccionario con la acción y los valores a aplicar al FG.
        """
        if phi_med is None:
            # Sin medición: no podemos hacer nada, devolvemos respuesta nula
            return self._resp("SIN_PHI", FREC_NOMINAL + self.delta_f)

        # 1) Actualizar PHI interno (fresca / stale / outlier)
        fresh = self._actualizar_phi(phi_med, stats)

        # 2) Contadores de tiempo y estadísticas
        self.cnt += 1
        self.dt_since_fresh += dt  # acumula tiempo desde última fresca
        if fresh:
            self.dt_since_fresh = 0.0
            if stats is not None:
                stats.n_fresh += 1
        elif stats is not None:
            stats.n_stale += 1

        # 3) FP absoluto para logging
        fp_abs = abs(fp_med) if fp_med is not None \
            else abs(np.cos(np.radians(self.phi_fresh)))

        # 4) Mejor histórico (menor |PHI|)
        if fresh and abs(self.phi_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.phi_fresh)
            self.mejor_fase = self.fase_cmd
            self.mejor_fp = fp_abs

        # 5) Aprendizaje del slope (sólo si hay movimiento pendiente)
        self._actualizar_slope()

        # 6) Contadores de histéresis (sólo frescas)
        if fresh:
            if abs(self.phi_fresh) > PHI_BUSCAR_ENTER:
                self.fresh_enter_buscar += 1
            else:
                self.fresh_enter_buscar = 0

            if abs(self.phi_fresh) < PHI_BUSCAR_EXIT:
                self.fresh_exit_buscar += 1
            else:
                self.fresh_exit_buscar = 0

        # 7) Transición de modo (si corresponde)
        if self.modo == "PLL" and self.fresh_enter_buscar >= N_FRESH_ENTER_BUSCAR:
            # PHI se fue muy grande → volver a BUSCAR
            self._cambiar_modo("BUSCAR", stats)
        elif self.modo == "BUSCAR" and self.fresh_exit_buscar >= N_FRESH_EXIT_BUSCAR:
            # PHI ya está en zona buena → entrar a PLL
            self._cambiar_modo("PLL", stats)

        # 8) Ejecutar el modo actual
        if self.modo == "BUSCAR":
            return self._buscar(fresh)
        else:
            return self._pll(fresh)

    # ------------------------------------------------------------------
    def _buscar(self, fresh):
        """
        MODO BUSCAR: la fase del FG se mueve a saltos para acercarse a
        |PHI| < 25°. Δf se mantiene en 0.

        Pasos:
            1. Si la lectura no es fresca → esperar.
            2. Si acabamos de mover fase, esperar N_SETTLE_BUSCAR muestras
               antes de decidir el siguiente paso (evita reaccionar a
               lecturas intermedias del WT).
            3. Decidir dirección y tamaño del siguiente paso:
                - Si conocemos slope (aprendido o por defecto), calcular el
                  paso que llevaría PHI a 0:  δφ = -PHI/slope
                - Si no, usar la última dirección conocida (dir_buscar).
            4. Aplicar el paso con _mover_fase().
        """
        # Sin lectura fresca no podemos decidir
        if not fresh:
            return self._resp("BUSCAR-stale", FREC_NOMINAL + self.delta_f)

        # Esperar settle tras el último movimiento
        if self.cnt < N_SETTLE_BUSCAR:
            return self._resp("BUSCAR-settle", FREC_NOMINAL + self.delta_f)

        # En BUSCAR, Δf = 0 (no perturbar con frecuencia)
        self.delta_f = 0.0

        # Elegir tamaño y dirección del paso
        if self.slope is not None and abs(self.slope) > SLOPE_MIN_ABS:
            # Paso calculado: δφ = -PHI/slope llevaría PHI a 0 idealmente.
            # Envolvemos al tamaño mínimo (si el cálculo da muy poco,
            # forzamos un paso al menos igual al paso_buscar).
            delta = -self.phi_fresh / self.slope
            delta = np.sign(delta) * max(abs(delta), self.paso_buscar)
        else:
            # Sin slope confiable: explorar en la dirección actual
            delta = self.dir_buscar * self.paso_buscar

        # Saturar al máximo permitido para evitar saltos bruscos
        delta = clamp(delta, -PASO_BUSCAR_MAX, PASO_BUSCAR_MAX)

        cambio = self._mover_fase(delta)
        return self._resp(f"BUSCAR φ{delta:+.0f}°",
                          FREC_NOMINAL, cambio_fase=cambio, fresh=True)

    # ------------------------------------------------------------------
    def _pll(self, fresh):
        """
        MODO PLL: la fase del FG está CONGELADA. Todo el control se hace
        variando Δf (frecuencia del FG respecto a 60 Hz).

        Control PI clásico sobre el error:
            err  = -PHI / slope     [°]  (equivale a "qué Δf necesitaríamos")
            df_p = Kp · err         [Hz] (proporcional)
            df_i = Ki · ∫err·dt     [Hz] (integral)
            Δf   = df_p + df_i      [Hz]

        El signo de err es CLAVE: dividir por slope (negativo) hace que
        para PHI<0 el err<0 y por tanto Δf<0, lo cual AUMENTA dPHI/dt
        (porque dPHI/dt = slope·360·Δf = negativo·negativo = positivo),
        llevando PHI hacia 0.

        NOTA: el integrador sólo avanza en muestras frescas. Si no, el
        mismo error se acumularía varias veces (una por cada muestra stale
        con el mismo PHI), sobrecorrigiendo.
        """
        if fresh:
            # dt efectivo = tiempo desde última fresca, cap para no sobreintegrar
            eff_dt = min(self.dt_since_fresh, EFF_DT_MAX)
            if eff_dt < 0.05:
                eff_dt = 0.05

            # --- Cálculo del error ---
            s = self.slope if (self.slope is not None
                               and abs(self.slope) > SLOPE_MIN_ABS) else SLOPE_INIT
            # CLAVE: dividir por slope (con signo). Si slope<0 y PHI<0,
            # err<0 → Δf<0 → dPHI/dt>0 → PHI sube hacia 0.
            err = -self.phi_fresh / s

            # --- Término integral ---
            self.df_integral += KI_PLL * err * eff_dt
            self.df_integral = clamp(self.df_integral, -DF_MAX, DF_MAX)

            # --- Término proporcional ---
            df_p = clamp(KP_PLL * err, -DF_MAX, DF_MAX)

            # --- Salida del PI ---
            df_out = df_p + self.df_integral

            # --- Suavizado exponencial (reduce ruido del WT) ---
            self.delta_f = (1 - DF_LP) * self.delta_f + DF_LP * df_out
            self.delta_f = clamp(self.delta_f, -DF_MAX, DF_MAX)

        # Frecuencia final: 60.0 Hz + Δf
        f_fg = FREC_NOMINAL + self.delta_f
        return self._resp(f"PLL df={self.delta_f:+.5f}", f_fg, fresh=fresh)


# ==============================================================================
# LOGGING — encabezado y fila formateada del log en vivo
# ==============================================================================
def encabezado():
    """Imprime el encabezado del log en vivo."""
    print("-" * 160)
    print(f"{'t':>4} | {'FP':>7} | {'PHI':>8} | {'*':>1} | {'Fase':>8} | "
          f"{'Δf':>10} | {'FrecFG':>9} | {'Modo':>7} | {'Slope':>7} | "
          f"{'dfint':>10} | {'err':>8} | {'MejorFP':>7} | {'Acción':>18}")
    print("-" * 160)


def fila(t, fp_med, res, en_rango, df_integral=0.0, err_val=0.0):
    """
    Imprime una línea del log.

    Columnas:
        t        : tiempo desde inicio (s)
        FP       : factor de potencia medido por el WT
        PHI      : ángulo V-I fresco (grados)
        *        : '*'=fresca  '-'=stale  ' '=no aplica
        Fase     : fase actual del FG (grados)
        Δf       : delta de frecuencia aplicado al FG (Hz)
        FrecFG   : frecuencia total del FG (Hz) = 60 + Δf
        Modo     : BUSCAR o PLL
        Slope    : pendiente dPHI/dφ aprendida
        dfint    : acumulador del término integral
        err      : error del PLL (-PHI/slope)
        MejorFP  : mejor FP alcanzado hasta ahora
        Acción   : descripción de lo que hizo el controlador
    """
    mark = "*" if res.get("fresh") else ("-" if res.get("stale") else " ")
    print(f"{t:>4} | {fp_med:>7.4f} | {res['phi']:>+8.2f} | {mark:>1} | "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+10.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>7} | "
          f"{res['slope']:>+7.3f} | {df_integral:>+10.5f} | "
          f"{err_val:>+8.2f} | {res['mejor_fp']:>7.4f} | {res['accion']:>18}")


# ==============================================================================
# MAIN — lazo principal
# ==============================================================================
def main():
    # Cabecera de la aplicación
    print("=" * 160)
    print(" CONTROL FP v9.3 - PLL CON SIGNO CORREGIDO (err = -PHI/slope)  [MODO EXTREME]")
    print("=" * 160)
    print(f" Muestreo objetivo: {1/INTERVALO_MUESTREO:.0f} Hz")
    print(f" Kp = {KP_PLL:.5f}  Ki = {KI_PLL:.6f}  |Δf|max = {DF_MAX:.3f} Hz")
    print(f" BUSCAR entra si |PHI| > {PHI_BUSCAR_ENTER}° (3 frescas); "
          f"sale si |PHI| < {PHI_BUSCAR_EXIT}° (2 frescas)")
    print(f" Slope inicializado en {SLOPE_INIT}. Outliers: saltos > {OUTLIER_JUMP}°")
    print("=" * 160)

    # Referencias a instrumentos y objetos internos
    fg = None
    wt = None
    controlador = ControladorFPv9()
    stats = Estadisticas()

    # Contadores del lazo
    total = 0                   # iteraciones totales
    en_rango = 0                # iteraciones con FP en rango
    t_ctrl = None               # instante de inicio del control

    try:
        # ------------------------------------------------------------------
        # PASO 1: Conectar instrumentos y configurarlos en modo EXTREME
        # ------------------------------------------------------------------
        print("\n[1/3] Conectando equipos (modo EXTREME)...")
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()
        print(f"  FG420:  {fg.obtener_idn()[:60]}")
        print(f"  WT3000: {wt.obtener_idn()[:60]}")

        # Configurar FG420: señal senoidal 5 Vpp, 60 Hz, fase 0°, salida ON
        fg.extreme(
            canal=1, frecuencia_hz=FREC_NOMINAL,
            amplitud_vpp=AMPLITUD_FG, offset_v=OFFSET_V_FG,
            fase_grados=0.0, encender_salida=True,
        )
        # Configurar WT3000: salida numérica mínima (PHI, FU, LAMBda, P, Q),
        # sin filtros, sin promedio (avg=1) → mínima latencia por muestra.
        wt.extreme(
            elemento_entrada=ELEMENTO_WT,
            incluir_potencias=True, configurar_salida=True,
        )
        print("  Modo EXTREME aplicado.")

        # ------------------------------------------------------------------
        # PASO 2: Esperar a que el sistema se estabilice (5 s)
        # ------------------------------------------------------------------
        print("\n[2/3] Estabilizando 5 s...")
        for _ in range(10):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print(" OK")

        # ------------------------------------------------------------------
        # PASO 3: Lazo de control principal
        # ------------------------------------------------------------------
        print("\n[3/3] INICIANDO CONTROL\n")
        encabezado()

        t_ctrl = time.time()        # instante t=0
        t_prev = t_ctrl             # para calcular dt en cada iteración

        # Mientras no se cumpla TIEMPO_PRUEBA_SEG segundos
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev     # tiempo real transcurrido desde la iteración previa
            t_prev = t_now

            # --- Leer mediciones del WT3000 (salida numérica mínima) ---
            try:
                m = wt.leer_mediciones_minimas()
            except Exception as e:
                print(f"[X] Error lectura: {e}")
                time.sleep(INTERVALO_MUESTREO)
                continue

            # Descartar muestras con jitter anómalo (instrumento ocupado)
            if wt.is_outlier():
                continue

            # Extraer los tres valores que necesitamos
            fp_med = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med = m.get("frecuencia")
            if phi_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            # FP absoluto para el rango [0.97, 1.03]
            fp_abs = abs(fp_med) if fp_med is not None \
                else abs(np.cos(np.radians(phi_med)))
            total += 1
            en_rango_local = FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO
            if en_rango_local:
                en_rango += 1

            # --- Paso de control: actualizar el controlador ---
            res = controlador.actualizar(fp_med, phi_med, f_med, dt, stats)

            # --- Aplicar acciones al FG420 ---
            # 1) SIEMPRE escribir la frecuencia (60 + Δf). Es la única forma
            #    de mantener el PLL funcionando, aunque Δf=0.
            fg.establecer_frecuencia_extreme(1, res["frecuencia"])
            # 2) Escribir la fase SÓLO si cambió (para no añadir latencia
            #    innecesaria cuando estamos en modo PLL con fase congelada).
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            # --- Registrar en estadísticas ---
            stats.registrar(fp_abs, res["phi"], res["fase"],
                            res["frecuencia"], res["delta_f"],
                            res["modo"], dt)

            # --- Trazabilidad temporal ---
            t_rel = t_now - t_ctrl
            if stats.tiempo_primer_lock is None and abs(res["phi"]) < PHI_LOCKED:
                stats.tiempo_primer_lock = t_rel
            if stats.tiempo_primer_tight is None and fp_abs >= FP_TIGHT:
                stats.tiempo_primer_tight = t_rel

            # --- Log: sólo cuando hay lectura fresca, o cada 8 muestras ---
            if res.get("fresh") or total % 8 == 0:
                # Calcular err para mostrarlo en el log
                s = controlador.slope if controlador.slope else SLOPE_INIT
                err_val = -res["phi"] / s if abs(s) > SLOPE_MIN_ABS else 0.0
                fila(int(t_rel), fp_abs, res, en_rango_local,
                     controlador.df_integral, err_val)

            # --- Respetar el intervalo de muestreo (si el loop fue rápido) ---
            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        # Al terminar el tiempo, imprimir resumen
        stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)

    except KeyboardInterrupt:
        # El usuario interrumpió con Ctrl-C → imprimir resumen igualmente
        print("\n\n[!] Detenido por usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(controlador, time.time() - t_ctrl, total, en_rango)

    except Exception as e:
        # Cualquier otro error: imprimir traza y salir
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Limpieza: apagar salida del FG y desconectar instrumentos
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


# Punto de entrada
if __name__ == "__main__":
    main()