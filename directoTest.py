"""
CONTROL DE FP v7 - Δf + barrido de fase
========================================

Estrategia:
 - Fase: barrido grueso para encontrar la zona dulce (BUSCAR)
 - Δf:   ajuste fino integrando cambios de fase (AJUSTAR)
 - Δf=0: mantener posición una vez en el óptimo (FIJAR)

La clave: la frecuencia del FG se actualiza en CADA muestra para
evitar deriva de la fase relativa.
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

AMPLITUD_FG = 5.0
OFFSET_V_FG = 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0
FREC_MIN, FREC_MAX = 55.0, 65.0

# Transiciones entre modos
FP_BUSCAR_A_AJUSTAR = 0.965
FP_AJUSTAR_A_FIJAR = 0.995
FP_SALIR_FIJAR = 0.985

# Modo BUSCAR: barrido de fase
PASO_FASE_INICIAL = 20.0
PASO_FASE_MIN = 8.0
N_ESPERA_BUSCAR = 5      # muestras tras mover fase antes de evaluar

# Modo AJUSTAR: control fino por Δf
PASO_DF_INICIAL = 0.020
PASO_DF_MIN = 0.004
N_APLICAR_DF = 3         # muestras aplicando Δf
N_ESPERA_DF = 6          # muestras midiendo con Δf=0

# Umbrales de decisión
UMBRAL_MEJORA = 0.004
UMBRAL_EMPEORA = 0.004
FP_MALO = 0.700
N_MALO_PARA_SCAN = 8     # muestras con FP malo para re-buscar

# Diagnóstico
N_PRUEBAS_EFECTO = 6
UMBRAL_EFECTO = 0.005


def envolver_fase(a):
    return ((a + 180.0) % 360.0) - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==============================================================================
# ESTADÍSTICAS
# ==============================================================================

class Estadisticas:
    def __init__(self):
        self.fp_hist = []
        self.fase_hist = []
        self.frec_hist = []
        self.df_hist = []
        self.tiempos_modo = {"BUSCAR": 0.0, "AJUSTAR": 0.0, "FIJAR": 0.0}
        self.bins_fp = np.zeros(10)
        self.n_transiciones = 0
        self.n_rebusquedas = 0
        self.tiempo_al_mejor = None
        self.efecto_pruebas = 0
        self.efecto_exitosas = 0

    def registrar(self, fp, fase, frec, df, modo, dt):
        self.fp_hist.append(fp)
        self.fase_hist.append(fase)
        self.frec_hist.append(frec)
        self.df_hist.append(df)
        self.tiempos_modo[modo] = self.tiempos_modo.get(modo, 0.0) + dt
        self.bins_fp[min(int(fp * 10), 9)] += 1

    def resumen(self, controlador, t_total, n_iter, en_rango):
        print("\n" + "=" * 90)
        print(" RESUMEN FINAL v7")
        print("=" * 90)

        print(f"\n[ Tiempo y muestras ]")
        print(f"  Tiempo total:            {t_total:.1f} s")
        print(f"  Iteraciones:             {n_iter}")
        print(f"  Tasa efectiva:           {n_iter/max(t_total,1e-6):.1f} muestras/s")
        if self.fp_hist:
            print(f"  FP promedio:             {np.mean(self.fp_hist):.4f}")
            print(f"  FP desviación:           {np.std(self.fp_hist):.4f}")

        if n_iter > 0:
            efectividad = 100 * en_rango / n_iter
            print(f"\n[ Efectividad ]")
            print(f"  En rango (1±3%):         {en_rango} ({efectividad:.1f}%)")

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

        print(f"\n[ Mejor punto ]")
        print(f"  Mejor FP:                {controlador.mejor_fp:.4f}")
        print(f"  Fase en mejor FP:        {controlador.mejor_fase:+.2f}°")
        if self.tiempo_al_mejor is not None:
            print(f"  Alcanzado a los:         {self.tiempo_al_mejor:.1f} s")

        print(f"\n[ Comportamiento ]")
        print(f"  Transiciones de modo:    {self.n_transiciones}")
        print(f"  Re-búsquedas:            {self.n_rebusquedas}")
        print(f"  Cambios de fase totales: {controlador.n_cambios_fase}")
        if self.df_hist:
            print(f"  Δf promedio:             {np.mean(self.df_hist):+.5f} Hz")
            print(f"  Δf máximo abs:           {np.max(np.abs(self.df_hist)):.5f} Hz")

        if self.frec_hist:
            uf = self.frec_hist[-100:] if len(self.frec_hist) >= 100 else self.frec_hist
            print(f"\n[ Frecuencia de red ]")
            print(f"  Promedio (últimas):      {np.mean(uf):.4f} Hz")
            print(f"  Desviación:              {np.std(uf):.5f} Hz")
            print(f"  Rango global:            {np.min(self.frec_hist):.4f} - "
                  f"{np.max(self.frec_hist):.4f} Hz")

        print(f"\n[ Diagnóstico del actuador ]")
        if self.efecto_pruebas >= N_PRUEBAS_EFECTO:
            ratio = self.efecto_exitosas / self.efecto_pruebas
            if ratio > 0.5:
                print(f"  ✓ Actuador efectivo ({self.efecto_exitosas}/"
                      f"{self.efecto_pruebas} pruebas)")
            else:
                print(f"  ✗ Actuador poco efectivo ({self.efecto_exitosas}/"
                      f"{self.efecto_pruebas} pruebas)")
        else:
            print(f"  ? Indeterminado ({self.efecto_pruebas} pruebas)")

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
# CONTROLADOR v7
# ==============================================================================

class ControladorFPv7:
    """
    Máquina de estados: BUSCAR → AJUSTAR → FIJAR.

    - BUSCAR: barrido de fase en pasos de ~15-20°, sin tocar Δf.
    - AJUSTAR: ráfagas de Δf pequeñas (~0.01 Hz durante 0.15 s),
      luego mide con Δf=0.
    - FIJAR: Δf=0, monitoriza. Si FP cae, vuelve a AJUSTAR.
    """
    def __init__(self):
        # Actuadores
        self.fase_cmd = 0.0
        self.delta_f = 0.0

        # Estado de la máquina
        self.modo = "BUSCAR"
        self.fase_estado = "MOVER"
        self.cnt = 0
        self.fp_ref = None

        # Dirección y pasos
        self.signo = +1.0
        self.paso_fase = PASO_FASE_INICIAL
        self.paso_df = PASO_DF_INICIAL

        # Medición
        self.fp_buf = deque(maxlen=4)
        self.fp_suav = None

        # Mejor histórico
        self.mejor_fp = 0.0
        self.mejor_fase = 0.0

        # Contadores
        self.n_cambios_fase = 0
        self.n_cont_malo = 0

    # ------------------------------------------------------------------
    def _resp(self, accion, f_red, cambio_fase=False):
        return {
            "accion": accion,
            "fase": self.fase_cmd,
            "delta_f": self.delta_f,
            "frecuencia": f_red + self.delta_f,
            "cambio_fase": cambio_fase,
            "modo": self.modo,
            "fp": self.fp_suav if self.fp_suav is not None else 0.0,
            "mejor_fp": self.mejor_fp,
            "mejor_fase": self.mejor_fase,
            "paso_fase": self.paso_fase,
            "paso_df": self.paso_df,
            "signo": self.signo,
        }

    # ------------------------------------------------------------------
    def _transicionar(self, nuevo_modo, stats):
        if nuevo_modo != self.modo:
            stats.n_transiciones += 1
            self.modo = nuevo_modo
            self.fase_estado = "MOVER"
            self.cnt = 0
            self.fp_ref = self.fp_suav

    # ------------------------------------------------------------------
    def actualizar(self, fp_med, f_med, stats=None):
        if fp_med is None:
            return self._resp("SIN_FP", f_med or FREC_NOMINAL)

        fp = abs(fp_med)
        self.fp_buf.append(fp)
        self.fp_suav = float(np.mean(self.fp_buf))

        # Mejor histórico
        if self.fp_suav > self.mejor_fp:
            self.mejor_fp = self.fp_suav
            self.mejor_fase = self.fase_cmd

        # Contador de muestras con FP malo (para re-búsqueda)
        if fp < FP_MALO:
            self.n_cont_malo += 1
        else:
            self.n_cont_malo = 0

        # Re-búsqueda forzada
        if stats is not None and self.n_cont_malo >= N_MALO_PARA_SCAN \
                and self.modo != "BUSCAR":
            stats.n_rebusquedas += 1
            self._transicionar("BUSCAR", stats)
            self.paso_fase = PASO_FASE_INICIAL
            self.signo = +1.0

        # Despacho por modo
        if self.modo == "BUSCAR":
            return self._buscar(f_med, stats)
        elif self.modo == "AJUSTAR":
            return self._ajustar(f_med, stats)
        else:
            return self._fijar(f_med, stats)

    # ------------------------------------------------------------------
    def _buscar(self, f_red, stats):
        # ¿Encontramos zona aceptable?
        if self.fp_suav >= FP_BUSCAR_A_AJUSTAR:
            if stats is not None:
                self._transicionar("AJUSTAR", stats)
            else:
                self.modo = "AJUSTAR"
            self.signo = +1.0
            self.paso_df = PASO_DF_INICIAL
            self.delta_f = 0.0
            return self._resp("BUSCAR→AJUSTAR", f_red)

        if self.fase_estado == "MOVER":
            # Dar paso de fase
            self.fase_cmd = envolver_fase(self.fase_cmd + self.signo * self.paso_fase)
            self.fase_cmd = clamp(self.fase_cmd, FASE_MIN, FASE_MAX)
            self.fase_estado = "MEDIR"
            self.cnt = 0
            self.fp_ref = self.fp_suav
            self.n_cambios_fase += 1
            self.delta_f = 0.0
            return self._resp(f"FASE{self.signo*self.paso_fase:+.0f}",
                              f_red, cambio_fase=True)

        # MEDIR
        self.cnt += 1
        if self.cnt >= N_ESPERA_BUSCAR:
            delta = self.fp_suav - (self.fp_ref or 0.0)

            # Registrar prueba de efecto
            if stats is not None:
                stats.efecto_pruebas += 1
                if abs(delta) > UMBRAL_EFECTO:
                    stats.efecto_exitosas += 1

            if delta > UMBRAL_MEJORA:
                pass  # seguir mismo sentido
            elif delta < -UMBRAL_EMPEORA:
                self.signo *= -1
                self.paso_fase = max(self.paso_fase * 0.75, PASO_FASE_MIN)
            else:
                self.paso_fase = max(self.paso_fase * 0.9, PASO_FASE_MIN)

            self.fase_estado = "MOVER"
            self.cnt = 0

        return self._resp("MEDIR_BUSCAR", f_red)

    # ------------------------------------------------------------------
    def _ajustar(self, f_red, stats):
        # ¿Llegamos al óptimo?
        if self.fp_suav >= FP_AJUSTAR_A_FIJAR:
            if stats is not None:
                self._transicionar("FIJAR", stats)
            else:
                self.modo = "FIJAR"
            self.delta_f = 0.0
            return self._resp("AJUSTAR→FIJAR", f_red)

        # ¿Perdimos la zona?
        if self.fp_suav < FP_BUSCAR_A_AJUSTAR * 0.9:
            if stats is not None:
                self._transicionar("BUSCAR", stats)
            else:
                self.modo = "BUSCAR"
            self.paso_fase = max(self.paso_fase, PASO_FASE_MIN)
            self.delta_f = 0.0
            return self._resp("AJUSTAR→BUSCAR", f_red)

        if self.fase_estado == "MOVER":
            # Aplicar Δf durante N muestras
            self.delta_f = self.signo * self.paso_df
            self.cnt += 1
            if self.cnt >= N_APLICAR_DF:
                self.fase_estado = "MEDIR"
                self.delta_f = 0.0
                self.cnt = 0
                self.fp_ref = self.fp_suav
            return self._resp(f"DF{self.delta_f:+.4f}", f_red)

        # MEDIR
        self.cnt += 1
        if self.cnt >= N_ESPERA_DF:
            delta = self.fp_suav - (self.fp_ref or 0.0)

            if stats is not None:
                stats.efecto_pruebas += 1
                if abs(delta) > UMBRAL_EFECTO:
                    stats.efecto_exitosas += 1

            if delta > UMBRAL_MEJORA:
                pass  # seguir en la misma dirección
            elif delta < -UMBRAL_EMPEORA:
                self.signo *= -1
                self.paso_df = max(self.paso_df * 0.7, PASO_DF_MIN)
            else:
                self.paso_df = max(self.paso_df * 0.85, PASO_DF_MIN)

            self.fase_estado = "MOVER"
            self.cnt = 0

        return self._resp("MEDIR_AJUSTAR", f_red)

    # ------------------------------------------------------------------
    def _fijar(self, f_red, stats):
        self.delta_f = 0.0

        if self.fp_suav < FP_SALIR_FIJAR:
            if stats is not None:
                self._transicionar("AJUSTAR", stats)
            else:
                self.modo = "AJUSTAR"
            self.fase_estado = "MOVER"
            self.cnt = 0
            self.fp_ref = self.fp_suav
            return self._resp("FIJAR→AJUSTAR", f_red)

        return self._resp("FIJAR", f_red)


# ==============================================================================
# LOGGING
# ==============================================================================

def encabezado():
    print("-" * 130)
    print(f"{'t':>4} | {'FP':>7} | {'FPflt':>7} | {'Fase':>8} | "
          f"{'Δf':>9} | {'FrecFG':>9} | {'Modo':>8} | "
          f"{'Paso':>7} | {'Mejor':>7} | {'Acción':>18} | Estado")
    print("-" * 130)


def fila(t, fp_med, res, en_rango):
    estado = "OK" if en_rango else "OUT"
    if res["modo"] == "FIJAR":
        estado = "LOCK" if en_rango else "LOCK-OUT"
    if res["modo"] == "BUSCAR":
        estado = "SRCH"
    print(f"{t:>4} | {fp_med:>7.4f} | {res['fp']:>7.4f} | "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+9.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>8} | "
          f"{res['paso_fase']:>7.1f} | {res['mejor_fp']:>7.4f} | "
          f"{res['accion']:>18} | {estado}")


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("=" * 130)
    print(" CONTROL DE FP v7 - Δf COMO ACTUADOR FINO + BARRIDO DE FASE")
    print("=" * 130)
    print(f" Muestreo: {1/INTERVALO_MUESTREO:.0f} Hz")
    print(f" Transiciones: BUSCAR (FP>{FP_BUSCAR_A_AJUSTAR}) → "
          f"AJUSTAR (FP>{FP_AJUSTAR_A_FIJAR}) → FIJAR")
    print("=" * 130)

    fg = None
    wt = None
    controlador = ControladorFPv7()
    stats = Estadisticas()

    total = 0
    en_rango = 0
    t_ctrl = None

    try:
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
                m = wt.leer_mediciones_estandar()
            except Exception as e:
                print(f"[X] Error lectura: {e}")
                time.sleep(INTERVALO_MUESTREO)
                continue

            if wt.is_outlier():
                continue

            fp_med = m.get("factor_potencia")
            f_med = m.get("frecuencia")
            if fp_med is None or f_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med)
            total += 1
            en_rango_local = FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO
            if en_rango_local:
                en_rango += 1

            modo_previo = controlador.modo
            res = controlador.actualizar(fp_med, f_med, stats)

            # === CRÍTICO: SIEMPRE actualizar frecuencia ===
            fg.establecer_frecuencia_streaming(1, res["frecuencia"])

            # Actualizar fase solo si cambió
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            # Estadísticas
            stats.registrar(fp_abs, res["fase"], f_med,
                            res["delta_f"], res["modo"], dt)
            if modo_previo != res["modo"]:
                stats.n_transiciones += 0  # ya se cuenta en el controlador

            # Tiempo al mejor
            if (stats.tiempo_al_mejor is None
                    and controlador.fp_suav is not None
                    and controlador.fp_suav >= controlador.mejor_fp - 0.0001
                    and controlador.mejor_fp > 0.99):
                stats.tiempo_al_mejor = t_now - t_ctrl

            # Log: cada 3 muestras o si hubo cambio de fase
            if total % 3 == 0 or res["cambio_fase"]:
                fila(int(t_now - t_ctrl), fp_abs, res, en_rango_local)

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