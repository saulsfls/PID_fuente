"""
CONTROL DE FP v4 - HILL CLIMBING SOBRE FP
=================================================================
 - No usa angulo_fase (el WT3000 no lo entrega)
 - Búsqueda por pasos con inversión de dirección
 - Reset del filtro tras cada cambio de fase
 - Bloqueo al alcanzar el óptimo
"""

import time
import numpy as np

from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"

ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.5

FP_MIN_RANGO = 0.970
FP_MAX_RANGO = 1.030

AMPLITUD_INICIAL = 5.0
FASE_MIN = -180.0
FASE_MAX = 180.0

FREC_NOMINAL = 60.0
FREC_MIN = 55.0
FREC_MAX = 65.0

# Sincronización de frecuencia
FILTRO_FREC = 0.30
DEADBAND_FREC = 0.003
GANANCIA_FREC = 0.60

# ==============================================================================
# PARÁMETROS DEL HILL CLIMBING
# ==============================================================================

N_SETTLE = 4               # muestras a esperar tras cada cambio de fase
PASO_INICIAL = 15.0        # ° del primer paso
PASO_MIN = 0.5             # ° paso mínimo (parar si llegamos aquí)
PASO_MAX = 30.0
FACTOR_REDUCIR = 0.65      # al invertir o no mejorar
UMBRAL_MEJORA = 0.008      # ΔFP mínimo para considerar "mejoró"
UMBRAL_EMPEORA = 0.008     # ΔFP mínimo para considerar "empeoró"

# Bloqueo
FP_OPTIMO = 0.995          # consideramos óptimo si FP > esto
N_OPTIMO = 6               # muestras consecutivas para bloquear
FP_DESBLOQUEO = 0.985      # si cae por debajo, desbloquear

# ==============================================================================
# UTILIDADES
# ==============================================================================

def envolver_fase(ang):
    return ((ang + 180.0) % 360.0) - 180.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ==============================================================================
# CONTROLADOR v4
# ==============================================================================

class ControladorFP:
    def __init__(self):
        # Actuador
        self.fase_actual = 0.0
        self.frec_actual = FREC_NOMINAL
        self.frec_medida_filtrada = FREC_NOMINAL

        # Medición
        self.fp_suavizado = None

        # Mejor histórico
        self.mejor_fp = 0.0
        self.mejor_fase = 0.0
        self.mejor_frec = FREC_NOMINAL

        # Estado hill-climbing
        self.direccion = +1.0
        self.paso = PASO_INICIAL
        self.fase_anterior = None
        self.fp_en_fase_anterior = None
        self.contador_settle = 0

        # Bloqueo
        self.bloqueado = False
        self.contador_optimo = 0

        # Diagnóstico
        self.n_cambios_fase = 0
        self.n_cambios_sin_efecto = 0
        self.ultimo_delta_fp = 0.0
        self.alerta_sin_efecto = False

    # ------------------------------------------------------------------
    def actualizar_frecuencia(self, frec_medida):
        if frec_medida is None or not (FREC_MIN <= frec_medida <= FREC_MAX):
            return False
        self.frec_medida_filtrada = (
            (1.0 - FILTRO_FREC) * self.frec_medida_filtrada
            + FILTRO_FREC * frec_medida
        )
        error = self.frec_medida_filtrada - self.frec_actual
        if abs(error) < DEADBAND_FREC:
            return False
        nueva = clamp(self.frec_actual + GANANCIA_FREC * error, FREC_MIN, FREC_MAX)
        if abs(nueva - self.frec_actual) < 1e-4:
            return False
        self.frec_actual = nueva
        return True

    # ------------------------------------------------------------------
    def _respuesta(self, cambio_fase=False, accion="ESTABLE"):
        return {
            "cambio_fase": cambio_fase,
            "fase_fg": self.fase_actual,
            "frec_fg": self.frec_actual,
            "fp_suavizado": self.fp_suavizado if self.fp_suavizado is not None else 0.0,
            "mejor_fp": self.mejor_fp,
            "mejor_fase": self.mejor_fase,
            "bloqueado": self.bloqueado,
            "direccion": self.direccion,
            "paso": self.paso,
            "delta_fp": self.ultimo_delta_fp,
            "alerta": self.alerta_sin_efecto,
            "accion": accion,
        }

    # ------------------------------------------------------------------
    def _filtrar_fp(self, fp_real):
        if fp_real is None:
            return
        if self.fp_suavizado is None:
            self.fp_suavizado = fp_real
        else:
            # Media exponencial suave dentro de cada settle
            self.fp_suavizado = 0.6 * self.fp_suavizado + 0.4 * fp_real

    # ------------------------------------------------------------------
    def actualizar(self, fp_medido):
        fp_real = abs(fp_medido) if fp_medido is not None else None

        self._filtrar_fp(fp_real)

        # Mejor histórico
        if fp_real is not None and fp_real > self.mejor_fp + 0.0005:
            self.mejor_fp = fp_real
            self.mejor_fase = self.fase_actual
            self.mejor_frec = self.frec_actual

        # Bloqueo
        if fp_real is not None and fp_real > FP_OPTIMO:
            self.contador_optimo += 1
        else:
            self.contador_optimo = 0
        if self.contador_optimo >= N_OPTIMO:
            self.bloqueado = True
        if self.bloqueado and fp_real is not None and fp_real < FP_DESBLOQUEO:
            self.bloqueado = False
            self.contador_optimo = 0

        if self.bloqueado:
            return self._respuesta(accion="BLOQUEADO")

        if self.fp_suavizado is None:
            return self._respuesta(accion="SIN_FP")

        # --------------------------------------------------------------
        # Arranque: primera medición
        # --------------------------------------------------------------
        if self.fase_anterior is None:
            self.contador_settle += 1
            if self.contador_settle < N_SETTLE:
                return self._respuesta(accion="INIT")
            # Fijar referencia y dar el primer paso
            self.fase_anterior = self.fase_actual
            self.fp_en_fase_anterior = self.fp_suavizado
            self.fase_actual = envolver_fase(self.fase_actual + self.direccion * self.paso)
            self.fp_suavizado = None
            self.contador_settle = 0
            self.n_cambios_fase += 1
            return self._respuesta(cambio_fase=True,
                                   accion=f"INICIO{self.direccion*self.paso:+.0f}")

        # --------------------------------------------------------------
        # Esperando a que asiente el FP tras un cambio
        # --------------------------------------------------------------
        self.contador_settle += 1
        if self.contador_settle < N_SETTLE:
            return self._respuesta(accion="SETTLE")

        # --------------------------------------------------------------
        # Evaluar el paso que dimos
        # --------------------------------------------------------------
        delta = self.fp_suavizado - self.fp_en_fase_anterior
        self.ultimo_delta_fp = delta

        if delta > UMBRAL_MEJORA:
            # Mejoró: seguir en la misma dirección
            pass
        elif delta < -UMBRAL_EMPEORA:
            # Empeoró: invertir y reducir paso
            self.direccion *= -1.0
            self.paso = max(self.paso * FACTOR_REDUCIR, PASO_MIN)
        else:
            # Neutro: reducir paso y probar de nuevo (misma dirección)
            self.paso = max(self.paso * 0.85, PASO_MIN)

        # Detección de "sin efecto" (hardware no responde)
        if abs(delta) < 0.002 and self.n_cambios_fase >= 4:
            self.n_cambios_sin_efecto += 1
            if self.n_cambios_sin_efecto >= 3:
                self.alerta_sin_efecto = True
        else:
            self.n_cambios_sin_efecto = 0

        # --------------------------------------------------------------
        # ¿Estamos suficientemente cerca del óptimo?
        # --------------------------------------------------------------
        if abs(1.0 - self.fp_suavizado) < (1.0 - FP_OPTIMO):
            # Nos quedamos donde estamos
            return self._respuesta(accion="CERCA_OPTIMO")

        # --------------------------------------------------------------
        # Dar el siguiente paso
        # --------------------------------------------------------------
        self.fase_anterior = self.fase_actual
        self.fp_en_fase_anterior = self.fp_suavizado
        self.fase_actual = envolver_fase(
            clamp(self.fase_actual + self.direccion * self.paso, FASE_MIN, FASE_MAX)
        )
        self.fp_suavizado = None
        self.contador_settle = 0
        self.n_cambios_fase += 1

        signo = "+" if self.direccion > 0 else "-"
        return self._respuesta(cambio_fase=True,
                               accion=f"PASO{signo}{self.paso:.1f}")

# ==============================================================================
# RESUMEN
# ==============================================================================

def mostrar_resumen(controlador, total, en_rango, fp_hist, frec_hist, t_total):
    efectividad = (en_rango / total * 100) if total > 0 else 0
    print("\n" + "=" * 90)
    print(" RESUMEN FINAL")
    print("=" * 90)
    print(f" Tiempo total:                {t_total:.1f} s")
    print(f" Iteraciones:                 {total}")
    print(f" En rango (1±3%):             {en_rango}  ({efectividad:.1f}%)")
    if fp_hist:
        ult = fp_hist[-50:] if len(fp_hist) >= 50 else fp_hist
        print(f" FP promedio (últimas 50):    {np.mean(ult):.4f}")
        print(f" Desv. estándar FP:           {np.std(ult):.4f}")
        print(f" FP mín / máx:                {np.min(ult):.4f} / {np.max(ult):.4f}")
    if frec_hist:
        uf = frec_hist[-50:] if len(frec_hist) >= 50 else frec_hist
        print(f" Frec prom / std:             {np.mean(uf):.4f} / {np.std(uf):.5f} Hz")
    if controlador is not None:
        print(f" Mejor FP:                    {controlador.mejor_fp:.4f}")
        print(f" Fase FG en mejor:            {controlador.mejor_fase:+.1f}°")
        print(f" Fase FG final:               {controlador.fase_actual:+.1f}°")
        print(f" Dirección final:             {'+' if controlador.direccion>0 else '-'}")
        print(f" Paso final:                  {controlador.paso:.2f}°")
        print(f" Cambios de fase dados:       {controlador.n_cambios_fase}")
        if controlador.alerta_sin_efecto:
            print(f" *** ALERTA: fase del FG no parece afectar FP ***")
    print("=" * 90)

# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print("\n" + "="*90)
    print(" CONTROL DE FP v4 - HILL CLIMBING SOBRE FP")
    print("="*90)
    print(" (No requiere el ángulo del WT3000)")
    opcion = input(" 1 DIRECTO   |  2 HILL CLIMBING  -> ").strip()
    while opcion not in ('1', '2'):
        opcion = input("Opción (1 o 2): ").strip()
    metodo_directo = (opcion == '1')

    fg = None
    wt = None

    try:
        fg = YokogawaFG420(DIR_FG, mode='fast')
        wt = YokogawaWT3000(DIR_WT, mode='balanced')
        controlador = ControladorFP()

        total = en_rango = 0
        fp_hist = []
        frec_hist = []

        print("\n[1/3] Conectando equipos...")
        wt.conectar()
        wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)

        fg.conectar()
        fg.configurar_canal_rapido(
            canal=1, funcion="SIN",
            frecuencia_hz=FREC_NOMINAL,
            amplitud_vpp=AMPLITUD_INICIAL,
            offset_v=0.0, fase_grados=0.0,
            activar_salida=True,
        )
        fg.frecuencia_actual = FREC_NOMINAL

        print("\n[2/3] Estabilizando 3 s...")
        time.sleep(3.0)

        print("\n[3/3] INICIANDO CONTROL")
        print("-" * 120)
        print(f"{'t':<5}| {'FP':<8}| {'FPflt':<8}| {'FaseFG':<8}| {'FrecFG':<9}| "
              f"{'Mejor':<8}| {'Dir':<4}| {'Paso':<7}| {'ΔFP':<8}| "
              f"{'Acción':<14}| Estado")
        print("-" * 120)

        t_prev = t_ctrl = time.time()

        while True:
            t_now = time.time()
            if t_now - t_ctrl > TIEMPO_PRUEBA_SEG:
                break
            dt = t_now - t_prev
            t_prev = t_now

            m = wt.leer_mediciones_estandar()
            fp_med = m.get("factor_potencia")
            frec_med = m.get("frecuencia", FREC_NOMINAL)

            if fp_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med)
            frec_hist.append(frec_med)

            if controlador.actualizar_frecuencia(frec_med):
                fg.establecer_frecuencia(1, controlador.frec_actual)

            if metodo_directo:
                en_rango += 1 if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO else 0
                total += 1
                fp_hist.append(fp_abs)
                s = int(time.time() - t_ctrl)
                print(f"{s:<5}| {fp_abs:<8.4f}| {'N/A':<8}| "
                      f"{controlador.fase_actual:<+8.1f}| "
                      f"{controlador.frec_actual:<9.4f}| {'N/A':<8}| "
                      f"{'--':<4}| {'--':<7}| {'--':<8}| "
                      f"{'DIRECTO':<14}| "
                      f"{'OK' if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO else 'OUT'}")
            else:
                res = controlador.actualizar(fp_med)
                if res["cambio_fase"]:
                    fg.establecer_fase(1, res["fase_fg"])

                en_rango += 1 if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO else 0
                total += 1
                fp_hist.append(fp_abs)

                s = int(time.time() - t_ctrl)
                estado = "BLOQ" if res["bloqueado"] else (
                    "OK" if FP_MIN_RANGO <= fp_abs <= FP_MAX_RANGO else "OUT")
                if res["alerta"]:
                    estado = "ALERTA"
                print(f"{s:<5}| {fp_abs:<8.4f}| {res['fp_suavizado']:<8.4f}| "
                      f"{res['fase_fg']:<+8.1f}| {res['frec_fg']:<9.4f}| "
                      f"{res['mejor_fp']:<8.4f}| "
                      f"{('+' if res['direccion']>0 else '-'):<4}| "
                      f"{res['paso']:<7.2f}| {res['delta_fp']:<+8.4f}| "
                      f"{res['accion']:<14}| {estado}")

            time.sleep(max(0, INTERVALO_MUESTREO - (time.time() - t_now)))

        mostrar_resumen(controlador, total, en_rango, fp_hist, frec_hist,
                        time.time() - t_ctrl)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido.")
        if 'controlador' in locals():
            mostrar_resumen(controlador, total, en_rango, fp_hist, frec_hist,
                            time.time() - t_ctrl if 't_ctrl' in locals() else 0)

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