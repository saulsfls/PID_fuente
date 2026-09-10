"""
SISTEMA DE CONTROL DE FP - MEJORADO (CONTROL PI ADAPTATIVO DE FASE)
=====================================================================
Optimizado para convergencia rápida a FP = 1.0 con anti-windup,
detección de polaridad y búsqueda de rescate.
"""

import time
import numpy as np
from datetime import datetime

from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN GENERAL
# ==============================================================================

DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"

ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.5

FP_OBJETIVO = 1.0
MARGEN_FP_MIN = 0.970
MARGEN_FP_MAX = 1.030

AMPLITUD_MIN = 2.0
AMPLITUD_MAX = 10.0
AMPLITUD_INICIAL = 5.0

FASE_MIN = -180.0
FASE_MAX = 180.0

# ==============================================================================
# PARÁMETROS OPTIMIZADOS DEL CONTROLADOR
# ==============================================================================

KP_FASE = 0.45            # Ganancia Proporcional base
KI_FASE = 0.08            # Ganancia Integral (elimina error estacionario)
FILTRO_PHI = 0.35         # Filtro más rápido (menor retardo de fase)
DEADBAND_PHI = 0.15       # Umbral fino de zona muerta en grados

# ==============================================================================
# CLASE CONTROLADOR PI ADAPTATIVO
# ==============================================================================

class ControladorFaseAvanzado:
    def __init__(self):
        self.fase_actual = 0.0
        self.amplitud_actual = AMPLITUD_INICIAL
        self.phi_suavizado = 0.0
        self.fp_suavizado = 1.0

        # Término integral
        self.integral_error = 0.0

        # Mejor punto histórico (Rescate)
        self.mejor_fp = 0.0
        self.mejor_fase = 0.0
        self.mejor_amplitud = AMPLITUD_INICIAL

        # Detección de divergencia y saturación
        self.contador_saturado = 0
        self.polaridad = 1.0  # Multiplicador de dirección de corrección

    def actualizar(self, fp_medido, phi_medido, dt):
        # 1. Filtrado dinámico
        if phi_medido is not None and abs(phi_medido) <= 180:
            self.phi_suavizado = (1 - FILTRO_PHI) * self.phi_suavizado + FILTRO_PHI * phi_medido
        if fp_medido is not None and abs(fp_medido) <= 2.0:
            self.fp_suavizado = (1 - FILTRO_PHI) * self.fp_suavizado + FILTRO_PHI * abs(fp_medido)

        # 2. Registrar mejor desempeño
        if self.fp_suavizado > self.mejor_fp and self.fp_suavizado <= 1.05:
            self.mejor_fp = self.fp_suavizado
            self.mejor_fase = self.fase_actual
            self.mejor_amplitud = self.amplitud_actual

        # 3. Lógica de rescate ante saturación o colapso de FP
        if abs(self.fase_actual) >= (FASE_MAX - 1.0) or self.fp_suavizado < 0.35:
            self.contador_saturado += 1
            if self.contador_saturado > 4:
                # Regresar a la mejor fase conocida e invertir polaridad de control
                self.fase_actual = self.mejor_fase
                self.integral_error = 0.0
                self.polaridad *= -1.0
                self.contador_saturado = 0
                return {
                    "fase_fg": self.fase_actual,
                    "amplitud": self.amplitud_actual,
                    "phi_suavizado": self.phi_suavizado,
                    "fp_suavizado": self.fp_suavizado,
                    "error_phi": 0.0,
                    "correccion": 0.0,
                    "cambio": True,
                    "mejor_fp": self.mejor_fp,
                    "mejor_fase": self.mejor_fase,
                    "mejor_amplitud": self.mejor_amplitud,
                    "saturado": True,
                    "accion": "RESCATE_FASE"
                }
        else:
            self.contador_saturado = 0

        # 4. Cálculo del error (Objetivo: phi -> 0)
        error_phi = self.phi_suavizado * self.polaridad

        if abs(error_phi) < DEADBAND_PHI:
            return self._respuesta_sin_cambio()

        # 5. Adaptatividad de paso según la magnitud del problema
        abs_err = abs(error_phi)
        if abs_err > 20.0:
            paso_max = 5.0
            kp_adj = KP_FASE * 1.5
        elif abs_err > 5.0:
            paso_max = 2.0
            kp_adj = KP_FASE
        else:
            paso_max = 0.5
            kp_adj = KP_FASE * 0.7

        # 6. Cálculo PI
        self.integral_error += error_phi * dt
        # Limitar viento del acumulador integral (Anti-windup)
        self.integral_error = max(-20.0, min(20.0, self.integral_error))

        p_term = kp_adj * error_phi
        i_term = KI_FASE * self.integral_error

        correccion = p_term + i_term
        correccion = max(-paso_max, min(paso_max, correccion))

        nueva_fase = self.fase_actual - correccion
        nueva_fase = max(FASE_MIN, min(FASE_MAX, nueva_fase))

        if abs(nueva_fase - self.fase_actual) < 0.02:
            return self._respuesta_sin_cambio()

        self.fase_actual = nueva_fase

        return {
            "fase_fg": self.fase_actual,
            "amplitud": self.amplitud_actual,
            "phi_suavizado": self.phi_suavizado,
            "fp_suavizado": self.fp_suavizado,
            "error_phi": error_phi,
            "correccion": correccion,
            "cambio": True,
            "mejor_fp": self.mejor_fp,
            "mejor_fase": self.mejor_fase,
            "mejor_amplitud": self.mejor_amplitud,
            "saturado": False,
            "accion": f"FASE_{correccion:+.2f}"
        }

    def _respuesta_sin_cambio(self):
        return {
            "fase_fg": self.fase_actual,
            "amplitud": self.amplitud_actual,
            "phi_suavizado": self.phi_suavizado,
            "fp_suavizado": self.fp_suavizado,
            "error_phi": 0,
            "correccion": 0,
            "cambio": False,
            "mejor_fp": self.mejor_fp,
            "mejor_fase": self.mejor_fase,
            "mejor_amplitud": self.mejor_amplitud,
            "saturado": False,
            "accion": "ESTABLE"
        }

# ==============================================================================
# FUNCIÓN DE ESTADÍSTICAS
# ==============================================================================

def mostrar_resumen(controlador, metodo_directo, total_iteraciones, iteraciones_en_rango,
                    fp_historico, tiempo_total=None):
    efectividad = (iteraciones_en_rango / total_iteraciones * 100) if total_iteraciones > 0 else 0

    print("\n" + "=" * 90)
    print(" RESUMEN FINAL DE PRUEBA")
    print("=" * 90)
    if tiempo_total is not None:
        print(f" Tiempo total de prueba:      {tiempo_total:.1f} s")
    print(f" Total iteraciones:           {total_iteraciones}")
    print(f" En rango (1±3%):             {iteraciones_en_rango}")
    print(f" Efectividad:                 {efectividad:.2f}%")
    if fp_historico:
        ultimas = fp_historico[-50:] if len(fp_historico) >= 50 else fp_historico
        print(f" FP promedio (últimas 50):    {np.mean(ultimas):.4f}")
        print(f" Desviación estándar FP:      {np.std(ultimas):.4f}")
        print(f" FP mínimo:                   {np.min(ultimas):.4f}")
        print(f" FP máximo:                   {np.max(ultimas):.4f}")
    if not metodo_directo and controlador is not None:
        print(f" Mejor FP registrado:         {controlador.mejor_fp:.4f}")
        print(f" Fase FG en mejor FP:         {controlador.mejor_fase:.1f}°")
        print(f" Fase FG final:               {controlador.fase_actual:.1f}°")
    print("=" * 90)

# ==============================================================================
# PROGRAMA PRINCIPAL
# ==============================================================================

def main():
    print("\n" + "="*90)
    print(" SELECCIÓN DEL MÉTODO DE CONTROL DE FP")
    print("="*90)
    print(" 1. DIRECTO    -> Sincronización libre sin lazo de control")
    print(" 2. CONTROLADO -> Algoritmo PI Adaptativo con corrección de fase")
    print("="*90)
    opcion = input("Opción (1 o 2): ").strip()
    while opcion not in ('1', '2'):
        opcion = input("Opción inválida (1 o 2): ").strip()
    metodo_directo = (opcion == '1')

    fg = None
    wt = None

    try:
        fg = YokogawaFG420(DIR_FG, mode='fast')
        wt = YokogawaWT3000(DIR_WT, mode='balanced')
        controlador = ControladorFaseAvanzado() if not metodo_directo else None

        total_iteraciones = 0
        iteraciones_en_rango = 0
        fp_historico = []

        print("\n[1/3] Conectando equipos...")
        wt.conectar()
        wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)

        fg.conectar()
        fg.configurar_canal_rapido(
            canal=1,
            funcion="SIN",
            frecuencia_hz=60.0,
            amplitud_vpp=AMPLITUD_INICIAL,
            offset_v=0.0,
            fase_grados=0.0,
            activar_salida=True
        )
        fg.frecuencia_actual = 60.0

        print("\n[2/3] Estabilizando lecturas...")
        time.sleep(2.0)

        print("\n[3/3] INICIANDO CONTROL")
        print("-" * 90)
        print(f"{'Tiempo':<8} | {'FP':<8} | {'FP Filt':<8} | {'Fase FG':<10} | {'Phi Med':<8} | {'Mejor FP':<8} | {'Acción':<12} | {'Estado':<8}")
        print("-" * 90)

        tiempo_anterior = time.time()
        tiempo_control = time.time()

        while True:
            t_actual = time.time()

            if t_actual - tiempo_control > TIEMPO_PRUEBA_SEG:
                break

            dt = t_actual - tiempo_anterior
            tiempo_anterior = t_actual

            m = wt.leer_mediciones_estandar()
            fp_medido = m.get("factor_potencia")
            phi_medido = m.get("angulo_fase")
            frec_medida = m.get("frecuencia", 60.0)

            if fp_medido is None or phi_medido is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_medido)

            if metodo_directo:
                nueva_frec = max(59.0, min(61.0, frec_medida))
                if abs(nueva_frec - getattr(fg, 'frecuencia_actual', 60.0)) > 0.001:
                    fg.establecer_frecuencia(1, nueva_frec)
                    fg.frecuencia_actual = nueva_frec

                en_rango = MARGEN_FP_MIN <= fp_abs <= MARGEN_FP_MAX
                if en_rango:
                    iteraciones_en_rango += 1
                total_iteraciones += 1
                fp_historico.append(fp_abs)

                segundos = int(time.time() - tiempo_control)
                print(f"{segundos:03d}s     | {fp_abs:<8.4f} | {'N/A':<8} | {'0.0°':<10} | {phi_medido:<+8.1f} | {'N/A':<8} | {'DIRECTO':<12} | {('✓ OK' if en_rango else '✗ OUT'):<8}")

            else:
                res = controlador.actualizar(fp_medido, phi_medido, dt)

                if res["cambio"]:
                    fg.establecer_fase(1, res["fase_fg"])

                en_rango = MARGEN_FP_MIN <= fp_abs <= MARGEN_FP_MAX
                if en_rango:
                    iteraciones_en_rango += 1
                total_iteraciones += 1
                fp_historico.append(fp_abs)

                segundos = int(time.time() - tiempo_control)
                print(f"{segundos:03d}s     | {fp_abs:<8.4f} | {res['fp_suavizado']:<8.4f} | {res['fase_fg']:<+10.1f} | {res['phi_suavizado']:<+8.1f} | {res['mejor_fp']:<8.4f} | {res['accion']:<12} | {('✓ OK' if en_rango else '✗ OUT'):<8}")

            time.sleep(max(0, INTERVALO_MUESTREO - (time.time() - t_actual)))

        mostrar_resumen(controlador, metodo_directo, total_iteraciones, iteraciones_en_rango, fp_historico, time.time() - tiempo_control)

    except KeyboardInterrupt:
        print("\n\n[!] Proceso detenido por el usuario.")
        if 'tiempo_control' in locals():
            mostrar_resumen(controlador, metodo_directo, total_iteraciones, iteraciones_en_rango, fp_historico, time.time() - tiempo_control)

    finally:
        print("\n[!] Apagando salidas y cerrando comunicación...")
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
        print("[✓] Conexiones finalizadas.")

if __name__ == "__main__":
    main()