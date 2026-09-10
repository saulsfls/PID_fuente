"""
SISTEMA DE CONTROL - MODO DIRECTO (CON RECUPERACIÓN DE ERRORES)
=================================================================
Estrategia: Seguimiento directo de la frecuencia de la red.
Incluye reintentos en caso de fallo de comunicación con el WT3000.
"""

import time
import numpy as np
from datetime import datetime
import pyvisa

from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

DIR_FG = "GPIB1::1::INSTR"
DIR_WT = "GPIB0::2::INSTR"

ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300          # Duración de la prueba (segundos)
INTERVALO_MUESTREO = 0.5          # Tiempo entre lecturas

MARGEN_FP_MIN = 0.970
MARGEN_FP_MAX = 1.030

AMPLITUD_INICIAL = 5.0            # Amplitud fija (Vpp)

FREC_MIN = 59.0
FREC_MAX = 61.0

# Número máximo de reintentos para cada lectura del WT3000
MAX_REINTENTOS = 3
RETARDO_REINTENTO = 0.5           # segundos entre reintentos

# ==============================================================================
# FUNCIÓN PARA LEER MEDICIONES CON REINTENTOS
# ==============================================================================

def leer_con_reintentos(wt, max_intentos=MAX_REINTENTOS):
    """
    Intenta leer las mediciones del WT3000.
    Si falla, limpia errores y reintenta hasta max_intentos.
    Retorna el diccionario con las mediciones o None si falla.
    """
    for intento in range(max_intentos):
        try:
            # Intentar leer
            mediciones = wt.leer_mediciones_estandar()
            # Si la lectura es exitosa, retornar
            return mediciones
        except pyvisa.VisaIOError as e:
            print(f"  [WT3000] Error de comunicación (intento {intento+1}/{max_intentos}): {e}")
            if intento == max_intentos - 1:
                # Último intento fallido
                raise
            # Limpiar errores del instrumento y esperar
            try:
                wt.limpiar_errores()
                # También podemos leer el error para vaciar la cola
                error = wt.consultar(":SYSTem:ERRor?")
                print(f"    Error del WT3000: {error}")
            except Exception:
                pass
            time.sleep(RETARDO_REINTENTO)
    return None

# ==============================================================================
# FUNCIÓN PARA MOSTRAR ESTADÍSTICAS
# ==============================================================================

def mostrar_resumen(total_iteraciones, iteraciones_en_rango, fp_historico, tiempo_total):
    efectividad = (iteraciones_en_rango / total_iteraciones * 100) if total_iteraciones > 0 else 0

    print("\n" + "=" * 90)
    print(" RESUMEN FINAL")
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
    print("=" * 90)

# ==============================================================================
# FUNCIÓN PRINCIPAL
# ==============================================================================

def main():
    print("\n" + "=" * 90)
    print(" SISTEMA DE CONTROL - MODO DIRECTO (CON RECUPERACIÓN)")
    print(" =====================================================")
    print(" Estrategia: Seguimiento directo de la frecuencia de la red.")
    print(" Incluye reintentos automáticos en caso de fallo de comunicación.")
    print("=" * 90)

    # ---------- Instanciar controladores con timeout más largo ----------
    # Usamos timeout de 10000 ms para el WT3000 (más tolerante)
    fg = YokogawaFG420(DIR_FG, mode='fast')
    wt = YokogawaWT3000(DIR_WT, mode='balanced', timeout=10000)

    # Variables de estadísticas
    total_iteraciones = 0
    iteraciones_en_rango = 0
    fp_historico = []

    print(f"\n Configuración:")
    print(f"   - Modo FG: {fg.mode.upper()}")
    print(f"   - Modo WT: {wt.mode.upper()}")
    print(f"   - Timeout WT: 10000 ms")
    print(f"   - Amplitud fija: {AMPLITUD_INICIAL:.2f} Vpp")
    print(f"   - Intervalo de muestreo: {INTERVALO_MUESTREO} s")
    print(f"   - Reintentos máximos por lectura: {MAX_REINTENTOS}")

    try:
        # ---------- Conexión y configuración inicial ----------
        print("\n[1/3] Conectando al WT3000...")
        wt.conectar()
        wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)
        print("  ✓ WT3000 conectado")

        print("\n[2/3] Conectando al FG420...")
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
        print(f"  ✓ FG420 configurado: 60.000 Hz, {AMPLITUD_INICIAL:.2f}V, salida ON")

        # Variable para evitar escrituras innecesarias
        frecuencia_fg_actual = 60.0

        print("\n[3/3] Esperando estabilización inicial...")
        for i in range(5):
            time.sleep(0.5)
            print("  .", end="", flush=True)
        print(" ✓")

        print("\n[INICIO] Monitoreando frecuencia de la red...")
        print("=" * 90)
        print(f"{'Tiempo':<12} | {'FP':<10} | {'Frec Red':<12} | {'Frec FG':<12} | {'Estado':<10}")
        print("-" * 90)

        tiempo_inicio = time.time()
        tiempo_control = None
        primer_lectura = False

        # ---------- Bucle principal ----------
        while True:
            if tiempo_control is not None:
                if time.time() - tiempo_control > TIEMPO_PRUEBA_SEG:
                    break

            t_actual = time.time()

            # Leer mediciones del WT3000 con reintentos
            try:
                m = leer_con_reintentos(wt)
                if m is None:
                    # Si no se pudo leer después de reintentos, esperar y continuar
                    print("  [WT3000] No se pudo obtener medición. Se omite este ciclo.")
                    time.sleep(INTERVALO_MUESTREO)
                    continue
            except pyvisa.VisaIOError as e:
                # Error persistente, reiniciar la comunicación
                print(f"  [WT3000] Error persistente: {e}. Intentando reiniciar conexión...")
                try:
                    wt.desconectar()
                    time.sleep(1)
                    wt.conectar()
                    wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)
                    print("  [WT3000] Conexión restablecida.")
                except Exception as recon:
                    print(f"  [WT3000] Fallo al reconectar: {recon}. Se omite este ciclo.")
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_medido = m.get("factor_potencia")
            frec_medida = m.get("frecuencia")

            if fp_medido is None or frec_medida is None:
                print(f"{datetime.now().strftime('%H:%M:%S')} | {'---':<10} | {'---':<12} | {'COM_ERR':<10}")
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_medido)

            if not primer_lectura:
                primer_lectura = True
                tiempo_control = t_actual
                print(f"\n[!] Señal detectada: FP={fp_abs:.4f}, Freq={frec_medida:.3f} Hz\n")
                print(f"{'Tiempo':<12} | {'FP':<10} | {'Frec Red':<12} | {'Frec FG':<12} | {'Estado':<10}")
                print("-" * 90)

            # ---------- Modo directo ----------
            nueva_frec = frec_medida
            nueva_frec = max(FREC_MIN, min(FREC_MAX, nueva_frec))

            if abs(nueva_frec - frecuencia_fg_actual) > 0.001:
                try:
                    fg.establecer_frecuencia(1, nueva_frec)
                    frecuencia_fg_actual = nueva_frec
                except Exception as e:
                    print(f"  [FG420] Error al actualizar frecuencia: {e}")

            # Estadísticas
            en_rango = MARGEN_FP_MIN <= fp_abs <= MARGEN_FP_MAX
            if en_rango:
                iteraciones_en_rango += 1
            total_iteraciones += 1
            fp_historico.append(fp_abs)

            segundos = int(time.time() - tiempo_control)
            timer_display = f"{segundos:03d}s"
            str_3pct = "✓ RANGO" if en_rango else "✗ FUERA"

            print(f"{timer_display:<12} | {fp_abs:<10.4f} | {frec_medida:<12.4f} | {nueva_frec:<12.4f} | {str_3pct:<10}")

            # Esperar hasta el siguiente intervalo (con un pequeño margen)
            tiempo_espera = max(0, INTERVALO_MUESTREO - (time.time() - t_actual))
            time.sleep(tiempo_espera)

        # Si terminó normalmente
        tiempo_total = time.time() - tiempo_control if tiempo_control is not None else 0
        mostrar_resumen(total_iteraciones, iteraciones_en_rango, fp_historico, tiempo_total)

    except KeyboardInterrupt:
        print("\n\n[!] Prueba cancelada por el usuario.")
        if tiempo_control is not None:
            tiempo_transcurrido = time.time() - tiempo_control
        else:
            tiempo_transcurrido = 0
        mostrar_resumen(total_iteraciones, iteraciones_en_rango, fp_historico, tiempo_transcurrido)

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()
        if total_iteraciones > 0:
            mostrar_resumen(total_iteraciones, iteraciones_en_rango, fp_historico, None)

    finally:
        print("\n[!] Cerrando conexiones...")
        if fg:
            try:
                fg.establecer_salida(1, False)
                time.sleep(0.1)
                fg.desconectar()
                print("  ✓ FG420 desconectado y salida OFF")
            except Exception as e:
                print(f"  ✗ Error al cerrar FG420: {e}")
        if wt:
            try:
                wt.desconectar()
                print("  ✓ WT3000 desconectado")
            except Exception as e:
                print(f"  ✗ Error al desconectar WT3000: {e}")
        print("[✓] Conexiones cerradas")
        print("=" * 90)

if __name__ == "__main__":
    main()