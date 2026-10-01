import time
from controllers.wt3000controllerv2 import YokogawaWT3000  # Asegúrate de guardar el controlador WT3000 en este nombre de archivo
from controllers.fg420controllerv2 import YokogawaFG420    # Y el del generador FG420 aquí

def ejecutar_test_control_fp():
    # Direcciones GPIB predeterminadas (ajusta según tu hardware real)
    addr_wt3000 = "GPIB0::1::INSTR"
    addr_fg420  = "GPIB1::2::INSTR"

    print("=== INICIALIZANDO INSTRUMENTOS ===")
    wt = YokogawaWT3000(resource_address=addr_wt3000, mode='extreme')
    fg = YokogawaFG420(resource_address=addr_fg420, mode='extreme')

    try:
        # Conexión
        print(f"Conectando a WT3000: {wt.conectar()}")
        print(f"Conectando a FG420: {fg.conectar()}")

        # Configurar instrumentos para el ensayo
        # El WT3000 medirá dos elementos:
        # Elemento 1: Tensión de red (la más variable)
        # Elemento 2: Señal de corriente proveniente del generador FG420
        wt.extreme(elementos=[1, 2])
        
        # Arrancar FG420 en Canal 1 a 60 Hz, 5 Vpp, 0V offset y 0 grados de fase
        print("\nArrancando FG420 en Canal 1 (60 Hz, 0°, 5 Vpp)...")
        fg.extreme(canal=1, frecuencia_hz=60.0, amplitud_vpp=5.0, offset_v=0.0, fase_grados=0.0, encender_salida=True)
        
        time.sleep(1.0) # Estabilización inicial

        print("\n=== INICIANDO LAZO DE CONTROL Y ESTADÍSTICAS DE FACTOR DE POTENCIA ===")
        print("Objetivo: Factor de Potencia = 1.0 (Tolerancia estricta: dentro del 3% -> >= 0.97)")
        print("Presiona Ctrl+C para detener el test.\n")

        # Parámetros del lazo de control adaptativo (Ajuste de fase basado en error)
        fase_actual = 0.0
        ganancia_Kp = 15.0  # Ganancia proporcional para corregir la fase en función del FP
        target_fp = 1.0
        
        iteracion = 0
        max_iteraciones = 300  # Puedes ajustar o cambiar a un while True:

        while iteracion < max_iteraciones:
            iteracion += 1
            t_inicio = time.perf_counter()

            # Leer mediciones multielemento desde el WT3000
            mediciones = wt.leer_mediciones_multielemento()

            # Tomamos como referencia el Elemento 2 (Generador / Corriente vs Red) o el que corresponda
            # Analizamos por ejemplo el elemento 2 vinculado al generador
            datos_gen = mediciones.get("elemento_2", {})
            fp_actual = datos_gen.get("factor_potencia")

            if fp_actual is not None:
                # Calcular error respecto al objetivo (1.0)
                error_fp = target_fp - fp_actual

                # Lógica de corrección proporcional: si el FP baja de 0.97, ajustamos la fase del FG420
                if fp_actual < 0.97:
                    # Corrección de fase en grados orientada a mitigar el desfase de la red
                    fase_actual += ganancia_Kp * error_fp
                    fg.establecer_fase(canal=1, fase_grados=fase_actual)

            # Obtener estadísticas acumuladas del factor de potencia
            stats_fp = wt.obtener_estadisticas_fp(elemento=2)
            
            print(f"[{iteracion:03d}] FP Actual: {fp_actual if fp_actual is not None else 0.0:.4f} | "
                  f"Fase Ajustada: {fase_actual:.2f}° | "
                  f"Estabilidad en Rango: {stats_fp['en_rango_pct']:.1f}% "
                  f"(Media FP: {stats_fp['mean_fp']:.4f})")

            # Control de tasa de refresco del lazo (~20 Hz máximo con modo extreme)
            elapsed = time.perf_counter() - t_inicio
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

    except KeyboardInterrupt:
        print("\nTest interrumpido manualmente por el usuario.")
    finally:
        print("\n=== APAGANDO Y CERRANDO CONEXIONES ===")
        try:
            fg.apagar_salidas()
            fg.desconectar()
        except Exception:
            pass
        try:
            wt.desconectar()
        except Exception:
            pass
        print("Instrumentos desconectados de forma segura.")

if __name__ == "__main__":
    ejecutar_test_control_fp()