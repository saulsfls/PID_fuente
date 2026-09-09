import time
from datetime import datetime
import pyvisa
from wtcontroller import YokogawaWT3000


def ejecutar_prueba_lectura():
    # Ajusta la dirección GPIB según tu equipo
    DIRECCION_GPIB = "GPIB0::1::INSTR"
    ELEMENTO = 1  # Elemento de medición a consultar (1, 2, 3 o 4)
    INTERVALO_LECTURA_S = 0.5  # Tiempo entre consultas (segundos)

    wt = YokogawaWT3000(DIRECCION_GPIB)

    print("=" * 80)
    print(" TEST DE LECTURA EN CONSOLA EN TIEMPO REAL - YOKOGAWA WT3000")
    print("=" * 80)

    try:
        # 1. Conexión
        print(f"\n[1] Conectando a {DIRECCION_GPIB}...")
        idn = wt.conectar()
        print(f"    -> Conectado a: {idn}")

        # 2. Configuración del elemento
        print(f"\n[2] Configurando mediciones numéricas para Elemento {ELEMENTO}...")
        wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO)
        print("    -> Configuración lista.")

        # 3. Bucle de medición en tiempo real
        print("\n" + "=" * 80)
        print(" INICIANDO LECTURAS (Presiona Ctrl + C para salir)")
        print("=" * 80)
        print(
            f"{'Tiempo':<12} | {'V RMS (V)':<10} | {'I RMS (A)':<10} | {'P Activa (W)':<12} | {'F.P.':<8} | {'Frec (Hz)':<10}"
        )
        print("-" * 80)

        while True:
            t_actual = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            # Consulta directa al controlador
            m = wt.leer_mediciones_estandar()

            # Formatear la salida o mostrar OVER si el valor está fuera de rango
            v_str = f"{m['voltaje_rms']:10.4f}" if m["voltaje_rms"] is not None else "      OVER"
            i_str = f"{m['corriente_rms']:10.4f}" if m["corriente_rms"] is not None else "      OVER"
            p_str = f"{m['potencia_activa']:12.4f}" if m["potencia_activa"] is not None else "        OVER"
            pf_str = f"{m['factor_potencia']:8.4f}" if m["factor_potencia"] is not None else "    OVER"
            f_str = f"{m['frecuencia']:10.4f}" if m["frecuencia"] is not None else "      OVER"

            print(f"{t_actual:<12} | {v_str} | {i_str} | {p_str} | {pf_str} | {f_str}")

            time.sleep(INTERVALO_LECTURA_S)

    except pyvisa.VisaIOError as e:
        print(f"\n[X] Error de comunicación GPIB: {e}")

    except KeyboardInterrupt:
        print("\n\n[!] Prueba finalizada por el usuario.")

    finally:
        wt.desconectar()
        print("[!] Conexión GPIB cerrada correctamente.")


if __name__ == "__main__":
    ejecutar_prueba_lectura()