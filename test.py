import time
import pyvisa
from generadorcontroller import YokogawaFG420


def barrido_fase_grado_a_grado(gen: YokogawaFG420, canal: int, inicio: int = 0, fin: int = 360, retardo_paso_s: float = 0.003):
    """
    Realiza un barrido continuo de fase de 1° en 1° de forma ultrarrápida y gradual.
    
    :param gen: Instancia del controlador YokogawaFG420.
    :param canal: Canal objetivo (1 o 2).
    :param inicio: Grado inicial (ej. 0).
    :param fin: Grado final (ej. 360).
    :param retardo_paso_s: Tiempo entre cada grado en segundos (0.003s = 3ms).
    """
    paso = 1 if inicio <= fin else -1
    
    # Recorrido de 1° en 1°
    for grado in range(inicio, fin + paso, paso):
        fase_normalizada = grado % 360
        gen.establecer_fase(canal, float(fase_normalizada))
        time.sleep(retardo_paso_s)


def probar_barrido_1a1():
    direccion_gpib = "GPIB1::2::INSTR"

    print("=" * 65)
    print(" BARRIDO ULTRARRÁPIDO DE FASE DE 1° EN 1° (0° -> 360°)")
    print(" Generador: Yokogawa FG420 | Canal 1 (60 Hz / Circuito Abierto)")
    print("=" * 65)

    gen = YokogawaFG420(direccion_gpib)

    try:
        # 1. Conexión
        print("\n[1] Conectando con el generador...")
        idn = gen.conectar()
        print(f"    -> Conectado a: {idn}")

        # 2. Apagar CH2
        gen.establecer_salida(canal=2, estado=False)

        # 3. Configurar CH1 inicial a 60 Hz y 0° de fase
        print("\n[2] Configurando Canal 1 (Senoidal 60 Hz, 10 Vpp, Fase inicial: 0°)...")
        gen.configurar_canal(
            canal=1,
            funcion="SINusoid",
            frecuencia_hz=60.0,
            amplitud_vpp=10.0,
            offset_v=0.0,
            fase_grados=0.0,
            activar_salida=True,
        )
        print("    -> Canal 1 emitiendo señal. Esperando 2 segundos...")
        time.sleep(2)

        # 4. Barrido de 0° a 360° (360 comandos SCPI)
        print("\n[3] Iniciando barrido gradual de 0° a 360° (pasos de 1°)...")
        tiempo_inicio = time.time()
        
        barrido_fase_grado_a_grado(gen, canal=1, inicio=0, fin=360, retardo_paso_s=0.003)
        
        tiempo_total = time.time() - tiempo_inicio
        print(f"    -> Barrido completado en {tiempo_total:.2f} segundos.")

        # 5. Confirmación final en hardware
        estado = gen.obtener_estado_canal(1)
        print(f"    -> [Verificación Hardware] Fase actual: {estado['fase_grados']}°")

        time.sleep(2)

        # 6. Desactivar salida BNC por seguridad
        print("\n[4] Desactivando salida del Canal 1...")
        gen.establecer_salida(canal=1, estado=False)

        # 7. Error log
        error = gen.obtener_ultimo_error()
        print(f"    -> Registro de Errores SCPI: {error}")

        print("\n" + "=" * 65)
        print(" PRUEBA DE BARRIDO COMPLETO (1° A 1°) CONCLUIDA")
        print("=" * 65)

    except pyvisa.VisaIOError as e:
        print(f"\n[ERROR VISA]: {e}")
    finally:
        gen.desconectar()


if __name__ == "__main__":
    probar_barrido_1a1()