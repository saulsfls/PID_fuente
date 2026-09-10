import time
import numpy as np
import pyvisa
from wtcontroller import YokogawaWT3000
from fg240controller import YokogawaFG420

# ============================================================
# CONFIGURACIÓN
# ============================================================
GPIB_WT = "GPIB0::1::INSTR"
GPIB_FG = "GPIB1::2::INSTR"
ELEMENTO_WT = 1
TIEMPO_ADQUISICION = 20.0
DT = 1.0 / 20.0
FRECUENCIA_INICIAL_FG = 60.0
FASE_INICIAL = 0.0
AMPLITUD = 10.0
PARAMETROS_INICIALES = np.array([0.9, 0.1, 0.1])  # Kp, Kd, alpha

# ============================================================
# FUNCIÓN PID
# ============================================================
def PID(valor, dt, Kp, Kd, alpha, derivative_prev, valor_prev):
    P = Kp * valor
    derivative_raw = (valor - valor_prev) / dt
    derivative = alpha * derivative_prev + (1.0 - alpha) * derivative_raw
    D = Kd * derivative
    output = P + D
    return {"output": output, "P": P, "D": D, "derivada": derivative, "valor": valor}

# ============================================================
# FUNCIÓN DE ERROR PARA OPTIMIZACIÓN
# ============================================================
def funcion_error(par, x, dt):
    Kp, Kd, alpha = par
    derivative_anterior = 0.0
    valor_anterior = x[0]
    y = np.full(len(x), np.nan)
    for i in range(1, len(x)):
        resultado = PID(valor=x[i-1], dt=dt, Kp=Kp, Kd=Kd, alpha=alpha,
                        derivative_prev=derivative_anterior, valor_prev=valor_anterior)
        y[i] = resultado["output"]
        derivative_anterior = resultado["derivada"]
        valor_anterior = resultado["valor"]
    return np.nansum((x - y) ** 2)

# ============================================================
# OPTIMIZACIÓN DE PARÁMETROS
# ============================================================
def optimizar_pid(x, dt):
    from scipy.optimize import minimize
    print("\n============================================\n OPTIMIZACIÓN DEL PID\n============================================\n")
    print(f"Parámetros iniciales:\nKp    = {PARAMETROS_INICIALES[0]}\nKd    = {PARAMETROS_INICIALES[1]}\nalpha = {PARAMETROS_INICIALES[2]}")
    resultado = minimize(funcion_error, PARAMETROS_INICIALES, args=(x, dt), method="Nelder-Mead")
    Kp, Kd, alpha = resultado.x
    print("\n============================================\n PARÁMETROS ÓPTIMOS\n============================================\n")
    print(f"Kp    = {Kp:.12g}\nKd    = {Kd:.12g}\nalpha = {alpha:.12g}\n")
    print(f"Error de optimización = {resultado.fun:.12g}\n")
    return Kp, Kd, alpha

# ============================================================
# LECTURA DE FRECUENCIA Y FASE
# ============================================================
def leer_frecuencia_y_fase(wt):
    """Lee frecuencia y fase del elemento configurado. Devuelve (frecuencia, fase)."""
    try:
        mediciones = wt.leer_mediciones_estandar()
        frecuencia = mediciones.get("frecuencia")
        fase = mediciones.get("angulo_fase")
        if frecuencia is None or not np.isfinite(frecuencia):
            frecuencia = np.nan
        if fase is None or not np.isfinite(fase):
            fase = np.nan
        return frecuencia, fase
    except Exception:
        return np.nan, np.nan

# ============================================================
# CONFIGURACIÓN DEL FG420
# ============================================================
def configurar_fg420(fg, frecuencia, fase, amplitud):
    """Configura el canal 1 del generador con frecuencia y fase dadas."""
    fg.configurar_canal(
        canal=1,
        funcion="SINusoid",
        frecuencia_hz=frecuencia,
        amplitud_vpp=amplitud,
        offset_v=0.0,
        fase_grados=fase,
        activar_salida=True
    )

# ============================================================
# CAMBIO DE FRECUENCIA
# ============================================================
def cambiar_frecuencia(fg, frecuencia):
    """Actualiza la frecuencia del canal 1."""
    fg.establecer_frecuencia(1, frecuencia)

# ============================================================
# CAMBIO DE FASE
# ============================================================
def cambiar_fase(fg, fase):
    """Actualiza la fase del canal 1."""
    fg.establecer_fase(1, fase)

# ============================================================
# ADQUISICIÓN INICIAL DE FASE
# ============================================================
def adquirir_fase(wt):
    print("\n============================================\n ADQUISICIÓN INICIAL\n============================================\n")
    print(f"Tiempo de adquisición: {TIEMPO_ADQUISICION} s\nIntervalo de muestreo: {DT:.6f} s\n")
    x = []
    tiempo_inicio = time.perf_counter()
    siguiente_muestra = tiempo_inicio
    while True:
        ahora = time.perf_counter()
        tiempo = ahora - tiempo_inicio
        if tiempo >= TIEMPO_ADQUISICION:
            break
        _, fase = leer_frecuencia_y_fase(wt)
        if np.isfinite(fase):
            x.append(fase)
            print(f"t = {tiempo:8.3f} s   fase = {fase:.6f} deg")
        else:
            print("Diferencia de fase inválida.")
        siguiente_muestra += DT
        espera = siguiente_muestra - time.perf_counter()
        if espera > 0:
            time.sleep(espera)
    x = np.asarray(x, dtype=float)
    if len(x) < 3:
        raise RuntimeError("No se obtuvieron suficientes muestras para optimizar el PID.")
    print(f"\nMuestras obtenidas: {len(x)}")
    return x

# ============================================================
# CONTROL EN TIEMPO REAL
# - Frecuencia del generador = frecuencia medida
# - Fase del generador: tratamiento del programa PID
#   (delta_fase → PID → corrección acumulada)
# ============================================================
def control_tiempo_real(wt, fg, Kp, Kd, alpha):
    # Primera medición válida
    frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)
    while not (np.isfinite(frecuencia_medida) and np.isfinite(fase_medida)):
        time.sleep(DT)
        frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)

    # Asignar frecuencia medida y fase medida al generador
    cambiar_frecuencia(fg, frecuencia_medida)
    fase_generador = fase_medida
    cambiar_fase(fg, fase_generador)

    # Estados iniciales
    fase_anterior = fase_medida
    delta_fase_anterior = 0.0
    derivative_anterior = 0.0

    while True:
        inicio_ciclo = time.perf_counter()

        frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)

        # Frecuencia: igualar a la medida
        if np.isfinite(frecuencia_medida):
            cambiar_frecuencia(fg, frecuencia_medida)

        # Fase: mismo tratamiento que el programa PID
        if np.isfinite(fase_medida):
            delta_fase = fase_medida - fase_anterior
            fase_anterior = fase_medida
        else:
            delta_fase = delta_fase_anterior

        resultado = PID(valor=delta_fase, dt=DT, Kp=Kp, Kd=Kd, alpha=alpha,
                        derivative_prev=derivative_anterior, valor_prev=delta_fase_anterior)
        correccion = resultado["output"]
        derivative_anterior = resultado["derivada"]
        delta_fase_anterior = delta_fase

        fase_generador = fase_generador + correccion
        cambiar_fase(fg, fase_generador)

        # Mantener intervalo DT
        tiempo_ciclo = time.perf_counter() - inicio_ciclo
        espera = DT - tiempo_ciclo
        if espera > 0:
            time.sleep(espera)

# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================
def main():
    wt = None
    fg = None
    try:
        print("\n============================================\n SISTEMA DE CONTROL DE FRECUENCIA Y FASE\n============================================\n")

        # Conectar WT3000
        print("Conectando WT3000...")
        wt = YokogawaWT3000(GPIB_WT)
        idn_wt = wt.conectar()
        print(f"WT3000 conectado: {idn_wt}")
        wt.configurar_salida_numerica_estandar(ELEMENTO_WT)

        # Conectar FG420
        print("Conectando FG420...")
        fg = YokogawaFG420(GPIB_FG)
        idn_fg = fg.conectar()
        print(f"FG420 conectado: {idn_fg}")

        # Configurar generador (frecuencia y fase iniciales)
        configurar_fg420(fg, FRECUENCIA_INICIAL_FG, FASE_INICIAL, AMPLITUD)
        print(f"\nFG420 configurado a {FRECUENCIA_INICIAL_FG:.3f} Hz con fase inicial {FASE_INICIAL:.3f}°\nAmplitud = {AMPLITUD:.3f} Vpp")
        print("\nEsperando estabilización...")
        time.sleep(2.5)

        # Adquisición y optimización (solo para la fase)
        x = adquirir_fase(wt)
        dx = np.diff(x)
        Kp, Kd, alpha = optimizar_pid(dx, DT)

        # Control en tiempo real
        control_tiempo_real(wt, fg, Kp, Kd, alpha)

    except KeyboardInterrupt:
        print("\nControl detenido por el usuario.")
    except Exception as e:
        print("\n============================================\n ERROR\n============================================\n")
        print(f"{type(e).__name__}\n{str(e)}")
    finally:
        if fg is not None:
            try:
                fg.establecer_salida(1, False)
            except Exception:
                pass
            try:
                fg.desconectar()
            except Exception:
                pass
        if wt is not None:
            try:
                wt.desconectar()
            except Exception:
                pass
        print("\nPrograma finalizado.")

if __name__ == "__main__":
    main()