import time
import csv
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
DT = 1.0 / 20.0
FRECUENCIA_INICIAL_FG = 60.0
FASE_INICIAL = 0.0
AMPLITUD = 10.0
ARCHIVO_CSV = "incrementos_fase.csv"

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
# CONTROL EN TIEMPO REAL
# - Frecuencia del generador = frecuencia medida
# - Fase del generador += incremento de fase entre muestras
# - Los incrementos se registran en un CSV
# ============================================================
def control_tiempo_real(wt, fg):
    # Primera medición válida
    frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)
    while not (np.isfinite(frecuencia_medida) and np.isfinite(fase_medida)):
        time.sleep(DT)
        frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)

    # Asignar frecuencia medida y fase medida al generador
    cambiar_frecuencia(fg, frecuencia_medida)
    fase_generador = fase_medida
    cambiar_fase(fg, fase_generador)

    # Estado inicial
    fase_anterior = fase_medida

    # Abrir archivo CSV para registrar incrementos
    with open(ARCHIVO_CSV, mode="w", newline="") as archivo:
        escritor = csv.writer(archivo)
        escritor.writerow(["tiempo_s", "fase_medida_deg", "delta_fase_deg", "fase_generador_deg"])
        archivo.flush()

        tiempo_inicio = time.perf_counter()

        while True:
            inicio_ciclo = time.perf_counter()

            frecuencia_medida, fase_medida = leer_frecuencia_y_fase(wt)

            # Frecuencia: igualar a la medida
            if np.isfinite(frecuencia_medida):
                cambiar_frecuencia(fg, frecuencia_medida)

            # Fase: sumar directamente el incremento de fase
            if np.isfinite(fase_medida):
                delta_fase = fase_medida - fase_anterior
                fase_anterior = fase_medida
                fase_generador = fase_generador + delta_fase
                cambiar_fase(fg, fase_generador)

                # Registrar en CSV
                t = time.perf_counter() - tiempo_inicio
                escritor.writerow([f"{t:.6f}", f"{fase_medida:.6f}",
                                   f"{delta_fase:.6f}", f"{fase_generador:.6f}"])
                archivo.flush()

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

        # Control en tiempo real
        control_tiempo_real(wt, fg)

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
        print(f"\nDatos guardados en: {ARCHIVO_CSV}")
        print("\nPrograma finalizado.")

if __name__ == "__main__":
    main()