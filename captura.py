"""
TEST DE FRECUENCIA DE LA RED CON WT3000
========================================
Objetivo: Medir la frecuencia de la tensión de red con el WT3000
en modo rápido para obtener estadísticas detalladas.
Configuración optimizada para máxima velocidad de muestreo.
"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import pyvisa
import os

from controllers.wt3000controller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

DIR_WT = "GPIB0::1::INSTR"          # Dirección del WT3000
NUM_MUESTRAS = 1000                 # Número de muestras a tomar
INTERVALO_MUESTREO = 0.02           # Segundos entre lecturas (20 ms)
TIMEOUT = 5000                      # Timeout en ms (5 s)

# ==============================================================================
# CLASE EXTENDIDA PARA LECTURA RÁPIDA DE FRECUENCIA
# ==============================================================================

class WT3000FastFrequency(YokogawaWT3000):
    """
    Extiende YokogawaWT3000 para añadir un método de lectura rápida de frecuencia.
    """
    def configurar_solo_frecuencia(self, elemento=1):
        """
        Configura el WT3000 para medir solo la frecuencia del elemento especificado.
        Esto reduce el tiempo de transferencia de datos.
        """
        self.escribir(":NUMeric:FORMAT ASCII")
        self.escribir(":NUMeric:NUMBER 1")
        self.escribir(f":NUMeric:ITEM1 FU,{elemento}")  # Frecuencia de tensión
        # Forzar actualización
        self.escribir(":INITiate:CONTinuous ON")
        time.sleep(0.1)

    def leer_frecuencia(self) -> float:
        """
        Lee solo la frecuencia de la tensión de red (valor numérico).
        Retorna None si hay error.
        """
        try:
            # Consultar el primer (y único) valor numérico
            respuesta = self.consultar(":NUMeric:VALue?")
            partes = respuesta.strip().split(',')
            if len(partes) >= 1:
                val = float(partes[0])
                # Verificar sobreescala
                if abs(val) > 1e30:
                    return None
                return val
            return None
        except Exception:
            return None

# ==============================================================================
# FUNCIONES AUXILIARES
# ==============================================================================

def mostrar_estadisticas(datos):
    """Calcula y muestra estadísticas básicas."""
    if not datos:
        print("No hay datos para analizar.")
        return

    arr = np.array(datos)
    media = np.mean(arr)
    desv = np.std(arr)
    minimo = np.min(arr)
    maximo = np.max(arr)
    p95 = np.percentile(arr, 95)
    p99 = np.percentile(arr, 99)
    rango = maximo - minimo

    print("\n" + "=" * 80)
    print(" ESTADÍSTICAS DE FRECUENCIA DE LA RED")
    print("=" * 80)
    print(f" Número de muestras:       {len(datos)}")
    print(f" Frecuencia media:         {media:.6f} Hz")
    print(f" Desviación estándar:      {desv:.6f} Hz")
    print(f" Frecuencia mínima:        {minimo:.6f} Hz")
    print(f" Frecuencia máxima:        {maximo:.6f} Hz")
    print(f" Rango (pico a pico):      {rango:.6f} Hz")
    print(f" Percentil 95:             {p95:.6f} Hz")
    print(f" Percentil 99:             {p99:.6f} Hz")
    print(f" Desviación relativa:      {(desv/media*100):.4f} %")
    print("=" * 80)

    return {
        "media": media,
        "desviacion": desv,
        "minimo": minimo,
        "maximo": maximo,
        "rango": rango,
        "p95": p95,
        "p99": p99,
        "desv_relativa": desv/media*100
    }

def exportar_excel(datos, estadisticas, filename=None):
    """Exporta los datos y estadísticas a un archivo Excel."""
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"frecuencia_red_{timestamp}.xlsx"

    # Crear DataFrame con las muestras
    df_datos = pd.DataFrame({
        "Muestra": range(1, len(datos)+1),
        "Frecuencia_Hz": datos,
        "Tiempo_s": np.arange(0, len(datos)*INTERVALO_MUESTREO, INTERVALO_MUESTREO)[:len(datos)]
    })

    # Crear DataFrame con estadísticas
    df_stats = pd.DataFrame([estadisticas])

    # Escribir a Excel con dos hojas
    with pd.ExcelWriter(filename, engine='openpyxl') as writer:
        df_datos.to_excel(writer, sheet_name='Muestras', index=False)
        df_stats.to_excel(writer, sheet_name='Estadisticas', index=False)

    print(f"\n✓ Datos exportados a: {filename}")
    return filename

def graficar_datos(datos, estadisticas, guardar=True):
    """Genera gráficas de la evolución temporal y el histograma."""
    if not datos:
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Gráfica temporal
    tiempo = np.arange(0, len(datos)*INTERVALO_MUESTREO, INTERVALO_MUESTREO)[:len(datos)]
    ax1.plot(tiempo, datos, 'b-', linewidth=0.8, alpha=0.7)
    ax1.axhline(y=estadisticas["media"], color='r', linestyle='--', label=f'Media: {estadisticas["media"]:.6f} Hz')
    ax1.fill_between(tiempo, 
                     estadisticas["media"] - estadisticas["desviacion"], 
                     estadisticas["media"] + estadisticas["desviacion"], 
                     color='gray', alpha=0.2, label=f'±1σ ({estadisticas["desviacion"]:.6f} Hz)')
    ax1.set_xlabel('Tiempo (s)')
    ax1.set_ylabel('Frecuencia (Hz)')
    ax1.set_title('Evolución temporal de la frecuencia de la red')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Histograma
    ax2.hist(datos, bins=30, color='skyblue', edgecolor='black', alpha=0.7)
    ax2.axvline(x=estadisticas["media"], color='r', linestyle='--', label=f'Media: {estadisticas["media"]:.6f} Hz')
    ax2.axvline(x=estadisticas["media"] - estadisticas["desviacion"], color='gray', linestyle=':', label=f'-1σ')
    ax2.axvline(x=estadisticas["media"] + estadisticas["desviacion"], color='gray', linestyle=':', label=f'+1σ')
    ax2.set_xlabel('Frecuencia (Hz)')
    ax2.set_ylabel('Frecuencia absoluta')
    ax2.set_title('Distribución de la frecuencia de la red')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if guardar:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"frecuencia_red_grafica_{timestamp}.png"
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Gráfica guardada como: {filename}")

    plt.show()

# ==============================================================================
# FUNCIÓN PRINCIPAL
# ==============================================================================

def main():
    print("\n" + "=" * 90)
    print(" TEST DE FRECUENCIA DE LA RED CON WT3000")
    print(" ========================================")
    print(f" Modo: Ultrarrápido (lectura optimizada)")
    print(f" Número de muestras: {NUM_MUESTRAS}")
    print(f" Intervalo de muestreo: {INTERVALO_MUESTREO*1000:.1f} ms")
    print("=" * 90)

    # Inicializar controlador en modo 'fast' (ya está optimizado)
    wt = WT3000FastFrequency(DIR_WT, mode='fast', timeout=TIMEOUT)

    try:
        # ---------- Conexión y configuración ----------
        print("\n[1/3] Conectando al WT3000...")
        wt.conectar()
        print("  ✓ WT3000 conectado")

        print("\n[2/3] Configurando modo rápido (solo frecuencia)...")
        wt.configurar_solo_frecuencia(elemento=1)
        print("  ✓ Configuración aplicada")

        print("\n[3/3] Iniciando muestreo...")
        print(f"  Tomando {NUM_MUESTRAS} muestras cada {INTERVALO_MUESTREO*1000:.1f} ms...")
        print("  (Presiona Ctrl+C para interrumpir)")

        # ---------- Toma de muestras ----------
        muestras = []
        tiempos = []
        errores = 0

        inicio = time.time()
        for i in range(NUM_MUESTRAS):
            t_inicio = time.time()
            freq = wt.leer_frecuencia()
            if freq is not None:
                muestras.append(freq)
                tiempos.append(t_inicio - inicio)
            else:
                errores += 1

            # Mostrar progreso cada 100 muestras
            if (i+1) % 100 == 0:
                print(f"  Progreso: {i+1}/{NUM_MUESTRAS} muestras (errores: {errores})")

            # Esperar hasta el siguiente intervalo
            tiempo_espera = max(0, INTERVALO_MUESTREO - (time.time() - t_inicio))
            time.sleep(tiempo_espera)

        duracion_total = time.time() - inicio

        print(f"\n✓ Muestreo completado en {duracion_total:.2f} s")
        print(f"  Muestras válidas: {len(muestras)}")
        print(f"  Errores: {errores}")

        if len(muestras) == 0:
            print("  ✗ No se obtuvieron muestras válidas.")
            return

        # ---------- Análisis estadístico ----------
        estadisticas = mostrar_estadisticas(muestras)

        # ---------- Exportación a Excel ----------
        archivo_excel = exportar_excel(muestras, estadisticas)

        # ---------- Gráficas ----------
        print("\nGenerando gráficas...")
        graficar_datos(muestras, estadisticas)

        # ---------- Resumen final ----------
        print("\n" + "=" * 90)
        print(" RESUMEN DE LA PRUEBA")
        print("=" * 90)
        print(f" Duración total:          {duracion_total:.2f} s")
        print(f" Muestras tomadas:        {len(muestras)}")
        print(f" Tasa de muestreo:        {len(muestras)/duracion_total:.1f} muestras/s")
        print(f" Frecuencia media:        {estadisticas['media']:.6f} Hz")
        print(f" Desviación estándar:     {estadisticas['desviacion']:.6f} Hz")
        print(f" Archivo Excel generado:  {archivo_excel}")
        print("=" * 90)

    except KeyboardInterrupt:
        print("\n\n[!] Prueba cancelada por el usuario.")
        if muestras:
            print(f"  Se tomaron {len(muestras)} muestras antes de la interrupción.")
            estadisticas = mostrar_estadisticas(muestras)
            exportar_excel(muestras, estadisticas, f"frecuencia_red_interrumpido_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx")
    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[!] Cerrando conexión...")
        if wt:
            try:
                wt.desconectar()
                print("  ✓ WT3000 desconectado")
            except Exception as e:
                print(f"  ✗ Error al desconectar: {e}")
        print("[✓] Prueba finalizada")

if __name__ == "__main__":
    main()