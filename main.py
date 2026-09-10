"""
SISTEMA DE CONTROL DE FP - VERSIÓN CON OFFSET DE FRECUENCIA
============================================================
Estrategia: Usar offset de frecuencia para controlar la fase
El offset crea un desfase acumulativo de forma estable
"""

import time
import numpy as np
from collections import deque
from datetime import datetime 
import pyvisa
from wtcontroller import YokogawaWT3000

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================

GPIB_YOKOGAWA_WT = "GPIB0::1::INSTR" #Wattmetro WT3000
GPIB_YOKOGAWA_FG = "GPIB1::2::INSTR" #Generador de funciones FG420

ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.5

FP_OBJETIVO = 1.0
MARGEN_FP_MIN = 0.970
MARGEN_FP_MAX = 1.030

# Límites del FG420
AMPLITUD_MIN = 0.5
AMPLITUD_MAX = 10.0
AMPLITUD_INICIAL = 5.0

# Límites de offset de frecuencia (Hz)
OFFSET_MIN = -0.5
OFFSET_MAX = 0.5
OFFSET_INICIAL = 0.0

# Límites de frecuencia (Hz)
FREC_MIN = 59.0
FREC_MAX = 61.0

# Parámetros PID para offset de frecuencia
KP_OFFSET = 0.08      # Ganancia proporcional
KI_OFFSET = 0.01      # Ganancia integral
KD_OFFSET = 0.10      # Ganancia derivativa

# Umbrales
DEADBAND_FP = 0.005

# ==============================================================================
# CLASE CONTROLADOR CON OFFSET DE FRECUENCIA
# ==============================================================================

class ControladorOffsetFrecuencia:
    """
    Controlador que usa offset de frecuencia para ajustar la fase
    """
    def __init__(self):
        # Buffers
        self.buffer_fp = deque(maxlen=15)
        self.buffer_frec_red = deque(maxlen=8)
        
        # Estado
        self.fp_suavizado = 1.0
        self.frecuencia_red = 60.0
        self.frecuencia_fg = 60.0
        self.offset_actual = 0.0
        self.amplitud_actual = AMPLITUD_INICIAL
        
        # Memoria del mejor punto
        self.mejor_fp = 0.0
        self.mejor_offset = 0.0
        self.conteo_estable = 0
        
        # PID para offset
        self.integral_offset = 0.0
        self.last_error = 0.0
        self.last_fp = 1.0
        
    def actualizar(self, fp_medido, frec_red_medida, dt):
        """
        Actualiza el controlador usando offset de frecuencia
        """
        # 1. Suavizar FP
        if fp_medido is not None and abs(fp_medido) < 2.0:
            fp_abs = abs(fp_medido)
            self.buffer_fp.append(fp_abs)
            if len(self.buffer_fp) >= 3:
                self.fp_suavizado = np.mean(self.buffer_fp)
            else:
                self.fp_suavizado = fp_abs
        
        # 2. Suavizar frecuencia de la red
        if frec_red_medida is not None and frec_red_medida > 10:
            self.buffer_frec_red.append(frec_red_medida)
            if len(self.buffer_frec_red) >= 3:
                frec_prom = np.mean(self.buffer_frec_red)
                self.frecuencia_red = (frec_prom * 0.3 + 
                                       self.frecuencia_red * 0.7)
        
        # 3. Calcular error
        error_fp = FP_OBJETIVO - self.fp_suavizado
        
        # 4. Actualizar mejor punto
        if self.fp_suavizado > self.mejor_fp and self.fp_suavizado < 1.1:
            self.mejor_fp = self.fp_suavizado
            self.mejor_offset = self.offset_actual
            self.conteo_estable = 0
        else:
            self.conteo_estable += 1
        
        # 5. Deadband
        if abs(error_fp) < DEADBAND_FP:
            return self._respuesta_sin_cambio()
        
        # 6. PID para offset (signo invertido para dirección correcta)
        p_term = -KP_OFFSET * error_fp * 10
        
        # Integral (limitada)
        if abs(error_fp) < 0.1:
            self.integral_offset += error_fp * dt * 5
            self.integral_offset = max(-0.1, min(0.1, self.integral_offset))
        else:
            self.integral_offset = 0.0
        i_term = -KI_OFFSET * self.integral_offset
        
        # Derivativo
        d_term = 0.0
        if dt > 0.001 and len(self.buffer_fp) > 2:
            cambio_fp = self.fp_suavizado - self.last_fp
            d_term = -KD_OFFSET * cambio_fp / (dt + 0.001) * 5
        
        # Acción total
        accion_offset = p_term + i_term + d_term
        
        # Limitar cambio de offset
        cambio_max = 0.02 * dt
        accion_offset = max(-cambio_max, min(cambio_max, accion_offset))
        
        # Si la acción es muy pequeña, no hacer nada
        if abs(accion_offset) < 0.0005:
            return self._respuesta_sin_cambio()
        
        # Aplicar cambio
        nuevo_offset = self.offset_actual + accion_offset
        nuevo_offset = max(OFFSET_MIN, min(OFFSET_MAX, nuevo_offset))
        
        self.last_error = error_fp
        self.last_fp = self.fp_suavizado
        
        # 7. Calcular frecuencia del FG
        frecuencia_fg = self.frecuencia_red + nuevo_offset
        frecuencia_fg = max(FREC_MIN, min(FREC_MAX, frecuencia_fg))
        
        if abs(frecuencia_fg - self.frecuencia_fg) > 0.0005:
            self.frecuencia_fg = frecuencia_fg
            self.offset_actual = nuevo_offset
            
            return {
                "amplitud": self.amplitud_actual,
                "frecuencia_fg": frecuencia_fg,
                "frecuencia_red": self.frecuencia_red,
                "offset": nuevo_offset,
                "fp": self.fp_suavizado,
                "error_fp": error_fp,
                "accion": f"OFF_{accion_offset:+.4f}",
                "cambio": True,
                "mejor_fp": self.mejor_fp,
                "mejor_offset": self.mejor_offset
            }
        
        return self._respuesta_sin_cambio()
    
    def _respuesta_sin_cambio(self):
        return {
            "amplitud": self.amplitud_actual,
            "frecuencia_fg": self.frecuencia_fg,
            "frecuencia_red": self.frecuencia_red,
            "offset": self.offset_actual,
            "fp": self.fp_suavizado,
            "error_fp": 0,
            "accion": "SIN_CAMBIO",
            "cambio": False,
            "mejor_fp": self.mejor_fp,
            "mejor_offset": self.mejor_offset
        }

# ==============================================================================
# FUNCIONES PARA FG420
# ==============================================================================

def configurar_fg420_offset(fg_inst, frecuencia, amplitud):
    """Configura el FG420 con frecuencia y amplitud"""
    try:
        fg_inst.clear()
        time.sleep(0.05)
        
        fg_inst.write_termination = '\n'
        fg_inst.read_termination = '\n'
        fg_inst.timeout = 5000
        
        # Forma de onda sinusoidal
        fg_inst.write(":SOURce1:FUNCtion:SHAPe SIN")
        time.sleep(0.02)
        
        # Frecuencia
        fg_inst.write(f":SOURce1:FREQuency {frecuencia:.6f}HZ")
        time.sleep(0.02)
        
        # Amplitud
        fg_inst.write(f":SOURce1:VOLTage:AMPLitude {amplitud:.3f}V")
        time.sleep(0.02)
        
        # Carga
        try:
            fg_inst.write(":OUTPut1:LOAD INF")
            time.sleep(0.02)
        except:
            pass
        
        # Activar salida
        fg_inst.write(":OUTPut1:STATe ON")
        time.sleep(0.05)
        
        return True
    except Exception as e:
        print(f"  ✗ Error configurando FG420: {e}")
        return False

def actualizar_fg420_offset(fg_inst, frecuencia, amplitud):
    """Actualiza frecuencia y amplitud del FG420"""
    try:
        fg_inst.write(f":SOURce1:FREQuency {frecuencia:.6f}HZ")
        time.sleep(0.01)
        
        fg_inst.write(f":SOURce1:VOLTage:AMPLitude {amplitud:.3f}V")
        time.sleep(0.01)
        
        return True
    except Exception as e:
        print(f"  ✗ Error actualizando FG420: {e}")
        return False

# ==============================================================================
# FUNCIÓN PRINCIPAL
# ==============================================================================

def main():
    rm = pyvisa.ResourceManager()
    wt = YokogawaWT3000(GPIB_YOKOGAWA_WT)
    fg = None
    
    controlador = ControladorOffsetFrecuencia()
    
    total_iteraciones = 0
    iteraciones_en_rango = 0
    fp_historico = []
    
    print("=" * 90)
    print(" SISTEMA DE CONTROL DE FP - CON OFFSET DE FRECUENCIA")
    print(" ===================================================")
    print(" Estrategia:")
    print("   - Usar offset de frecuencia para crear desfase")
    print("   - Offset estable y predecible")
    print("   - Memoria del mejor punto encontrado")
    print("=" * 90)
    print(f"\n Configuración:")
    print(f"   - Rango offset: {OFFSET_MIN}Hz a {OFFSET_MAX}Hz")
    
    try:
        print("\n[1/4] Conectando al WT3000...")
        wt.conectar()
        wt.configurar_salida_numerica_estandar(elemento_entrada=ELEMENTO_WT)
        print("  ✓ WT3000 conectado")
        
        print("\n[2/4] Conectando al FG420...")
        fg = rm.open_resource(GPIB_YOKOGAWA_FG)
        
        if not configurar_fg420_offset(fg, 60.0, AMPLITUD_INICIAL):
            raise Exception("Error en FG420")
        
        print(f"  ✓ FG420 configurado: 60.000 Hz, {AMPLITUD_INICIAL:.3f}V")
        
        print("\n[3/4] Esperando estabilización inicial...")
        for i in range(5):
            time.sleep(0.5)
            print("  .", end="", flush=True)
        print(" ✓")
        
        print("\n[4/4] INICIANDO CONTROL")
        print("=" * 90)
        print(f"{'Tiempo':<12} | {'FP':<10} | {'FP Prom':<10} | {'Frec FG':<12} | {'Offset':<12} | {'Mejor FP':<10} | {'Acción':<12} | {'Estado':<10}")
        print("-" * 90)
        
        tiempo_inicio = time.time()
        tiempo_anterior = time.time()
        tiempo_control = None
        primer_fp = False
        
        while True:
            if tiempo_control is not None:
                if time.time() - tiempo_control > TIEMPO_PRUEBA_SEG:
                    break
            
            t_actual = time.time()
            dt = t_actual - tiempo_anterior
            tiempo_anterior = t_actual
            
            m = wt.leer_mediciones_estandar()
            
            fp_medido = m.get("factor_potencia")
            frec_medida = m.get("frecuencia")
            
            if fp_medido is None or frec_medida is None:
                print(f"{datetime.now().strftime('%H:%M:%S')} | {'---':<10} | {'---':<10} | {'---':<12} | {'---':<12} | {'---':<10} | {'ERROR':<12} | {'COM_ERR':<10}")
                time.sleep(INTERVALO_MUESTREO)
                continue
            
            fp_abs = abs(fp_medido)
            
            if not primer_fp:
                primer_fp = True
                tiempo_control = t_actual
                print(f"\n[!] Señal detectada: FP={fp_abs:.4f}")
                print("    Iniciando control con offset...\n")
                print(f"{'Tiempo':<12} | {'FP':<10} | {'FP Prom':<10} | {'Frec FG':<12} | {'Offset':<12} | {'Mejor FP':<10} | {'Acción':<12} | {'Estado':<10}")
                print("-" * 90)
            
            resultado = controlador.actualizar(fp_medido, frec_medida, dt)
            
            if resultado["cambio"]:
                actualizar_fg420_offset(
                    fg,
                    resultado["frecuencia_fg"],
                    resultado["amplitud"]
                )
            
            segundos = int(time.time() - tiempo_control)
            
            en_rango = MARGEN_FP_MIN <= fp_abs <= MARGEN_FP_MAX
            if en_rango:
                iteraciones_en_rango += 1
            
            total_iteraciones += 1
            fp_historico.append(fp_abs)
            
            timer_display = f"{segundos:03d}s"
            str_3pct = "✓ RANGO" if en_rango else "✗ FUERA"
            
            mejor_fp_str = f"{resultado['mejor_fp']:.4f}"
            
            print(f"{timer_display:<12} | {fp_abs:<10.4f} | {resultado['fp']:<10.4f} | "
                  f"{resultado['frecuencia_fg']:<12.4f} | {resultado['offset']:<+12.4f} | "
                  f"{mejor_fp_str:<10} | {resultado['accion']:<12} | {str_3pct:<10}")
            
            time.sleep(max(0, INTERVALO_MUESTREO - (time.time() - t_actual)))
        
        # ======================================================================
        # RESUMEN FINAL
        # ======================================================================
        
        efectividad = (iteraciones_en_rango / total_iteraciones * 100) if total_iteraciones > 0 else 0
        
        print("\n" + "=" * 90)
        print(" RESUMEN FINAL")
        print("=" * 90)
        print(f" Total iteraciones:           {total_iteraciones}")
        print(f" En rango (1±3%):             {iteraciones_en_rango}")
        print(f" Efectividad:                 {efectividad:.2f}%")
        if fp_historico:
            print(f" FP promedio final:           {np.mean(fp_historico[-50:]):.4f}")
            print(f" Desviación estándar FP:      {np.std(fp_historico[-50:]):.4f}")
        print(f" Mejor FP encontrado:         {controlador.mejor_fp:.4f}")
        print(f" Offset en mejor FP:          {controlador.mejor_offset:.4f} Hz")
        print(f" Offset final:                {controlador.offset_actual:.4f} Hz")
        print(f" Frecuencia FG final:         {controlador.frecuencia_fg:.4f} Hz")
        print("=" * 90)
        
    except KeyboardInterrupt:
        print("\n\n[!] Prueba cancelada por el usuario.")
    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[!] Cerrando conexiones...")
        
        if fg:
            try:
                fg.write(":OUTPut1:STATe OFF")
                time.sleep(0.1)
                fg.close()
                print("  ✓ FG420 apagado")
            except:
                print("  ✗ Error al cerrar FG420")
        
        if wt:
            try:
                wt.desconectar()
                print("  ✓ WT3000 desconectado")
            except:
                print("  ✗ Error al desconectar WT3000")
        
        print("[✓] Conexiones cerradas")
        print("=" * 90)

if __name__ == "__main__":
    main()