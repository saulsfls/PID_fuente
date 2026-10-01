import sys
import time
import math
import pyvisa

class YokogawaWT3000UnlockDSP:
    """Configuración de bajo nivel para forzar refresco real de 50 ms en el DSP del WT3000."""
    
    def __init__(self, resource_name: str, timeout_ms: int = 5000):
        self.resource_name = resource_name
        self.timeout_ms = timeout_ms
        self.rm = pyvisa.ResourceManager()
        self.instrument = None

    def connect(self) -> bool:
        """Abre la conexión con el instrumento."""
        try:
            self.instrument = self.rm.open_resource(self.resource_name)
            self.instrument.timeout = self.timeout_ms
            self.instrument.write_termination = '\n'
            self.instrument.read_termination = '\n'
            
            idn = self.instrument.query("*IDN?").strip()
            print(f"[OK] Conectado exitosamente: {idn}")
            return True
        except pyvisa.VisaIOError as e:
            print(f"[ERROR] No se pudo conectar al instrumento: {e}")
            return False

    def force_unlocked_50ms(self):
        """Desactiva Auto-Range, Averaging y sincroniza por U1 para permitir 20 Hz reales."""
        if not self.instrument:
            raise RuntimeError("El instrumento no está conectado.")
        
        self.instrument.write("*CLS")
        
        # 1. Configurar Update Rate
        self.instrument.write(":INPut:UPDate:HOLD OFF")
        self.instrument.write(":RATE 50MS")
        
        # 2. Desactivar Averaging (Promediado de señal)
        self.instrument.write(":INPut:AVERaging:STATe OFF")
        
        # 3. Fijar la fuente de sincronización en el canal de Voltaje (U1)
        # Esto evita que el DSP se quede esperando cruces por cero en corriente
        self.instrument.write(":INPut:SYNC:SOURce U1")
        
        # 4. Desactivar el Rango Automático en Elemento 1 (Desbloquea el DSP)
        self.instrument.write(":INPut:VOLTage:AUTO:ELEMENT1 OFF")
        self.instrument.write(":INPut:CURRent:AUTO:ELEMENT1 OFF")
        
        # 5. Formato ASCII y selección de variables
        self.instrument.write(":NUMeric:FORMAT ASCII")
        self.instrument.write(":NUMeric:ITEM1 U,1")       # V
        self.instrument.write(":NUMeric:ITEM2 I,1")       # A
        self.instrument.write(":NUMeric:ITEM3 P,1")       # W
        self.instrument.write(":NUMeric:ITEM4 S,1")       # VA
        self.instrument.write(":NUMeric:ITEM5 Q,1")       # var
        self.instrument.write(":NUMeric:ITEM6 LAMBda,1")  # PF
        self.instrument.write(":NUMeric:NUMber 6")

        confirmed_rate = self.instrument.query(":RATE?").strip()
        print(f"[INFO] Tasa en hardware: {confirmed_rate}")

    def fetch_data(self) -> list:
        """Consulta directa de valores numéricos."""
        raw_str = self.instrument.query(":NUMeric:VALue?").strip()
        
        values = []
        for val in raw_str.split(','):
            try:
                values.append(float(val))
            except ValueError:
                values.append(0.0)
                
        return values

    def close(self):
        """Cierra la conexión VISA."""
        if self.instrument:
            self.instrument.close()
            print("[INFO] Conexión con el WT3000 cerrada.")
        self.rm.close()


def is_duplicate(curr_data: list, last_data: list) -> bool:
    """Compara si los valores difieren."""
    if last_data is None:
        return False
    for idx in range(4):
        if not math.isclose(curr_data[idx], last_data[idx], rel_tol=1e-6, abs_tol=1e-6):
            return False
    return True


def main():
    VISA_ADDRESS = 'GPIB0::1::INSTR'
    wt3000 = YokogawaWT3000UnlockDSP(resource_name=VISA_ADDRESS)

    if not wt3000.connect():
        sys.exit(1)

    try:
        wt3000.force_unlocked_50ms()

        print("\n" + "="*75)
        print(" ADQUISICIÓN CON DSP DESBLOQUEADO (Rangos Fijos / Sync U1)")
        print("="*75 + "\n")

        sample_count = 0
        start_time = time.time()
        last_data = None

        while True:
            data = wt3000.fetch_data()

            if data and len(data) >= 6:
                if is_duplicate(data, last_data):
                    time.sleep(0.002)
                    continue

                last_data = data
                u1, i1, p1, s1, q1, pf1 = data[:6]
                sample_count += 1
                
                now = time.time()
                timestamp = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"

                print(f"[{timestamp}] Muestra Única #{sample_count:05d} | "
                      f"V: {u1:8.3f} V | "
                      f"I: {i1:8.4f} A | "
                      f"P: {p1:8.3f} W | "
                      f"PF: {pf1:6.4f}", flush=True)

    except KeyboardInterrupt:
        print("\n\n[INFO] Captura finalizada por el usuario.")
    except Exception as e:
        print(f"\n[ERROR] Ocurrió un error en tiempo de ejecución: {e}")
    finally:
        elapsed_time = time.time() - start_time if 'start_time' in locals() else 0
        if sample_count > 0 and elapsed_time > 0:
            print(f"\n[ESTADÍSTICAS] Muestras únicas capturadas: {sample_count} en {elapsed_time:.2f} s "
                  f"({sample_count / elapsed_time:.2f} muestras/segundo)")
        
        wt3000.close()


if __name__ == '__main__':
    main()