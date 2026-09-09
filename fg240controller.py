import time
import pyvisa


class YokogawaFG420:
    """Controlador de automatización mediante PyVISA para el generador de ondas Yokogawa FG420.
    
    Utiliza el direccionamiento explícito de subsistemas (:SOURce1: / :SOURce2:) 
    para garantizar independencia total entre canales y estabilidad en el bus GPIB.
    """

    def __init__(self, resource_address: str, timeout: int = 5000):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None

    def conectar(self) -> str:
        """Abre la conexión GPIB/VISA con el equipo y configura los delimitadores."""
        self.inst = self.rm.open_resource(self.address)
        self.inst.timeout = self.timeout
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        time.sleep(0.1)
        self.limpiar_errores()
        return self.obtener_idn()

    def desconectar(self):
        """Cierra la conexión con el instrumento de forma segura."""
        if self.inst:
            try:
                self.inst.close()
            except Exception:
                pass
        if self.rm:
            try:
                self.rm.close()
            except Exception:
                pass

    def escribir(self, comando: str):
        """Envía un comando SCPI al equipo con un breve retardo de procesamiento."""
        self.inst.write(comando)
        time.sleep(0.03)  # Pausa para dar tiempo de procesamiento al buffer del FG420

    def consultar(self, comando: str) -> str:
        """Consulta una instrucción SCPI y retorna la respuesta limpia."""
        respuesta = self.inst.query(comando).strip()
        time.sleep(0.03)  # Pausa entre lectura y siguiente escritura
        return respuesta

    def limpiar_errores(self):
        """Limpia los registros de estado y la cola de errores del sistema (*CLS)."""
        self.escribir("*CLS")

    def reset(self):
        """Restaura los parámetros de fábrica del equipo (*RST)."""
        self.escribir("*RST")
        time.sleep(0.5)

    def obtener_idn(self) -> str:
        """Obtiene la cadena de identificación del dispositivo."""
        return self.consultar("*IDN?")

    # --- CONFIGURACIÓN DE PARÁMETROS INDIVIDUALES POR CANAL ---

    def establecer_salida(self, canal: int, estado: bool):
        """Activa (True / 1) o desactiva (False / 0) la salida BNC del canal especificado."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        val = "ON" if estado else "OFF"
        self.escribir(f":OUTPut{canal} {val}")

    def establecer_forma_onda(self, canal: int, funcion: str):
        """Define la forma de onda: SINusoid, SQUare, PULSe, RAMP, ARBitrary, DC."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        self.escribir(f":SOURce{canal}:FUNCtion {funcion.upper()}")

    def establecer_frecuencia(self, canal: int, frecuencia_hz: float):
        """Establece la frecuencia en Hertz (Hz) para el canal especificado."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        self.escribir(f":SOURce{canal}:FREQuency {frecuencia_hz}")

    def establecer_amplitud_vpp(self, canal: int, amplitud_vpp: float):
        """Configura la amplitud Pico a Pico (Vpp) para el canal especificado."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        self.escribir(f":SOURce{canal}:VOLTage {amplitud_vpp}VPP")

    def establecer_offset(self, canal: int, offset_v: float):
        """Configura el Offset de Corriente Directa en Volts (V) para el canal especificado."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        self.escribir(f":SOURce{canal}:VOLTage:OFFSet {offset_v}V")

    def establecer_fase(self, canal: int, fase_grados: float):
        """Ajusta la fase de la señal en grados para el canal especificado."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        self.escribir(f":SOURce{canal}:PHASe {fase_grados}")

    # --- CONFIGURACIÓN INTEGRAL Y LECTURA DE ESTADO ---

    def configurar_canal(
        self,
        canal: int,
        funcion: str = "SINusoid",
        frecuencia_hz: float = 60.0,
        amplitud_vpp: float = 10.0,
        offset_v: float = 0.0,
        fase_grados: float = 0.0,
        activar_salida: bool = True,
    ):
        """Aplica una configuración completa de parámetros a un canal de forma independiente."""
        self.establecer_forma_onda(canal, funcion)
        self.establecer_frecuencia(canal, frecuencia_hz)
        self.establecer_amplitud_vpp(canal, amplitud_vpp)
        self.establecer_offset(canal, offset_v)
        self.establecer_fase(canal, fase_grados)
        self.establecer_salida(canal, activar_salida)
        time.sleep(0.05)

    def obtener_estado_canal(self, canal: int) -> dict:
        """Lee y devuelve el estado actual consultando directamente el árbol :SOURce del canal."""
        if canal not in [1, 2]:
            raise ValueError("El canal debe ser 1 o 2.")
        
        salida = self.consultar(f":OUTPut{canal}?")
        funcion = self.consultar(f":SOURce{canal}:FUNCtion?")
        frecuencia = float(self.consultar(f":SOURce{canal}:FREQuency?"))
        amplitud = float(self.consultar(f":SOURce{canal}:VOLTage?"))
        offset = float(self.consultar(f":SOURce{canal}:VOLTage:OFFSet?"))
        fase = float(self.consultar(f":SOURce{canal}:PHASe?"))

        return {
            "canal": canal,
            "salida": salida,
            "funcion": funcion,
            "frecuencia_hz": frecuencia,
            "amplitud_vpp": amplitud,
            "offset_v": offset,
            "fase_grados": fase,
        }

    def obtener_ultimo_error(self) -> str:
        """Devuelve el error más reciente registrado en la cola del sistema (*SYSTem:ERRor?)."""
        return self.consultar(":SYSTem:ERRor?")


# ============================================================
# EJECUCIÓN DIRECTA PARA PRUEBA DE STANDALONE
# ============================================================
if __name__ == "__main__":
    DIRECCION_GPIB = "GPIB0::10::INSTR"
    gen = YokogawaFG420(DIRECCION_GPIB)

    try:
        print("Probando controlador YokogawaFG420...")
        idn = gen.conectar()
        print(f"Conectado a: {idn}\n")

        # Configurar Canal 1 (60 Hz, 10 Vpp, SIN)
        gen.configurar_canal(1, funcion="SINusoid", frecuencia_hz=60.0, amplitud_vpp=10.0, activar_salida=True)
        # Configurar Canal 2 (1000 Hz, 5 Vpp, SQU)
        gen.configurar_canal(2, funcion="SQUare", frecuencia_hz=1000.0, amplitud_vpp=5.0, activar_salida=True)

        print("Verificando lecturas:")
        for ch in [1, 2]:
            st = gen.obtener_estado_canal(ch)
            print(f"CH{ch} -> Función: {st['funcion']} | Freq: {st['frecuencia_hz']} Hz | Volt: {st['amplitud_vpp']} Vpp")

        print(f"\nEstado de errores: {gen.obtener_ultimo_error()}")

    except pyvisa.VisaIOError as e:
        print(f"Error VISA: {e}")

    finally:
        gen.desconectar()
        print("Conexión cerrada.")