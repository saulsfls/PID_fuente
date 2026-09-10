import time
import pyvisa

class YokogawaFG420:
    """Controlador para el generador de ondas Yokogawa FG420.
    Soporta tres modos: 'fast', 'balanced', 'precise'.
    """

    def __init__(self, resource_address: str, timeout: int = 5000, mode: str = 'balanced'):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None
        self.mode = mode.lower()
        self._configure_mode()

    def _configure_mode(self):
        """Ajusta los tiempos de sleep y comportamiento según el modo."""
        if self.mode == 'fast':
            self.write_sleep = 0.001      # 1 ms
            self.query_sleep = 0.001
            self.use_opc = False
            self.use_chaining = True      # Usar comandos encadenados en configurar_canal
        elif self.mode == 'balanced':
            self.write_sleep = 0.005      # 5 ms
            self.query_sleep = 0.005
            self.use_opc = False
            self.use_chaining = False
        elif self.mode == 'precise':
            self.write_sleep = 0.03       # 30 ms
            self.query_sleep = 0.03
            self.use_opc = True
            self.use_chaining = False
        else:
            raise ValueError("Modo debe ser 'fast', 'balanced' o 'precise'")

    def set_mode(self, mode: str):
        """Cambia el modo de operación en tiempo de ejecución."""
        self.mode = mode.lower()
        self._configure_mode()

    def conectar(self) -> str:
        self.inst = self.rm.open_resource(self.address)
        self.inst.timeout = self.timeout
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        time.sleep(0.1)
        self.limpiar_errores()
        return self.obtener_idn()

    def desconectar(self):
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

    def escribir(self, comando: str, opc: bool = None):
        """Envía un comando SCPI. Si opc es True, espera a que termine (*OPC?)."""
        self.inst.write(comando)
        if opc or (self.use_opc and 'FREQ' in comando.upper()):  # ejemplo: esperar en cambios de frecuencia
            self.inst.query("*OPC?")
        time.sleep(self.write_sleep)

    def consultar(self, comando: str) -> str:
        respuesta = self.inst.query(comando).strip()
        time.sleep(self.query_sleep)
        return respuesta

    def limpiar_errores(self):
        self.escribir("*CLS")

    def reset(self):
        self.escribir("*RST")
        time.sleep(0.5)

    def obtener_idn(self) -> str:
        return self.consultar("*IDN?")

    # --- Métodos de configuración por canal ---

    def establecer_salida(self, canal: int, estado: bool):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        val = "ON" if estado else "OFF"
        self.escribir(f":OUTPut{canal} {val}")

    def establecer_forma_onda(self, canal: int, funcion: str):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        self.escribir(f":SOURce{canal}:FUNCtion {funcion.upper()}")

    def establecer_frecuencia(self, canal: int, frecuencia_hz: float):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        self.escribir(f":SOURce{canal}:FREQuency {frecuencia_hz}")

    def establecer_amplitud_vpp(self, canal: int, amplitud_vpp: float):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        self.escribir(f":SOURce{canal}:VOLTage {amplitud_vpp}VPP")

    def establecer_offset(self, canal: int, offset_v: float):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        self.escribir(f":SOURce{canal}:VOLTage:OFFSet {offset_v}V")

    def establecer_fase(self, canal: int, fase_grados: float):
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        self.escribir(f":SOURce{canal}:PHASe {fase_grados}")

    # --- Configuración integral ---

    def configurar_canal(self, canal: int, funcion: str = "SINusoid",
                         frecuencia_hz: float = 60.0, amplitud_vpp: float = 10.0,
                         offset_v: float = 0.0, fase_grados: float = 0.0,
                         activar_salida: bool = True):
        """Configuración estándar (usando comandos individuales)."""
        if self.use_chaining and self.mode == 'fast':
            # En modo fast usamos la versión encadenada para máxima velocidad
            return self.configurar_canal_rapido(canal, funcion, frecuencia_hz,
                                                amplitud_vpp, offset_v, fase_grados, activar_salida)
        self.establecer_forma_onda(canal, funcion)
        self.establecer_frecuencia(canal, frecuencia_hz)
        self.establecer_amplitud_vpp(canal, amplitud_vpp)
        self.establecer_offset(canal, offset_v)
        self.establecer_fase(canal, fase_grados)
        self.establecer_salida(canal, activar_salida)
        time.sleep(0.01)  # pequeño margen

    def configurar_canal_rapido(self, canal: int, funcion: str = "SINusoid",
                                frecuencia_hz: float = 60.0, amplitud_vpp: float = 10.0,
                                offset_v: float = 0.0, fase_grados: float = 0.0,
                                activar_salida: bool = True):
        """Envía todos los parámetros en un único comando SCPI (separados por ';')."""
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        on_off = "ON" if activar_salida else "OFF"
        cmd = (f":SOURce{canal}:FUNCtion {funcion.upper()};"
               f":SOURce{canal}:FREQuency {frecuencia_hz};"
               f":SOURce{canal}:VOLTage {amplitud_vpp}VPP;"
               f":SOURce{canal}:VOLTage:OFFSet {offset_v}V;"
               f":SOURce{canal}:PHASe {fase_grados};"
               f":OUTPut{canal} {on_off}")
        self.escribir(cmd, opc=self.use_opc)  # si use_opc=True, espera a que termine todo
        time.sleep(0.005)

    def obtener_estado_canal(self, canal: int) -> dict:
        if canal not in [1,2]: raise ValueError("Canal 1 o 2")
        salida = self.consultar(f":OUTPut{canal}?")
        funcion = self.consultar(f":SOURce{canal}:FUNCtion?")
        frecuencia = float(self.consultar(f":SOURce{canal}:FREQuency?"))
        amplitud = float(self.consultar(f":SOURce{canal}:VOLTage?"))
        offset = float(self.consultar(f":SOURce{canal}:VOLTage:OFFSet?"))
        fase = float(self.consultar(f":SOURce{canal}:PHASe?"))
        return {"canal": canal, "salida": salida, "funcion": funcion,
                "frecuencia_hz": frecuencia, "amplitud_vpp": amplitud,
                "offset_v": offset, "fase_grados": fase}

    def obtener_ultimo_error(self) -> str:
        return self.consultar(":SYSTem:ERRor?")