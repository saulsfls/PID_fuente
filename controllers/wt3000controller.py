import time
import pyvisa

class YokogawaWT3000:
    """Controlador para el analizador de potencia Yokogawa WT3000.
    Soporta tres modos: 'fast', 'balanced', 'precise'.
    El modo afecta la velocidad de muestreo, el promediado y los filtros.
    """

    def __init__(self, resource_address: str, timeout: int = 5000, mode: str = 'balanced'):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None
        self.mode = mode.lower()
        self._configure_mode()

    def _configure_mode(self):
        """Ajusta tiempos, promediado y filtros según el modo."""
        if self.mode == 'fast':
            self.write_sleep = 0.002      # 2 ms
            self.query_sleep = 0.002
            self.avg_count = 1            # Sin promediado
            self.sync_source = "LINE"     # Sincronización con línea (50/60 Hz) para lectura rápida
            self.line_filter = False
            self.freq_filter = False
        elif self.mode == 'balanced':
            self.write_sleep = 0.01       # 10 ms
            self.query_sleep = 0.01
            self.avg_count = 4
            self.sync_source = "LINE"
            self.line_filter = True
            self.freq_filter = False
        elif self.mode == 'precise':
            self.write_sleep = 0.02       # 20 ms
            self.query_sleep = 0.02
            self.avg_count = 16
            self.sync_source = "LINE"     # o "EXTERNAL" si se desea
            self.line_filter = True
            self.freq_filter = True
        else:
            raise ValueError("Modo debe ser 'fast', 'balanced' o 'precise'")
        # Aplicamos la configuración al instrumento (si ya está conectado)
        if self.inst:
            self._apply_wt_settings()

    def _apply_wt_settings(self):
        """Envía los comandos de configuración al WT3000."""
        # Configurar promediado
        self.escribir(f":NUMeric:AVERage {self.avg_count}")
        # Sincronización (PLL)
        self.escribir(f":INPut:FILTer:LPASs:STATe {1 if self.line_filter else 0}")
        self.escribir(f":INPut:FILTer:HPASs:STATe {1 if self.freq_filter else 0}")
        # El sync source se configura con :SYNC:SOURce LINE|EXTERNAL
        self.escribir(f":SYNC:SOURce {self.sync_source}")

    def set_mode(self, mode: str):
        """Cambia el modo en tiempo real y reconfigura el WT3000."""
        self.mode = mode.lower()
        self._configure_mode()
        if self.inst:
            self._apply_wt_settings()

    def conectar(self) -> str:
        self.inst = self.rm.open_resource(self.address)
        self.inst.timeout = self.timeout
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        time.sleep(0.1)
        self.limpiar_errores()
        # Aplicar configuración inicial según modo
        self._apply_wt_settings()
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

    def escribir(self, comando: str):
        self.inst.write(comando)
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
        self._apply_wt_settings()  # reaplicamos nuestra configuración

    def obtener_idn(self) -> str:
        return self.consultar("*IDN?")

    @staticmethod
    def _parse_float(val_str: str):
        try:
            val = float(val_str)
            if val > 1e30 or val < -1e30:
                return None
            return val
        except ValueError:
            return None

    def configurar_salida_numerica_estandar(self, elemento_entrada: int = 1):
        """Configura los 12 parámetros de medición para un elemento dado."""
        elem = elemento_entrada
        self.escribir(":NUMeric:FORMAT ASCII")
        self.escribir(":NUMeric:NUMBER 12")
        self.escribir(f":NUMeric:ITEM1 U,{elem}")
        self.escribir(f":NUMeric:ITEM2 UMN,{elem}")
        self.escribir(f":NUMeric:ITEM3 UDC,{elem}")
        self.escribir(f":NUMeric:ITEM4 I,{elem}")
        self.escribir(f":NUMeric:ITEM5 IMN,{elem}")
        self.escribir(f":NUMeric:ITEM6 IDC,{elem}")
        self.escribir(f":NUMeric:ITEM7 P,{elem}")
        self.escribir(f":NUMeric:ITEM8 S,{elem}")
        self.escribir(f":NUMeric:ITEM9 Q,{elem}")
        self.escribir(f":NUMeric:ITEM10 LAMBda,{elem}")
        self.escribir(f":NUMeric:ITEM11 PHI,{elem}")
        self.escribir(f":NUMeric:ITEM12 FU,{elem}")

    def leer_mediciones_estandar(self) -> dict:
        """Lee los 12 valores numéricos y los retorna en un diccionario."""
        data_raw = self.consultar(":NUMeric:VALue?")
        values = data_raw.split(",")
        if len(values) < 12:
            raise ValueError(f"Respuesta incompleta: {data_raw}")
        return {
            "voltaje_rms": self._parse_float(values[0]),
            "voltaje_medio": self._parse_float(values[1]),
            "voltaje_dc": self._parse_float(values[2]),
            "corriente_rms": self._parse_float(values[3]),
            "corriente_media": self._parse_float(values[4]),
            "corriente_dc": self._parse_float(values[5]),
            "potencia_activa": self._parse_float(values[6]),
            "potencia_aparente": self._parse_float(values[7]),
            "potencia_reactiva": self._parse_float(values[8]),
            "factor_potencia": self._parse_float(values[9]),
            "angulo_fase": self._parse_float(values[10]),
            "frecuencia": self._parse_float(values[11]),
        }

    def escribir_comando(self, comando: str):
        """Método auxiliar para enviar comandos directos."""
        if self.inst:
            self.inst.write(comando)