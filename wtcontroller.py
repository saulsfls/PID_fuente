import time
import pyvisa


class YokogawaWT3000:
    """Controlador de automatización mediante PyVISA para el analizador de potencia Yokogawa WT3000."""

    def __init__(self, resource_address: str, timeout: int = 5000):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None

    def conectar(self) -> str:
        """Abre la conexión GPIB/VISA con el analizador y configura los delimitadores."""
        self.inst = self.rm.open_resource(self.address)
        self.inst.timeout = self.timeout
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        time.sleep(0.1)
        self.limpiar_errores()
        return self.obtener_idn()

    def desconectar(self):
        """Cierra la conexión con el equipo de forma segura."""
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
        """Envía un comando SCPI al analizador."""
        self.inst.write(comando)
        time.sleep(0.02)

    def consultar(self, comando: str) -> str:
        """Consulta una instrucción SCPI y devuelve la respuesta limpia."""
        respuesta = self.inst.query(comando).strip()
        time.sleep(0.02)
        return respuesta

    def limpiar_errores(self):
        """Limpia los registros de estado (*CLS)."""
        self.escribir("*CLS")

    def reset(self):
        """Restaura la configuración por defecto (*RST)."""
        self.escribir("*RST")
        time.sleep(0.5)

    def obtener_idn(self) -> str:
        """Devuelve la cadena de identificación del equipo."""
        return self.consultar("*IDN?")

    @staticmethod
    def _parse_float(val_str: str):
        """Convierte cadenas a float, convirtiendo lecturas de sobreescala/OVER (>1e30) a None."""
        try:
            val = float(val_str)
            if val > 1e30 or val < -1e30:
                return None
            return val
        except ValueError:
            return None

    def configurar_salida_numerica_estandar(self, elemento_entrada: int = 1):
        """Configura los 12 elementos de medición numéricos estándar para un elemento especificado (por defecto Elemento 1)."""
        elem = elemento_entrada
        self.escribir(":NUMeric:FORMAT ASCII")
        self.escribir(":NUMeric:NUMBER 12")
        self.escribir(f":NUMeric:ITEM1 U,{elem}")        # Voltaje RMS (V)
        self.escribir(f":NUMeric:ITEM2 UMN,{elem}")      # Voltaje Medio (V)
        self.escribir(f":NUMeric:ITEM3 UDC,{elem}")      # Voltaje DC (V)
        self.escribir(f":NUMeric:ITEM4 I,{elem}")        # Corriente RMS (A)
        self.escribir(f":NUMeric:ITEM5 IMN,{elem}")      # Corriente Media (A)
        self.escribir(f":NUMeric:ITEM6 IDC,{elem}")      # Corriente DC (A)
        self.escribir(f":NUMeric:ITEM7 P,{elem}")        # Potencia Activa (W)
        self.escribir(f":NUMeric:ITEM8 S,{elem}")        # Potencia Aparente (VA)
        self.escribir(f":NUMeric:ITEM9 Q,{elem}")        # Potencia Reactiva (VAR)
        self.escribir(f":NUMeric:ITEM10 LAMBda,{elem}")  # Factor de Potencia (PF)
        self.escribir(f":NUMeric:ITEM11 PHI,{elem}")     # Ángulo de Fase (deg)
        self.escribir(f":NUMeric:ITEM12 FU,{elem}")      # Frecuencia de Voltaje (Hz)

    def leer_mediciones_estandar(self) -> dict: 
        """Solicita los datos numéricos configurados y los devuelve parseados en un diccionario."""
        data_raw = self.consultar(":NUMeric:VALue?")
        values = data_raw.split(",")

        if len(values) < 12:
            raise ValueError(f"Respuesta incompleta recibida del equipo: {data_raw}")

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
        """Envía un comando SCPI directo al WT3000."""
        if hasattr(self, 'inst') and self.inst:
            self.inst.write(comando)
        elif hasattr(self, 'instrumento') and self.instrumento:
            self.instrumento.write(comando)
