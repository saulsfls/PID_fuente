import pyvisa


class YokogawaFG420:
    MODOS_VALIDOS = ("extreme", "streaming", "fast", "balanced", "precise")

    # Límites de frecuencia del Yokogawa FG420 (1 uHz a 15 MHz)
    FREQ_MIN_HZ = 1e-6
    FREQ_MAX_HZ = 15.0e6

    def __init__(self, resource_address: str = "GPIB1::2::INSTR", mode: str = "extreme"):
        self.resource_address = resource_address
        self.rm = None
        self.instrumento = None
        self.modo = mode
        self.sleep_scale = 0.0 if mode == "extreme" else 0.05

    # ------------------------------------------------------------------ #
    # Gestión de Conexión VISA y Comunicación Cruda
    # ------------------------------------------------------------------ #
    def conectar(self, resource_address: str = None) -> str:
        """Establece comunicación VISA con el FG420."""
        if resource_address:
            self.resource_address = resource_address

        self.rm = pyvisa.ResourceManager()
        self.instrumento = self.rm.open_resource(self.resource_address)
        self.instrumento.timeout = 3000
        return self.consultar("*IDN?")

    def desconectar(self) -> None:
        """Apaga salidas activas y cierra el recurso VISA."""
        if self.instrumento:
            try:
                self.apagar_salidas()
                self.instrumento.close()
            except Exception:
                pass
            self.instrumento = None

    def _write_raw(self, comando: str) -> None:
        """Escritura directa al bus VISA sin demoras."""
        if not self.instrumento:
            raise RuntimeError("El instrumento no está conectado.")
        self.instrumento.write(comando)

    def escribir(self, comando: str) -> None:
        self._write_raw(comando)

    def consultar(self, comando: str) -> str:
        if not self.instrumento:
            raise RuntimeError("El instrumento no está conectado.")
        return self.instrumento.query(comando).strip()
    
    def obtener_idn(self) -> str:
            return self.consultar("*IDN?")

    # ------------------------------------------------------------------ #
    # Modos y Validación de Parámetros
    # ------------------------------------------------------------------ #
    def set_mode(self, mode: str, sleep_scale: float = 0.0) -> None:
        if mode not in self.MODOS_VALIDOS:
            raise ValueError(f"Modo inválido: {mode}. Permitidos: {self.MODOS_VALIDOS}")
        self.modo = mode
        self.sleep_scale = sleep_scale

    def _check_canal(self, canal: int):
        if canal not in (1, 2):
            raise ValueError(f"Canal inválido: {canal}. Debe ser 1 o 2.")

    def _validar_frecuencia(self, frecuencia_hz: float):
        """Garantiza que la frecuencia esté dentro del rango del hardware."""
        if not (self.FREQ_MIN_HZ <= frecuencia_hz <= self.FREQ_MAX_HZ):
            raise ValueError(
                f"Frecuencia {frecuencia_hz} Hz fuera de rango. "
                f"Permitido: [{self.FREQ_MIN_HZ} Hz, {self.FREQ_MAX_HZ} Hz]"
            )

    # ------------------------------------------------------------------ #
    # Métodos de Alto Nivel para el Lazo de Control
    # ------------------------------------------------------------------ #
    def apagar_salidas(self) -> None:
        """Desconecta las salidas de ambos canales por seguridad."""
        self._write_raw(":OUTPut1 OFF;:OUTPut2 OFF")

    def establecer_frecuencia(self, canal: int, frecuencia_hz: float):
        self._check_canal(canal)
        self._validar_frecuencia(frecuencia_hz)
        self.escribir(f":SOURce{canal}:FREQuency {frecuencia_hz}")

    def establecer_frecuencia_streaming(self, canal: int, frecuencia_hz: float):
        """Escritura cruda de frecuencia para el lazo. Sin sleep, sin OPC."""
        self._check_canal(canal)
        self._validar_frecuencia(frecuencia_hz)
        self._write_raw(f":SOURce{canal}:FREQuency {frecuencia_hz:.6f}")

    def establecer_frecuencia_extreme(self, canal: int, frecuencia_hz: float) -> None:
        """Write crudo de frecuencia para el lazo en modo extreme."""
        self._check_canal(canal)
        self._validar_frecuencia(frecuencia_hz)
        self._write_raw(f":SOURce{canal}:FREQuency {frecuencia_hz:.6f}")

    def configurar_canal_streaming(
        self,
        canal: int,
        frecuencia_hz: float = 60.0,
        amplitud_vpp: float = 5.0,
        offset_v: float = 0.0,
        fase_grados: float = 0.0,
    ):
        self._check_canal(canal)
        self._validar_frecuencia(frecuencia_hz)
        cmd = (
            f":SOURce{canal}:FUNCtion SIN;"
            f":SOURce{canal}:FREQuency {frecuencia_hz:.6f};"
            f":SOURce{canal}:VOLTage {amplitud_vpp:.4f}VPP;"
            f":SOURce{canal}:VOLTage:OFFSet {offset_v:.4f}V;"
            f":SOURce{canal}:PHASe {fase_grados:.4f};"
            f":OUTPut{canal} ON"
        )
        self._write_raw(cmd)

    def extreme(
        self,
        canal: int = 1,
        frecuencia_hz: float = 60.0,
        amplitud_vpp: float = 5.0,
        offset_v: float = 0.0,
        fase_grados: float = 0.0,
        encender_salida: bool = True,
    ) -> None:
        self.set_mode("extreme", sleep_scale=0.0)
        self._check_canal(canal)
        self._validar_frecuencia(frecuencia_hz)

        on_off = "ON" if encender_salida else "OFF"
        cmd = (
            f"*CLS;"
            f":SOURce{canal}:FUNCtion SIN;"
            f":SOURce{canal}:FREQuency {frecuencia_hz:.6f};"
            f":SOURce{canal}:VOLTage {amplitud_vpp:.4f}VPP;"
            f":SOURce{canal}:VOLTage:OFFSet {offset_v:.4f}V;"
            f":SOURce{canal}:PHASe {fase_grados:.4f};"
            f":OUTPut{canal} {on_off}"
        )
        self._write_raw(cmd)
        
    def establecer_fase(self, canal: int, fase_grados: float) -> None:
        """Escritura cruda de fase para el lazo (extreme/streaming)."""
        self._check_canal(canal)
        # Envolver a [-180, 180]
        fase_norm = ((fase_grados + 180.0) % 360.0) - 180.0
        self._write_raw(f":SOURce{canal}:PHASe {fase_norm:.4f}")
    