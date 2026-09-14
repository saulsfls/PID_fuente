"""
Controlador mejorado para el generador de ondas Yokogawa FG420.

Modos:
    - 'extreme'  : sleeps a 0, sin OPC, sin queries. Latencia mínima absoluta.
                   Configuración one-shot + writes crudos en el lazo.
    - 'streaming': sleeps a 0, sin OPC. Para el lazo de control (PLL).
    - 'fast'     : sleeps ~1-2 ms, sin OPC. Adquisición rápida no crítica.
    - 'balanced' : sleeps ~5 ms, sin OPC. Uso general.
    - 'precise'  : sleeps ~30 ms, con OPC. Caracterización offline.

El parámetro sleep_scale (0.0 a 1.0+) multiplica todos los sleeps del modo.
Útil para afinar el compromiso velocidad/robustez sin cambiar de modo.
"""
import time
import statistics as _st
from collections import deque
import pyvisa


class YokogawaFG420:
    MODOS_VALIDOS = ('extreme', 'streaming', 'fast', 'balanced', 'precise')

    def __init__(self, resource_address: str, timeout: int = 5000,
                 mode: str = 'balanced', sleep_scale: float = 1.0):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None
        self._sleep_scale = float(sleep_scale)
        self._set_mode_defaults(mode.lower())

        # Estadísticas de I/O
        self._write_times = deque(maxlen=200)
        self._last_write_ms = 0.0
        self._jitter_threshold_ms = 15.0  # p99 aprox observado

    # ------------------------------------------------------------------ #
    # Configuración de modo
    # ------------------------------------------------------------------ #
    def _set_mode_defaults(self, mode: str):
        if mode not in self.MODOS_VALIDOS:
            raise ValueError(f"Modo debe ser uno de {self.MODOS_VALIDOS}")
        self.mode = mode
        if mode == 'extreme':
            self._write_sleep_base = 0.0
            self._query_sleep_base = 0.0
            self.use_opc = False
            self.use_chaining = True
        elif mode == 'streaming':
            self._write_sleep_base = 0.0
            self._query_sleep_base = 0.0
            self.use_opc = False
            self.use_chaining = True
        elif mode == 'fast':
            self._write_sleep_base = 0.001
            self._query_sleep_base = 0.001
            self.use_opc = False
            self.use_chaining = True
        elif mode == 'balanced':
            self._write_sleep_base = 0.005
            self._query_sleep_base = 0.005
            self.use_opc = False
            self.use_chaining = False
        else:  # precise
            self._write_sleep_base = 0.030
            self._query_sleep_base = 0.030
            self.use_opc = True
            self.use_chaining = False

    @property
    def sleep_scale(self) -> float:
        return self._sleep_scale

    @sleep_scale.setter
    def sleep_scale(self, value: float):
        self._sleep_scale = max(0.0, float(value))

    @property
    def write_sleep(self) -> float:
        return self._write_sleep_base * self._sleep_scale

    @property
    def query_sleep(self) -> float:
        return self._query_sleep_base * self._sleep_scale

    def set_mode(self, mode: str, sleep_scale: float = None):
        """Cambia modo en runtime. Opcionalmente redefine sleep_scale."""
        self._set_mode_defaults(mode.lower())
        if sleep_scale is not None:
            self.sleep_scale = sleep_scale

    # ------------------------------------------------------------------ #
    # Conexión
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # I/O de bajo nivel (con instrumentación)
    # ------------------------------------------------------------------ #
    def _write_raw(self, comando: str):
        """Escritura sin sleep ni OPC. Para el lazo de control."""
        t0 = time.perf_counter()
        self.inst.write(comando)
        dt_ms = (time.perf_counter() - t0) * 1e3
        self._last_write_ms = dt_ms
        self._write_times.append(dt_ms)

    def escribir(self, comando: str, opc: bool = None):
        self._write_raw(comando)
        if opc or (self.use_opc and 'FREQ' in comando.upper()):
            self.inst.query("*OPC?")
        if self.write_sleep > 0:
            time.sleep(self.write_sleep)

    def consultar(self, comando: str) -> str:
        respuesta = self.inst.query(comando).strip()
        if self.query_sleep > 0:
            time.sleep(self.query_sleep)
        return respuesta

    # ------------------------------------------------------------------ #
    # Diagnóstico
    # ------------------------------------------------------------------ #
    @property
    def stats(self) -> dict:
        xs = list(self._write_times)
        if not xs:
            return {"n": 0}
        s = sorted(xs)
        n = len(s)
        return {
            "n": n,
            "last_ms": self._last_write_ms,
            "mean_ms": _st.mean(xs),
            "std_ms": _st.stdev(xs) if n > 1 else 0.0,
            "p50_ms": s[int(0.50 * (n - 1))],
            "p95_ms": s[int(0.95 * (n - 1))],
            "p99_ms": s[int(0.99 * (n - 1))],
            "max_ms": s[-1],
        }

    def is_outlier(self) -> bool:
        """True si la última escritura tuvo jitter anómalo."""
        return self._last_write_ms > self._jitter_threshold_ms

    def reset_stats(self):
        self._write_times.clear()
        self._last_write_ms = 0.0

    # ------------------------------------------------------------------ #
    # Comandos básicos
    # ------------------------------------------------------------------ #
    def limpiar_errores(self):
        self.escribir("*CLS")

    def reset(self):
        self.escribir("*RST")
        time.sleep(0.5)

    def obtener_idn(self) -> str:
        return self.consultar("*IDN?")

    def obtener_ultimo_error(self) -> str:
        return self.consultar(":SYSTem:ERRor?")

    # ------------------------------------------------------------------ #
    # Configuración de canal (uso general)
    # ------------------------------------------------------------------ #
    def _check_canal(self, canal: int):
        if canal not in (1, 2):
            raise ValueError("Canal 1 o 2")

    def establecer_salida(self, canal: int, estado: bool):
        self._check_canal(canal)
        self.escribir(f":OUTPut{canal} {'ON' if estado else 'OFF'}")

    def establecer_forma_onda(self, canal: int, funcion: str):
        self._check_canal(canal)
        self.escribir(f":SOURce{canal}:FUNCtion {funcion.upper()}")

    def establecer_frecuencia(self, canal: int, frecuencia_hz: float):
        self._check_canal(canal)
        self.escribir(f":SOURce{canal}:FREQuency {frecuencia_hz}")

    def establecer_amplitud_vpp(self, canal: int, amplitud_vpp: float):
        self._check_canal(canal)
        self.escribir(f":SOURce{canal}:VOLTage {amplitud_vpp}VPP")

    def establecer_offset(self, canal: int, offset_v: float):
        self._check_canal(canal)
        self.escribir(f":SOURce{canal}:VOLTage:OFFSet {offset_v}V")

    def establecer_fase(self, canal: int, fase_grados: float):
        self._check_canal(canal)
        self.escribir(f":SOURce{canal}:PHASe {fase_grados}")

    def configurar_canal(self, canal: int, funcion: str = "SINusoid",
                         frecuencia_hz: float = 60.0, amplitud_vpp: float = 10.0,
                         offset_v: float = 0.0, fase_grados: float = 0.0,
                         activar_salida: bool = True):
        """Configuración secuencial estándar."""
        if self.use_chaining:
            return self.configurar_canal_rapido(
                canal, funcion, frecuencia_hz, amplitud_vpp,
                offset_v, fase_grados, activar_salida)
        self.establecer_forma_onda(canal, funcion)
        self.establecer_frecuencia(canal, frecuencia_hz)
        self.establecer_amplitud_vpp(canal, amplitud_vpp)
        self.establecer_offset(canal, offset_v)
        self.establecer_fase(canal, fase_grados)
        self.establecer_salida(canal, activar_salida)
        time.sleep(0.01)

    def configurar_canal_rapido(self, canal: int, funcion: str = "SINusoid",
                                frecuencia_hz: float = 60.0, amplitud_vpp: float = 10.0,
                                offset_v: float = 0.0, fase_grados: float = 0.0,
                                activar_salida: bool = True):
        """Todos los parámetros en un único comando SCPI encadenado."""
        self._check_canal(canal)
        on_off = "ON" if activar_salida else "OFF"
        cmd = (f":SOURce{canal}:FUNCtion {funcion.upper()};"
               f":SOURce{canal}:FREQuency {frecuencia_hz};"
               f":SOURce{canal}:VOLTage {amplitud_vpp}VPP;"
               f":SOURce{canal}:VOLTage:OFFSet {offset_v}V;"
               f":SOURce{canal}:PHASe {fase_grados};"
               f":OUTPut{canal} {on_off}")
        self.escribir(cmd, opc=self.use_opc)

    # ------------------------------------------------------------------ #
    # API optimizada para el lazo de control (streaming)
    # ------------------------------------------------------------------ #
    def configurar_canal_streaming(self, canal: int, frecuencia_hz: float = 60.0,
                                   amplitud_vpp: float = 5.0, offset_v: float = 0.0,
                                   fase_grados: float = 0.0):
        """
        Configuración one-shot para el lazo. Un único write, sin sleeps
        ni OPC. Deja la salida encendida.
        """
        self._check_canal(canal)
        cmd = (f":SOURce{canal}:FUNCtion SIN;"
               f":SOURce{canal}:FREQuency {frecuencia_hz:.6f};"
               f":SOURce{canal}:VOLTage {amplitud_vpp:.4f}VPP;"
               f":SOURce{canal}:VOLTage:OFFSet {offset_v:.4f}V;"
               f":SOURce{canal}:PHASe {fase_grados:.4f};"
               f":OUTPut{canal} ON")
        self._write_raw(cmd)

    def establecer_frecuencia_streaming(self, canal: int, frecuencia_hz: float):
        """
        Escritura cruda de frecuencia para el lazo. Sin sleep, sin OPC.
        Es el único comando que debe ejecutarse en el bucle del PLL.
        """
        self._check_canal(canal)
        self._write_raw(f":SOURce{canal}:FREQuency {frecuencia_hz:.6f}")

    # ------------------------------------------------------------------ #
    # Modo EXTREME — latencia mínima absoluta
    # ------------------------------------------------------------------ #
    def extreme(self, canal: int = 1, frecuencia_hz: float = 60.0,
                amplitud_vpp: float = 5.0, offset_v: float = 0.0,
                fase_grados: float = 0.0, encender_salida: bool = True) -> None:
        """
        Lleva el FG420 al límite: latencia mínima absoluta.

        Diferencias con 'streaming':
            - Fuerza sleep_scale = 0.0 (anula cualquier escala previa).
            - Emite *CLS + configuración completa en UN ÚNICO write.
            - No ejecuta ninguna query (ni *IDN?, ni *OPC?, ni estado).
            - Usa :FUNCtion SIN y precisión suficiente para el lazo.

        Pensado para ejecutarse UNA vez al entrar en el lazo de control.
        Después, en cada iteración, llama solo a
        `establecer_frecuencia_extreme()` (o `_streaming`, equivalente).
        """
        # 1) Modo extreme + sleep_scale = 0
        self.set_mode('extreme', sleep_scale=0.0)
        self._check_canal(canal)

        # 2) Todo en un único write (sin sleeps, sin OPC, sin queries)
        on_off = "ON" if encender_salida else "OFF"
        cmd = (f"*CLS;"
               f":SOURce{canal}:FUNCtion SIN;"
               f":SOURce{canal}:FREQuency {frecuencia_hz:.6f};"
               f":SOURce{canal}:VOLTage {amplitud_vpp:.4f}VPP;"
               f":SOURce{canal}:VOLTage:OFFSet {offset_v:.4f}V;"
               f":SOURce{canal}:PHASe {fase_grados:.4f};"
               f":OUTPut{canal} {on_off}")
        self._write_raw(cmd)

    def establecer_frecuencia_extreme(self, canal: int, frecuencia_hz: float) -> None:
        """
        Write crudo de frecuencia para el lazo en modo extreme.
        Sin sleeps, sin OPC, sin queries. Es el único comando que debe
        ejecutarse dentro del bucle del PLL.
        """
        self._check_canal(canal)
        self._write_raw(f":SOURce{canal}:FREQuency {frecuencia_hz:.6f}")

    # ------------------------------------------------------------------ #
    # Estado del canal
    # ------------------------------------------------------------------ #
    def obtener_estado_canal(self, canal: int) -> dict:
        self._check_canal(canal)
        return {
            "canal": canal,
            "salida": self.consultar(f":OUTPut{canal}?"),
            "funcion": self.consultar(f":SOURce{canal}:FUNCtion?"),
            "frecuencia_hz": float(self.consultar(f":SOURce{canal}:FREQuency?")),
            "amplitud_vpp": float(self.consultar(f":SOURce{canal}:VOLTage?")),
            "offset_v": float(self.consultar(f":SOURce{canal}:VOLTage:OFFSet?")),
            "fase_grados": float(self.consultar(f":SOURce{canal}:PHASe?")),
        }