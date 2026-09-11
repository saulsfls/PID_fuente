"""
Controlador mejorado para el analizador de potencia Yokogawa WT3000.

Modos:
    - 'streaming': sleeps a 0. Para el lazo de control (PLL).
                   avg_count=1, sin filtros. Latencia mínima.
    - 'fast'     : sleeps ~2 ms, avg_count=1. Adquisición rápida.
    - 'balanced' : sleeps ~10 ms, avg_count=4. Uso general.
    - 'precise'  : sleeps ~20 ms, avg_count=16. Caracterización offline.

El parámetro sleep_scale multiplica todos los sleeps del modo.
"""
import time
import statistics as _st
from collections import deque
import pyvisa


class YokogawaWT3000:
    MODOS_VALIDOS = ('streaming', 'fast', 'balanced', 'precise')

    # Ítems numéricos usados por la salida mínima del lazo
    # (PHI: ángulo de fase, FU: frecuencia, LAMBda: factor de potencia)
    MIN_ITEMS = (("PHI",), ("FU",), ("LAMBda",), ("P",), ("Q",))

    def __init__(self, resource_address: str, timeout: int = 5000,
                 mode: str = 'balanced', sleep_scale: float = 1.0):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None
        self._sleep_scale = float(sleep_scale)
        self._set_mode_defaults(mode.lower())

        # Estadísticas de I/O
        self._query_times = deque(maxlen=200)
        self._last_query_ms = 0.0
        self._jitter_threshold_ms = 25.0  # p99 aprox observado

        # Flag: modo de salida numérica configurada ('estandar' o 'minima')
        self._salida_config = None

    # ------------------------------------------------------------------ #
    # Configuración de modo
    # ------------------------------------------------------------------ #
    def _set_mode_defaults(self, mode: str):
        if mode not in self.MODOS_VALIDOS:
            raise ValueError(f"Modo debe ser uno de {self.MODOS_VALIDOS}")
        self.mode = mode
        if mode == 'streaming':
            self._write_sleep_base = 0.0
            self._query_sleep_base = 0.0
            self.avg_count = 1
            self.sync_source = "LINE"
            self.line_filter = False
            self.freq_filter = False
        elif mode == 'fast':
            self._write_sleep_base = 0.002
            self._query_sleep_base = 0.002
            self.avg_count = 1
            self.sync_source = "LINE"
            self.line_filter = False
            self.freq_filter = False
        elif mode == 'balanced':
            self._write_sleep_base = 0.010
            self._query_sleep_base = 0.010
            self.avg_count = 4
            self.sync_source = "LINE"
            self.line_filter = True
            self.freq_filter = False
        else:  # precise
            self._write_sleep_base = 0.020
            self._query_sleep_base = 0.020
            self.avg_count = 16
            self.sync_source = "LINE"
            self.line_filter = True
            self.freq_filter = True

        if self.inst:
            self._apply_wt_settings()

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
        self._set_mode_defaults(mode.lower())
        if sleep_scale is not None:
            self.sleep_scale = sleep_scale

    def _apply_wt_settings(self):
        self.escribir(f":NUMeric:AVERage {self.avg_count}")
        self.escribir(f":INPut:FILTer:LPASs:STATe {1 if self.line_filter else 0}")
        self.escribir(f":INPut:FILTer:HPASs:STATe {1 if self.freq_filter else 0}")
        self.escribir(f":SYNC:SOURce {self.sync_source}")

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

    # ------------------------------------------------------------------ #
    # I/O de bajo nivel (con instrumentación)
    # ------------------------------------------------------------------ #
    def _query_raw(self, comando: str) -> str:
        t0 = time.perf_counter()
        respuesta = self.inst.query(comando).strip()
        dt_ms = (time.perf_counter() - t0) * 1e3
        self._last_query_ms = dt_ms
        self._query_times.append(dt_ms)
        return respuesta

    def escribir(self, comando: str):
        self.inst.write(comando)
        if self.write_sleep > 0:
            time.sleep(self.write_sleep)

    def consultar(self, comando: str) -> str:
        r = self._query_raw(comando)
        if self.query_sleep > 0:
            time.sleep(self.query_sleep)
        return r

    # ------------------------------------------------------------------ #
    # Diagnóstico
    # ------------------------------------------------------------------ #
    @property
    def stats(self) -> dict:
        xs = list(self._query_times)
        if not xs:
            return {"n": 0}
        s = sorted(xs)
        n = len(s)
        return {
            "n": n,
            "last_ms": self._last_query_ms,
            "mean_ms": _st.mean(xs),
            "std_ms": _st.stdev(xs) if n > 1 else 0.0,
            "p50_ms": s[int(0.50 * (n - 1))],
            "p95_ms": s[int(0.95 * (n - 1))],
            "p99_ms": s[int(0.99 * (n - 1))],
            "max_ms": s[-1],
        }

    def is_outlier(self) -> bool:
        return self._last_query_ms > self._jitter_threshold_ms

    def reset_stats(self):
        self._query_times.clear()
        self._last_query_ms = 0.0

    # ------------------------------------------------------------------ #
    # Comandos básicos
    # ------------------------------------------------------------------ #
    def limpiar_errores(self):
        self.escribir("*CLS")

    def reset(self):
        self.escribir("*RST")
        time.sleep(0.5)
        self._apply_wt_settings()

    def obtener_idn(self) -> str:
        return self.consultar("*IDN?")

    # ------------------------------------------------------------------ #
    # Parseo
    # ------------------------------------------------------------------ #
    @staticmethod
    def _parse_float(val_str: str):
        try:
            val = float(val_str)
            if val > 1e30 or val < -1e30:
                return None
            return val
        except ValueError:
            return None

    # ------------------------------------------------------------------ #
    # Configuración de salida numérica
    # ------------------------------------------------------------------ #
    def configurar_salida_numerica_estandar(self, elemento_entrada: int = 1):
        """Los 12 parámetros clásicos. Uso general y compatibilidad."""
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
        self._salida_config = "estandar"

    def configurar_salida_minima(self, elemento_entrada: int = 1,
                                 incluir_potencias: bool = True):
        """
        Salida reducida para el lazo: PHI, FU y (opcional) LAMBda, P, Q.
        Menos datos que parsear, respuesta más rápida del WT.
        """
        elem = elemento_entrada
        items = ["PHI", "FU"]
        if incluir_potencias:
            items += ["LAMBda", "P", "Q"]
        self.escribir(":NUMeric:FORMAT ASCII")
        self.escribir(f":NUMeric:NUMBER {len(items)}")
        for i, it in enumerate(items, start=1):
            self.escribir(f":NUMeric:ITEM{i} {it},{elem}")
        self._salida_config = "minima"
        self._min_items = items

    # ------------------------------------------------------------------ #
    # Lectura estándar (compatibilidad)
    # ------------------------------------------------------------------ #
    def leer_mediciones_estandar(self) -> dict:
        data_raw = self._query_raw(":NUMeric:VALue?")
        if self.query_sleep > 0:
            time.sleep(self.query_sleep)
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

    # ------------------------------------------------------------------ #
    # Lectura mínima para el PLL
    # ------------------------------------------------------------------ #
    def leer_mediciones_minimas(self) -> dict:
        """
        Devuelve solo lo que el PLL necesita: ángulo de fase, frecuencia y
        (si se configuró con incluir_potencias=True) LAMBda, P, Q.
        Requiere haber llamado a configurar_salida_minima() antes.
        """
        if self._salida_config != "minima":
            raise RuntimeError(
                "Debes llamar a configurar_salida_minima() antes de usar "
                "leer_mediciones_minimas().")
        data_raw = self._query_raw(":NUMeric:VALue?")
        if self.query_sleep > 0:
            time.sleep(self.query_sleep)
        values = data_raw.split(",")
        out = {}
        for nombre, val in zip(self._min_items, values):
            f = self._parse_float(val)
            key = {
                "PHI": "angulo_fase",
                "FU": "frecuencia",
                "LAMBda": "factor_potencia",
                "P": "potencia_activa",
                "Q": "potencia_reactiva",
            }[nombre]
            out[key] = f
        return out

    def escribir_comando(self, comando: str):
        if self.inst:
            self.inst.write(comando)