import time
import math
import statistics as _st
from collections import deque
import pyvisa


# ============================================================================ #
# 1. CONTROLADOR DE INSTRUMENTOS (Yokogawa FG420 & WT3000)
# ============================================================================ #

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

        self._write_times = deque(maxlen=200)
        self._last_write_ms = 0.0
        self._jitter_threshold_ms = 15.0

    def _set_mode_defaults(self, mode: str):
        if mode not in self.MODOS_VALIDOS:
            raise ValueError(f"Modo debe ser uno de {self.MODOS_VALIDOS}")
        self.mode = mode
        if mode in ('extreme', 'streaming'):
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

    def set_mode(self, mode: str, sleep_scale: float = None):
        self._set_mode_defaults(mode.lower())
        if sleep_scale is not None:
            self.sleep_scale = sleep_scale

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

    def _write_raw(self, comando: str):
        t0 = time.perf_counter()
        self.inst.write(comando)
        dt_ms = (time.perf_counter() - t0) * 1e3
        self._last_write_ms = dt_ms
        self._write_times.append(dt_ms)

    def escribir(self, comando: str, opc: bool = None):
        self._write_raw(comando)
        if opc or (self.use_opc and 'FREQ' in comando.upper()):
            self.inst.query("*OPC?")
        if self._write_sleep_base * self._sleep_scale > 0:
            time.sleep(self._write_sleep_base * self._sleep_scale)

    def consultar(self, comando: str) -> str:
        respuesta = self.inst.query(comando).strip()
        if self._query_sleep_base * self._sleep_scale > 0:
            time.sleep(self._query_sleep_base * self._sleep_scale)
        return respuesta

    def limpiar_errores(self):
        self.escribir("*CLS")

    def obtener_idn(self) -> str:
        return self.consultar("*IDN?")

    def _check_canal(self, canal: int):
        if canal not in (1, 2):
            raise ValueError("Canal 1 o 2")

    def extreme(self, canal: int = 1, frecuencia_hz: float = 60.0,
                amplitud_vpp: float = 5.0, offset_v: float = 0.0,
                fase_grados: float = 0.0, encender_salida: bool = True) -> None:
        self.set_mode('extreme', sleep_scale=0.0)
        self._check_canal(canal)
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
        self._check_canal(canal)
        self._write_raw(f":SOURce{canal}:FREQuency {frecuencia_hz:.6f}")


class YokogawaWT3000:
    MODOS_VALIDOS = ('extreme', 'streaming', 'fast', 'balanced', 'precise')

    def __init__(self, resource_address: str, timeout: int = 5000,
                 mode: str = 'balanced', sleep_scale: float = 1.0):
        self.address = resource_address
        self.timeout = timeout
        self.rm = pyvisa.ResourceManager()
        self.inst = None
        self._sleep_scale = float(sleep_scale)
        self._set_mode_defaults(mode.lower())

        self._query_times = deque(maxlen=200)
        self._last_query_ms = 0.0
        self._jitter_threshold_ms = 25.0

        self._salida_config = None
        self._min_items = []

    def _set_mode_defaults(self, mode: str):
        if mode not in self.MODOS_VALIDOS:
            raise ValueError(f"Modo debe ser uno de {self.MODOS_VALIDOS}")
        self.mode = mode
        if mode in ('extreme', 'streaming'):
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

    def set_mode(self, mode: str, sleep_scale: float = None):
        self._set_mode_defaults(mode.lower())
        if sleep_scale is not None:
            self.sleep_scale = sleep_scale

    def _apply_wt_settings(self):
        self.escribir(f":NUMeric:AVERage {self.avg_count}")
        self.escribir(f":INPut:FILTer:LPASs:STATe {1 if self.line_filter else 0}")
        self.escribir(f":INPut:FILTer:HPASs:STATe {1 if self.freq_filter else 0}")
        self.escribir(f":SYNC:SOURce {self.sync_source}")

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

    def _query_raw(self, comando: str) -> str:
        t0 = time.perf_counter()
        respuesta = self.inst.query(comando).strip()
        dt_ms = (time.perf_counter() - t0) * 1e3
        self._last_query_ms = dt_ms
        self._query_times.append(dt_ms)
        return respuesta

    def escribir(self, comando: str):
        self.inst.write(comando)
        if self._write_sleep_base * self._sleep_scale > 0:
            time.sleep(self._write_sleep_base * self._sleep_scale)

    def consultar(self, comando: str) -> str:
        r = self._query_raw(comando)
        if self._query_sleep_base * self._sleep_scale > 0:
            time.sleep(self._query_sleep_base * self._sleep_scale)
        return r

    def limpiar_errores(self):
        self.escribir("*CLS")

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

    def configurar_salida_minima(self, elemento_entrada: int = 1,
                                 incluir_potencias: bool = True):
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

    def extreme(self, elemento_entrada: int = 1,
                incluir_potencias: bool = False,
                configurar_salida: bool = True) -> None:
        self.set_mode('extreme', sleep_scale=0.0)
        if configurar_salida:
            self.configurar_salida_minima(
                elemento_entrada=elemento_entrada,
                incluir_potencias=incluir_potencias,
            )

    def leer_mediciones_minimas(self) -> dict:
        if self._salida_config != "minima":
            raise RuntimeError("Debes configurar la salida mínima antes de leer.")
        data_raw = self._query_raw(":NUMeric:VALue?")
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


# ============================================================================ #
# 2. CONTROLADOR PID Y ESTADÍSTICAS
# ============================================================================ #

class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, output_limits=(-0.05, 0.05)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_out, self.max_out = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

    def compute(self, setpoint: float, measurement: float) -> float:
        now = time.perf_counter()
        if self._last_time is None:
            self._last_time = now
            return 0.0

        dt = now - self._last_time
        if dt <= 0.0:
            return 0.0

        error = setpoint - measurement
        p_term = self.Kp * error

        self._integral += error * dt
        i_term = self.Ki * self._integral

        derivative = (error - self._prev_error) / dt
        d_term = self.Kd * derivative

        output = p_term + i_term + d_term

        # Clamp Anti-Windup
        if output > self.max_out:
            output = self.max_out
            self._integral -= error * dt
        elif output < self.min_out:
            output = self.min_out
            self._integral -= error * dt

        self._prev_error = error
        self._last_time = now
        return output


class EstadisticasFP:
    def __init__(self, fp_tol: float = 0.03):
        self.fp_tol = fp_tol
        self.t_inicio = time.time()
        self.fps = []
        self.fases = []
        self.frecuencias_red = []
        self.tiempos = []
        self.modos_tiempo = {"BUSCAR": 0.0, "AJUSTAR": 0.0, "FIJAR": 0.0}
        self.transiciones = 0
        self.cambios_fase = 0
        self.delta_f_list = []
        self.pruebas_actuador = 0
        self.efectivas_actuador = 0

    def registrar(self, fp: float, fase: float, freq_red: float, modo: str, dt_iter: float, delta_f: float = 0.0):
        t_actual = time.time() - self.t_inicio
        self.tiempos.append(t_actual)
        self.fps.append(fp)
        self.fases.append(fase)
        self.frecuencias_red.append(freq_red)

        if modo in self.modos_tiempo:
            self.modos_tiempo[modo] += dt_iter

        if delta_f != 0.0:
            self.cambios_fase += 1
            self.delta_f_list.append(delta_f)

    def registrar_actuador(self, es_efectivo: bool):
        self.pruebas_actuador += 1
        if es_efectivo:
            self.efectivas_actuador += 1

    def generar_reporte(self) -> str:
        t_total = time.time() - self.t_inicio
        n = len(self.fps)
        if n == 0:
            return "Sin datos registrados."

        fp_mean = _st.mean(self.fps)
        fp_std = _st.stdev(self.fps) if n > 1 else 0.0

        en_rango = sum(1 for fp in self.fps if abs(1.0 - fp) <= self.fp_tol)
        pct_en_rango = (en_rango / n) * 100.0

        histograma = [0] * 10
        for fp in self.fps:
            idx = min(int(fp * 10), 9)
            histograma[idx] += 1

        max_fp_idx = max(range(n), key=lambda i: self.fps[i])
        best_fp = self.fps[max_fp_idx]
        best_fase = self.fases[max_fp_idx]
        best_t = self.tiempos[max_fp_idx]

        f_mean = _st.mean(self.frecuencias_red)
        f_std = _st.stdev(self.frecuencias_red) if n > 1 else 0.0
        f_min, f_max = min(self.frecuencias_red), max(self.frecuencias_red)

        df_mean = _st.mean(self.delta_f_list) if self.delta_f_list else 0.0
        df_max = max((abs(df) for df in self.delta_f_list), default=0.0)

        if pct_en_rango >= 80.0:
            veredicto = f"EXCELENTE ({pct_en_rango:.1f}% en rango)"
        elif pct_en_rango >= 50.0:
            veredicto = f"ACEPTABLE ({pct_en_rango:.1f}% en rango)"
        else:
            veredicto = f"POBRE ({pct_en_rango:.1f}% en rango)"

        def bar(pct, max_chars=30):
            return "#" * int((pct / 100.0) * max_chars)

        t_buscar = self.modos_tiempo['BUSCAR']
        t_ajustar = self.modos_tiempo['AJUSTAR']
        t_fijar = self.modos_tiempo['FIJAR']

        resumen = f"""
==========================================================================================
 RESUMEN FINAL [MODO EXTREME + PID]
==========================================================================================

[ Tiempo y muestras ]
  Tiempo total:             {t_total:.1f} s
  Iteraciones:              {n}
  Tasa efectiva:            {n / t_total:.1f} muestras/s
  FP promedio:              {fp_mean:.4f}
  FP desviación:            {fp_std:.4f}

[ Efectividad ]
  En rango (1±3%):         {en_rango} ({pct_en_rango:.1f}%)

[ Tiempo por modo ]
  BUSCAR        {t_buscar:5.1f}s ({t_buscar/t_total*100:5.1f}%)  {bar(t_buscar/t_total*100)}
  AJUSTAR       {t_ajustar:5.1f}s ({t_ajustar/t_total*100:5.1f}%)  {bar(t_ajustar/t_total*100)}
  FIJAR         {t_fijar:5.1f}s ({t_fijar/t_total*100:5.1f}%)  {bar(t_fijar/t_total*100)}

[ Distribución de FP ]
"""
        for i in range(9, -1, -1):
            low = i / 10.0
            high = (i + 1) / 10.0
            cnt = histograma[i]
            pct = (cnt / n) * 100.0
            resumen += f"  [{low:.1f}-{high:.1f}]   {cnt:4d} ({pct:5.1f}%)  {bar(pct)}\n"

        resumen += f"""
[ Mejor punto ]
  Mejor FP:                 {best_fp:.4f}
  Fase en mejor FP:         {best_fase:+.2f}°
  Alcanzado a los:          {best_t:.1f} s

[ Comportamiento ]
  Transiciones de modo:    {self.transiciones}
  Cambios de fase totales: {self.cambios_fase}
  Δf promedio:              {df_mean:+.5f} Hz
  Δf máximo abs:            {df_max:.5f} Hz

[ Frecuencia de red ]
  Promedio:                 {f_mean:.4f} Hz
  Desviación:               {f_std:.5f} Hz
  Rango global:             {f_min:.4f} - {f_max:.4f} Hz

[ Diagnóstico del actuador ]
  {"✓ Actuador efectivo" if self.efectivas_actuador > (self.pruebas_actuador / 2) else "✗ Actuador poco efectivo"} ({self.efectivas_actuador}/{self.pruebas_actuador} pruebas)

[ Veredicto ]
  {veredicto}
==========================================================================================
"""
        return resumen


# ============================================================================ #
# 3. BUCLE PRINCIPAL DE CONTROL
# ============================================================================ #

def ejecutar_lazo_control(fg: YokogawaFG420, wt: YokogawaWT3000, duracion_s: float = 60.0):
    pid = PIDController(Kp=0.0015, Ki=0.0005, Kd=0.0001, output_limits=(-0.05, 0.05))
    stats = EstadisticasFP(fp_tol=0.03)

    fg.extreme(canal=1, frecuencia_hz=60.0, amplitud_vpp=5.0)
    wt.extreme(elemento_entrada=1, incluir_potencias=True)

    frecuencia_actual_fg = 60.0
    t_inicio = time.perf_counter()
    modo_actual = "BUSCAR"

    print("[✓] Iniciando lazo de control con PID...")

    try:
        while (time.perf_counter() - t_inicio) < duracion_s:
            t0_iter = time.perf_counter()

            mediciones = wt.leer_mediciones_minimas()
            fase_medida = mediciones.get("angulo_fase")
            freq_red = mediciones.get("frecuencia")
            fp = mediciones.get("factor_potencia")

            if fase_medida is None or fp is None or freq_red is None:
                continue

            if abs(1.0 - fp) <= 0.03:
                nuevo_modo = "FIJAR"
            elif abs(1.0 - fp) <= 0.15:
                nuevo_modo = "AJUSTAR"
            else:
                nuevo_modo = "BUSCAR"

            if nuevo_modo != modo_actual:
                stats.transiciones += 1
                modo_actual = nuevo_modo

            correccion_hz = pid.compute(setpoint=0.0, measurement=fase_medida)
            frecuencia_nueva_fg = freq_red + correccion_hz
            delta_f = frecuencia_nueva_fg - frecuencia_actual_fg

            if abs(delta_f) > 1e-5:
                fg.establecer_frecuencia_extreme(canal=1, frecuencia_hz=frecuencia_nueva_fg)
                frecuencia_actual_fg = frecuencia_nueva_fg
                stats.registrar_actuador(es_efectivo=True)
            else:
                stats.registrar_actuador(es_efectivo=False)

            dt_iter = time.perf_counter() - t0_iter

            stats.registrar(
                fp=fp,
                fase=fase_medida,
                freq_red=freq_red,
                modo=modo_actual,
                dt_iter=dt_iter,
                delta_f=delta_f
            )

    except KeyboardInterrupt:
        print("\n[!] Detenido por el usuario.")

    print(stats.generar_reporte())


# ============================================================================ #
# 4. FUNCIÓN MAIN
# ============================================================================ #

def main():
    # --- CONFIGURACIÓN DE DIRECCIONES VISA ---
    # Reemplaza con las direcciones GPIB, USB o TCPIP reales de tus equipos.
    # Puedes usar `rm.list_resources()` para descubrirlas automáticamente.
    ADDRESS_FG420 = "GPIB1::2::INSTR"
    ADDRESS_WT3000 = "GPIB0::1::INSTR"

    print("==================================================")
    print(" Control de Factor de Potencia (FP = 1.0) via PID ")
    print("==================================================")

    fg = YokogawaFG420(resource_address=ADDRESS_FG420, timeout=3000)
    wt = YokogawaWT3000(resource_address=ADDRESS_WT3000, timeout=3000)

    try:
        print("\n[1/3] Conectando con Yokogawa FG420...")
        idn_fg = fg.conectar()
        print(f"      -> Conectado: {idn_fg}")

        print("[2/3] Conectando con Yokogawa WT3000...")
        idn_wt = wt.conectar()
        print(f"      -> Conectado: {idn_wt}")

        print("\n[3/3] Ejecutando lazo de control durante 60 segundos...")
        ejecutar_lazo_control(fg, wt, duracion_s=60.0)

    except pyvisa.errors.VisaIOError as e:
        print(f"\n[ERROR VISA] No se pudo comunicar con un instrumento: {e}")
        print("Revisa la dirección de los instrumentos y las conexiones física/Driver GPIB.")
    except Exception as e:
        print(f"\n[ERROR CRÍTICO] {e}")

    finally:
        print("\n[!] Apagando conexiones de instrumentos...")
        fg.desconectar()
        wt.desconectar()
        print("[✓] Listo.")


if __name__ == "__main__":
    main()