import math
import sys
import time
from collections import deque
from pathlib import Path
import numpy as np

# Agregar directorio padre para importar controladores
sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000

# ==================== CONFIGURACION Y CONSTANTES ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.10          # lectura "cruda" del WT3000

# --- OBJETIVO: FP = 0.01 ---
FP_OBJETIVO  = 0.0100
PHI_OBJETIVO = math.degrees(math.acos(FP_OBJETIVO))  # ~ 89.4271 grados

TOLERANCIA_CORTA = 0.0100  # Banda +-0.010 (en FP, solo diagnostico)
TOLERANCIA_FINA  = 0.0020  # Banda +-0.002 (en FP, solo diagnostico)

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = -180.0, 180.0
FREC_NOMINAL = 60.0

DELTA_MIN_MOVER = 0.001
MAX_OUTLIERS_CONSEC = 5
OUTLIER_JUMP = 30.0

SIGNO_LAZO = +1          # cambiar a -1 si el error crece en vez de bajar
DEADZONE_DEG = 0.03       # zona muerta de control, en grados de error de fase

# --- Parametros nuevos v12.18: desacople de tasa de control ---
CONTROL_PERIOD_SEG = 0.5   # el PI solo actualiza el comando cada 0.5 s
SETTLE_SAMPLES = 3         # lecturas crudas descartadas tras cada comando


def envolver_fase(a):
    """Mantiene la fase dentro del rango [-180, 180) grados."""
    return ((a + 180.0) % 360.0) - 180.0


def error_fase(phi_deg):
    """Error de fase con signo, desenvuelto, respecto al objetivo."""
    return envolver_fase(phi_deg - PHI_OBJETIVO)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==================== ESTADISTICAS Y DIAGNOSTICO ====================
class Stats:
    BANDS = [0.002, 0.005, 0.010, 0.020]

    def __init__(self):
        self.fp_hist = []
        self.fp_err_hist = []
        self.phi_err_hist = []
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = 0
        self.n_stale = 0
        self.n_outliers = 0
        self.n_settle_discard = 0
        self.n_trims = 0
        self.trims_phi_delta = []
        self.t_in_band = {b: 0.0 for b in self.BANDS}
        self.t_first_band = {b: None for b in self.BANDS}

    def registrar(self, fp, phi_err, dt):
        self.t_total += dt
        self.n_total += 1
        self.fp_hist.append(fp)
        fp_err = abs(fp - FP_OBJETIVO)
        self.fp_err_hist.append(fp_err)
        self.phi_err_hist.append(abs(phi_err))

        for b in self.BANDS:
            if fp_err <= b:
                self.t_in_band[b] += dt
                if self.t_first_band[b] is None:
                    self.t_first_band[b] = self.t_total

    def registrar_trim(self, dphi):
        if abs(dphi) > DELTA_MIN_MOVER:
            self.n_trims += 1
            self.trims_phi_delta.append(abs(dphi))

    def resumen(self, ctrl):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNOSTICO v12.18-GPIB - Objetivo FP={FP_OBJETIVO:.4f} "
              f"(phi_obj={PHI_OBJETIVO:.4f} grados)")
        print("=" * 78)
        print(f"\n[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  "
              f"frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}  "
              f"descartadas(settle)={self.n_settle_discard}")

        if self.fp_hist:
            media_fp = np.mean(self.fp_hist)
            std_fp = np.std(self.fp_hist)
            sesgo = media_fp - FP_OBJETIVO
            mediana_err = np.median(self.fp_err_hist)
            print(f"[ Control FP ]  media={media_fp:.4f}  std={std_fp:.4f}  "
                  f"sesgo (offset)={sesgo:+.4f}  |FP-{FP_OBJETIVO:.2f}| med={mediana_err:.4f}")

        print(f"\n[ Tiempo en banda |FP-{FP_OBJETIVO:.2f}| ]")
        for b in self.BANDS:
            pct = 100 * self.t_in_band[b] / max(self.t_total, 1e-6)
            t1 = self.t_first_band[b]
            t1_str = f"{t1:6.1f}s" if t1 is not None else "  --  "
            print(f"   +-{b:.3f}   {pct:5.1f}%   1er: {t1_str}   {'#' * int(pct / 2)}")

        pct_t = 100 * self.t_in_band[0.010] / max(self.t_total, 1e-6)
        v = ("EXCELENTE (>=85%)" if pct_t >= 85 else "BUENO" if pct_t >= 50 else "NECESITA AJUSTE")
        print(f"\n[ Veredicto ]  {v}  (+-0.010: {pct_t:.1f}%)")
        dphi_m = np.mean(self.trims_phi_delta) if self.trims_phi_delta else 0.0
        print(f"[ Trims aplicados ]  n={self.n_trims}  |Delta phi| medio={dphi_m:.2f} grados")

        if ctrl is not None:
            print(f"[ Integrador fase ]  integral_err={ctrl.integral_err:+.3f} grados")
            print(f"[ Comando Final ]   fase_cmd={ctrl.fase_cmd:+.3f} grados")
        print("=" * 78)


# ==================== CONTROLADOR CORREGIDO (v12.18) ====================
class ControladorFP:
    """
    Controlador PI(D) de Factor de Potencia.

    - Cierra el lazo con el error de FASE REAL medido (con signo).
    - Muestrea rapido (INTERVALO_MUESTREO) pero solo ACTUALIZA el comando
      de fase cada CONTROL_PERIOD_SEG, usando el promedio de las lecturas
      validas de esa ventana (reduce ruido y desacopla el lazo de control
      de la latencia de asentamiento del instrumento).
    - Tras cada comando, descarta SETTLE_SAMPLES lecturas crudas antes de
      volver a promediar, para no reaccionar sobre un transitorio.
    - Incluye anti-windup, limite de slew rate y termino derivativo para
      amortiguar oscilaciones.
    """
    def __init__(self, target_fp=FP_OBJETIVO, kp=0.35, ki=0.03, kd=0.15,
                 max_trim_deg=0.5, control_period=CONTROL_PERIOD_SEG,
                 settle_samples=SETTLE_SAMPLES):
        self.target_fp = target_fp
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_trim_deg = max_trim_deg
        self.control_period = control_period
        self.settle_samples = settle_samples

        self.fase_cmd = PHI_OBJETIVO
        self.integral_err = 0.0
        self.limite_integral = 4.0  # grados

        self.phi_fresh = PHI_OBJETIVO
        self.error_fresh = 0.0
        self.phi_prev_raw = None
        self.n_outliers_consec = 0

        # Buffer de la ventana de control y control de asentamiento
        self._buffer_err = deque()
        self._t_acum_ventana = 0.0
        self._settle_restantes = 0
        self._err_filt_prev = 0.0
        self._tiene_prev = False

    def _filtrar_lectura(self, phi_raw, stats):
        """Filtro de validez y deteccion de outliers en la lectura de fase.
        Devuelve (es_valida, es_transitorio_post_comando)."""
        phi_w = envolver_fase(phi_raw)

        if self.phi_prev_raw is None:
            self.phi_prev_raw = phi_w
            self.phi_fresh = phi_w
            self.error_fresh = error_fase(phi_w)
            return True

        step_prev = abs(envolver_fase(phi_w - self.phi_prev_raw))
        if step_prev > OUTLIER_JUMP:
            stats.n_outliers += 1
            self.n_outliers_consec += 1
            if self.n_outliers_consec < MAX_OUTLIERS_CONSEC:
                return False
        else:
            self.n_outliers_consec = 0

        self.phi_prev_raw = phi_w
        self.phi_fresh = phi_w
        self.error_fresh = error_fase(phi_w)
        return True

    def actualizar(self, fp_medido, phi_medido, dt, stats):
        if phi_medido is None or fp_medido is None:
            return {
                "accion": "SIN_LECTURA", "fase": self.fase_cmd,
                "frecuencia": FREC_NOMINAL, "cambio_fase": False,
                "modo": "CONTROL_PID", "error_fase": self.error_fresh
            }

        fresh = self._filtrar_lectura(phi_medido, stats)
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1

        if not fresh:
            return {
                "accion": "STALE/OUTLIER", "fase": self.fase_cmd,
                "frecuencia": FREC_NOMINAL, "cambio_fase": False,
                "modo": "CONTROL_PID", "error_fase": self.error_fresh
            }

        # Descarta lecturas del periodo de asentamiento tras el ultimo comando
        if self._settle_restantes > 0:
            self._settle_restantes -= 1
            stats.n_settle_discard += 1
            self._t_acum_ventana += dt
            return {
                "accion": "ASENTANDO", "fase": self.fase_cmd,
                "frecuencia": FREC_NOMINAL, "cambio_fase": False,
                "modo": "CONTROL_PID", "error_fase": self.error_fresh
            }

        # Acumula esta lectura valida en la ventana de control
        self._buffer_err.append(self.error_fresh)
        self._t_acum_ventana += dt

        cambio_fase = False
        accion_str = f"ACUM ({len(self._buffer_err)} muestras)"

        if self._t_acum_ventana >= self.control_period and self._buffer_err:
            err_filt = float(np.mean(self._buffer_err))
            n_ventana = len(self._buffer_err)
            self._buffer_err.clear()
            self._t_acum_ventana = 0.0

            if abs(err_filt) < DEADZONE_DEG:
                delta_phi = 0.0
                accion_str = f"EN_OBJETIVO (n={n_ventana})"
                # deja que el integrador decaiga suavemente en vez de congelarse
                self.integral_err *= 0.9
            else:
                # Anti-windup: solo integra dentro de un rango operativo razonable
                if abs(err_filt) < 20.0:
                    self.integral_err += err_filt * self.control_period * self.ki
                    self.integral_err = clamp(self.integral_err, -self.limite_integral, self.limite_integral)

                # Termino derivativo: amortigua si el error esta oscilando
                if self._tiene_prev:
                    d_err = (err_filt - self._err_filt_prev) / self.control_period
                else:
                    d_err = 0.0

                correccion = self.kp * err_filt + self.integral_err + self.kd * d_err
                delta_phi = -SIGNO_LAZO * correccion
                delta_phi = clamp(delta_phi, -self.max_trim_deg, self.max_trim_deg)
                accion_str = (f"TRIM {delta_phi:+.3f} (n={n_ventana}, err={err_filt:+.3f}, "
                              f"I={self.integral_err:+.3f}, D={d_err:+.3f})")

            self._err_filt_prev = err_filt
            self._tiene_prev = True

            if abs(delta_phi) >= DELTA_MIN_MOVER:
                self.fase_cmd = clamp(envolver_fase(self.fase_cmd + delta_phi), FASE_MIN, FASE_MAX)
                stats.registrar_trim(delta_phi)
                cambio_fase = True
                self._settle_restantes = self.settle_samples

        return {
            "accion": accion_str,
            "fase": self.fase_cmd,
            "frecuencia": FREC_NOMINAL,
            "cambio_fase": cambio_fase,
            "modo": "CONTROL_PID",
            "error_fase": self.error_fresh
        }


# ==================== MAIN BUCLE PRINCIPAL ====================
def main():
    print("=" * 78)
    print(f" CONTROL FP v12.18-GPIB - Objetivo FP={FP_OBJETIVO} "
          f"(phi_obj={PHI_OBJETIVO:.4f} grados)")
    print("=" * 78)

    fg = wt = None
    ctrl = ControladorFP(target_fp=FP_OBJETIVO, kp=0.35, ki=0.03, kd=0.15,
                          max_trim_deg=0.50, control_period=CONTROL_PERIOD_SEG,
                          settle_samples=SETTLE_SAMPLES)
    stats = Stats()
    total = 0
    t_ctrl = None

    try:
        # Inicializacion de hardware via GPIB
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        # Configuracion inicial de instrumentos
        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=ctrl.fase_cmd, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True, configurar_salida=True)

        print(f"\n{'t (s)':>6} | {'FP Medido':>10} | {'|FP-0.01|':>10} | "
              f"{'Modo':>11} | {'Accion':>52}")
        print("-" * 100)

        t_ctrl = t_prev = time.time()

        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now

            # Lectura del medidor WT3000
            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO)
                continue

            if wt.is_outlier():
                continue

            fp_med = m.get("factor_potencia")
            phi_med = m.get("angulo_fase")
            f_med = m.get("frecuencia")

            if phi_med is None or fp_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med)
            total += 1

            # Actualizacion del bucle de control PID (usa phi_med con signo)
            res = ctrl.actualizar(fp_abs, phi_med, dt, stats)

            # Aplicar cambio de fase en el generador FG420 si hubo ajuste
            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], dt)

            print(f"{int(t_now - t_ctrl):>6} | {fp_abs:>10.4f} | "
                  f"{abs(fp_abs - FP_OBJETIVO):>10.4f} | {res['modo']:>11} | {res['accion']:>52}")

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)

    except KeyboardInterrupt:
        print("\nPrueba interrumpida por el usuario.")
        if total > 0 and t_ctrl is not None:
            stats.resumen(ctrl)

    finally:
        if fg:
            fg.desconectar()
        if wt:
            wt.desconectar()


if __name__ == "__main__":
    main()