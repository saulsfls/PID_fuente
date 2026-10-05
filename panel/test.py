# ============================================================================
#  SeguidorFrecuenciaFG v1.1 — Sin módulo de estadísticas
#  Setea la frecuencia del generador de funciones Yokogawa FG420
#  en base a la frecuencia medida por el wattmetro Yokogawa WT3000.
#
#  Flujo:
#    WT.frecuencia -> filtro spikes -> filtro exp -> deadband -> rate-limit -> FG
#
#  NO controla fase. Solo frecuencia.
# ============================================================================

import sys
import time
from pathlib import Path
from collections import deque
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000

# ==================== CONFIGURACIÓN ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 400
INTERVALO_MUESTREO = 0.09

# --- Punto de operación ---
FREC_NOMINAL = 60.0             # Hz (centro de referencia)
FASE_FIJA    = 0.0              # grados
AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0

# --- Feedforward de frecuencia ---
FF_ENABLE       = True
FF_GAIN         = 1.00          # FG.freq = NOMINAL + GAIN*(f_wt_filt - NOMINAL)
FF_LIMIT_HZ     = 0.50          # límite de desviación respecto a NOMINAL
ALPHA_FREQ      = 0.10          # filtro exponencial sobre f_wt
N_FREQ_WARMUP   = 5             # muestras iniciales sin actuar

# --- Deadband y rate limit ---
DEADBAND_HZ         = 0.001     # no actualizar FG si |Δ| < 1 mHz
MIN_UPDATE_INTERVAL = 0.30      # segundos mínimos entre updates al FG

# --- Filtro de spikes de frecuencia del WT ---
FP_SPIKE_WINDOW_HZ = 15         # muestras para mediana móvil
FP_SPIKE_JUMP_HZ   = 0.50       # salto permitido respecto a mediana (Hz)
FP_SPIKE_SANITY_HZ = 70.0       # límite duro de frecuencia aceptable

# --- Temporización ---
MAX_OUTLIERS_CONSEC = 4
OUTLIER_JUMP_HZ     = 1.0       # Hz, salto considerado outlier

def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== FILTRO DE SPIKES DE FRECUENCIA ====================
class FiltroSpikesFrec:
    """
    Rechaza picos anómalos de frecuencia del WT usando mediana móvil + sanity.
    Devuelve (f, es_valido). Si es_valido=False, el caller debe usar
    ultimo_valido (evita que un glitch actualice el FG).
    """
    def __init__(self, window=FP_SPIKE_WINDOW_HZ, jump=FP_SPIKE_JUMP_HZ,
                 sanity_max=FP_SPIKE_SANITY_HZ):
        self.buf = deque(maxlen=window)
        self.window = window
        self.jump = jump
        self.sanity_max = sanity_max
        self.n_rejected = 0
        self.ultimo_valido = None

    def filtrar(self, f):
        if f is None:
            return self.ultimo_valido, False
        f_abs = abs(f)
        # Sanity: fuera de rango razonable
        if f_abs > self.sanity_max or f_abs < 1.0:
            self.n_rejected += 1
            return self.ultimo_valido, False
        # Jump vs mediana reciente
        if len(self.buf) >= self.window:
            med = float(np.median(self.buf))
            if abs(f_abs - med) > self.jump:
                self.n_rejected += 1
                return self.ultimo_valido, False
        self.buf.append(f_abs)
        self.ultimo_valido = f_abs
        return f_abs, True


# ==================== SEGUIDOR DE FRECUENCIA ====================
class SeguidorFrecuencia:
    def __init__(self):
        self.f_filt = FREC_NOMINAL
        self.f_cmd_actual = FREC_NOMINAL
        self.n_muestras = 0
        self.n_updates_fg = 0
        self.last_fg_update = 0.0
        self.f_prev_raw = None
        self.n_outliers_consec = 0

    def _filtrar_freq(self, f_wt):
        self.f_filt = (1 - ALPHA_FREQ) * self.f_filt + ALPHA_FREQ * f_wt
        return self.f_filt

    def _es_outlier(self, f_wt):
        if self.f_prev_raw is None:
            self.f_prev_raw = f_wt
            return False
        salto = abs(f_wt - self.f_prev_raw)
        self.f_prev_raw = f_wt
        if salto > OUTLIER_JUMP_HZ:
            self.n_outliers_consec += 1
            return self.n_outliers_consec < MAX_OUTLIERS_CONSEC
        self.n_outliers_consec = 0
        return False

    def actualizar(self, f_wt, dt):
        self.n_muestras += 1

        # 1) Outlier detection
        if self._es_outlier(f_wt):
            return {"f_cmd": self.f_cmd_actual, "cambio": False,
                    "f_filt": self.f_filt, "estado": "outlier"}

        # 2) Warmup
        if self.n_muestras < N_FREQ_WARMUP:
            self._filtrar_freq(f_wt)
            return {"f_cmd": self.f_cmd_actual, "cambio": False,
                    "f_filt": self.f_filt, "estado": "warmup"}

        # 3) Filtro exponencial
        f_filt = self._filtrar_freq(f_wt)

        if not FF_ENABLE:
            return {"f_cmd": self.f_cmd_actual, "cambio": False,
                    "f_filt": f_filt, "estado": "ff_off"}

        # 4) Delta respecto a NOMINAL, limitado
        delta = FF_GAIN * (f_filt - FREC_NOMINAL)
        delta = clamp(delta, -FF_LIMIT_HZ, FF_LIMIT_HZ)
        f_objetivo = FREC_NOMINAL + delta

        # 5) Deadband
        if abs(f_objetivo - self.f_cmd_actual) < DEADBAND_HZ:
            return {"f_cmd": self.f_cmd_actual, "cambio": False,
                    "f_filt": f_filt, "estado": "deadband"}

        # 6) Rate limit
        now = time.time()
        if (now - self.last_fg_update) < MIN_UPDATE_INTERVAL:
            return {"f_cmd": self.f_cmd_actual, "cambio": False,
                    "f_filt": f_filt, "estado": "rate_limit"}

        # 7) Aplicar
        self.f_cmd_actual = f_objetivo
        self.last_fg_update = now
        self.n_updates_fg += 1
        return {"f_cmd": f_objetivo, "cambio": True,
                "f_filt": f_filt, "estado": "update"}


# ==================== MAIN ====================
def main():
    print("=" * 78)
    print(f" SeguidorFrecuenciaFG v1.1 — FG sigue al WT")
    print(f" NOMINAL={FREC_NOMINAL} Hz  |  α={ALPHA_FREQ}  |  "
          f"GAIN={FF_GAIN}  |  LÍMITE=±{FF_LIMIT_HZ} Hz")
    print(f" Deadband={DEADBAND_HZ*1000:.1f} mHz  |  "
          f"Rate limit={MIN_UPDATE_INTERVAL}s  |  "
          f"Spike filter: jump={FP_SPIKE_JUMP_HZ} Hz  sanity={FP_SPIKE_SANITY_HZ} Hz")
    print("=" * 78)

    fg = wt = None
    ctrl = SeguidorFrecuencia()
    filtro_f = FiltroSpikesFrec()
    total = 0
    t_ctrl = None

    # Stats locales mínimas (sin módulo externo)
    f_wt_hist, f_cmd_hist, err_hist = [], [], []

    try:
        # ---------- Conexión ----------
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        # ---------- Configuración inicial ----------
        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=FASE_FIJA, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True,
                   configurar_salida=True)

        print(f"\n{'t (s)':>6} | {'f_wt (Hz)':>10} | {'f_filt':>10} | "
              f"{'f_FG':>10} | {'Δ vs NOM':>10} | {'err':>8} | {'Estado':>11}")
        print("-" * 92)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            t_rel = t_now - t_ctrl

            # ---------- Leer WT ----------
            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO); continue
            if wt.is_outlier():
                continue

            f_med = m.get("frecuencia")
            if f_med is None:
                time.sleep(INTERVALO_MUESTREO); continue

            # ---------- Filtro de spikes ----------
            f_valida, f_valido = filtro_f.filtrar(f_med)
            if not f_valido:
                # Si no hay aún ninguna válida, esperar
                if filtro_f.ultimo_valido is None:
                    time.sleep(INTERVALO_MUESTREO); continue
                # Spike: no actualizamos, seguimos con la última válida
                continue

            f_wt = f_valida
            total += 1

            # ---------- Actualizar seguidor ----------
            res = ctrl.actualizar(f_wt, dt)

            # ---------- Aplicar al FG ----------
            if res["cambio"]:
                try:
                    fg.establecer_frecuencia_extreme(1, res["f_cmd"])
                except Exception as e:
                    print(f"[!] Error seteando frecuencia FG: {e}")

            # ---------- Historial local ----------
            f_wt_hist.append(f_wt)
            f_cmd_hist.append(res["f_cmd"])
            err_hist.append(abs(res["f_cmd"] - f_wt))

            # ---------- Print ----------
            err_mhz = (res["f_cmd"] - f_wt) * 1000
            delta_mhz = (res["f_cmd"] - FREC_NOMINAL) * 1000
            print(f"{int(t_rel):>6} | {f_wt:>10.5f} | {res['f_filt']:>10.5f} | "
                  f"{res['f_cmd']:>10.5f} | {delta_mhz:>+7.2f} mHz | "
                  f"{err_mhz:>+5.1f}mHz | {res['estado']:>11}")

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario (Ctrl+C).")

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback; traceback.print_exc()

    finally:
        # ---------- Resumen final ----------
        print("\n" + "=" * 78)
        print(f" RESUMEN SeguidorFrecuenciaFG v1.1 — NOMINAL={FREC_NOMINAL} Hz")
        print("=" * 78)
        print(f"\n[ Datos Base ]  N={total}  fg_updates={ctrl.n_updates_fg}  "
              f"rechazos_spike={filtro_f.n_rejected}")

        if f_wt_hist:
            print(f"\n[ Frecuencia WT (medida) ]")
            print(f"   media={np.mean(f_wt_hist):.5f} Hz  "
                  f"std={np.std(f_wt_hist)*1000:.2f} mHz  "
                  f"rango=[{np.min(f_wt_hist):.4f}, {np.max(f_wt_hist):.4f}]")

        if f_cmd_hist:
            print(f"\n[ Frecuencia comandada al FG ]")
            print(f"   media={np.mean(f_cmd_hist):.5f} Hz  "
                  f"std={np.std(f_cmd_hist)*1000:.2f} mHz  "
                  f"rango=[{np.min(f_cmd_hist):.4f}, {np.max(f_cmd_hist):.4f}]")
            print(f"   desviación media de NOMINAL: "
                  f"{(np.mean(f_cmd_hist) - FREC_NOMINAL)*1000:+.2f} mHz")

        if err_hist:
            print(f"\n[ Error de seguimiento |f_cmd - f_wt| ]")
            print(f"   med={np.median(err_hist)*1000:.2f} mHz  "
                  f"media={np.mean(err_hist)*1000:.2f} mHz  "
                  f"p90={np.percentile(err_hist, 90)*1000:.2f} mHz  "
                  f"máx={np.max(err_hist)*1000:.2f} mHz")
            pct_ok  = 100.0 * sum(1 for e in err_hist if e < 0.010) / len(err_hist)
            pct_ok5 = 100.0 * sum(1 for e in err_hist if e < 0.005) / len(err_hist)
            print(f"   |err|<10 mHz: {pct_ok:5.1f}%    |err|<5 mHz: {pct_ok5:5.1f}%")

            v = ("EXCELENTE (>=95%)" if pct_ok >= 95 else
                 "BUENO"       if pct_ok >= 80 else
                 "ACEPTABLE"   if pct_ok >= 60 else
                 "NECESITA AJUSTE")
            print(f"\n[ Veredicto ]  {v}  (seguimiento <10 mHz: {pct_ok:.1f}%)")

        print(f"\n[ Estado final ]")
        print(f"   f_filt  = {ctrl.f_filt:.5f} Hz")
        print(f"   f_cmd   = {ctrl.f_cmd_actual:.5f} Hz")
        print(f"   Δf vs NOMINAL = {(ctrl.f_cmd_actual - FREC_NOMINAL)*1000:+.3f} mHz")
        print("=" * 78)

        # ---------- Apagado ----------
        print("\n[!] Apagando...")
        if fg:
            try: fg.establecer_salida(1, False)
            except Exception: pass
            try: fg.desconectar()
            except Exception: pass
        if wt:
            try: wt.desconectar()
            except Exception: pass
        print("[✓] Listo.")


if __name__ == "__main__":
    main()