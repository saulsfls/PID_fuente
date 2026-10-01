"""
CONTROL DE FACTOR DE POTENCIA v12.15-000 (Rango Expandido & Búsqueda Antirruido)
--------------------------------------------------------------------------------
Objetivo: FP = 0.0100  (φ_obj ≈ 89.4271°)
Instrumentación: Yokogawa FG420 + Yokogawa WT3000 (vía GPIB)
--------------------------------------------------------------------------------
"""

import math
import sys
import time
from pathlib import Path
import numpy as np

sys.path.append(str(Path(__file__).resolve().parent.parent))

from controllers.fg420controllerv2 import YokogawaFG420
from controllers.wt3000controllerv2 import YokogawaWT3000

# ==================== CONFIGURACIÓN Y CONSTANTES ====================
DIR_FG, DIR_WT = "GPIB1::2::INSTR", "GPIB0::1::INSTR"
ELEMENTO_WT = 1
TIEMPO_PRUEBA_SEG = 300
INTERVALO_MUESTREO = 0.09

FP_OBJETIVO  = 0.01
PHI_OBJETIVO = 89.4271

AMPLITUD_FG, OFFSET_V_FG = 5.0, 0.0
FASE_MIN, FASE_MAX = 0.0, 120.0  # Permite alcanzar 89.4271° sin chocar con topes
FREC_NOMINAL = 60.0

# --- Umbrales BUSCAR <-> PLL ---
PHI_BUSCAR_ENTER, PHI_BUSCAR_EXIT = 8.0, 2.5
PASO_BUSCAR_DEG = 0.80
N_SETTLE_BUSCAR = 2
BUSCAR_TIMEOUT_SEC = 8.0
FLIP_CONFIRM_COUNT = 4      # Confirmación estricta de 4 muestras
FLIP_MIN_DELTA_DEG = 0.20    # Ignora ruido menor a 0.20°

# --- Control PI / PLL ---
KP_PHI_VEL = 0.35
KI_PHI_VEL = 0.80
INTEGRAL_CMD_LIMIT   = 12.00
INTEGRAL_ENABLE_BAND = 20.00

# --- Filtro y Tolerancias ---
ERR_FILT_ALPHA = 0.35
STALE_TOL, STALE_FORCE_SEC = 0.05, 0.3
MAX_OUTLIERS_CONSEC = 5

PHI_TRIM_DEADBAND = 0.06
PHI_TRIM_MAX      = 1.20
PHI_TRIM_MIN      = 0.02
N_FRESH_COOLDOWN_TRIM = 4

OUTLIER_JUMP = 30.0
DELTA_MIN_MOVER = 0.02

def envolver_fase(a): return ((a + 180.0) % 360.0) - 180.0
def error_fase(phi_deg): return envolver_fase(phi_deg - PHI_OBJETIVO)
def clamp(v, lo, hi): return max(lo, min(hi, v))


# ==================== ESTADÍSTICAS AVANZADAS ====================
class StatsAvanzadas:
    BANDS = [0.002, 0.005, 0.010, 0.020]

    def __init__(self):
        self.fp_hist, self.phi_err_hist = [], []
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = 0
        self.n_fresh = self.n_stale = self.n_outliers = 0
        
        self.trims_phi_delta = []
        self.flips_count = 0
        self.t_in_band = {b: 0.0 for b in self.BANDS}
        self.first_time_in_band = {b: None for b in self.BANDS}

    def registrar(self, fp, phi_err, modo, dt):
        self.t_total += dt
        self.n_total += 1
        self.fp_hist.append(fp)
        fp_err = abs(fp - FP_OBJETIVO)
        self.phi_err_hist.append(phi_err)
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt

        for b in self.BANDS:
            if fp_err <= b:
                self.t_in_band[b] += dt
                if self.first_time_in_band[b] is None:
                    self.first_time_in_band[b] = self.t_total

    def registrar_trim(self, dphi):
        self.trims_phi_delta.append(dphi)

    def resumen(self, ctrl):
        print("\n" + "=" * 78)
        print(f" RESUMEN DE DIAGNÓSTICO v12.15-000 — Objetivo FP={FP_OBJETIVO}")
        print("=" * 78)
        print(f"[ Datos Base ]  T={self.t_total:.1f}s  N={self.n_total}  frescas={self.n_fresh}  stale={self.n_stale}  outliers={self.n_outliers}")
        
        if self.fp_hist:
            fp_arr = np.array(self.fp_hist)
            media_fp, std_fp = np.mean(fp_arr), np.std(fp_arr)
            mediana_err = np.median(np.abs(fp_arr - FP_OBJETIVO))
            print(f"[ Control FP ]  media={media_fp:.4f}  std={std_fp:.4f}  sesgo={media_fp - FP_OBJETIVO:+.4f}  |FP-0.01| med={mediana_err:.4f}")

        print(f"[ Distribución Modos ] BUSCAR={self.times_mode.get('BUSCAR',0):.1f}s ({(self.times_mode.get('BUSCAR',0)/max(self.t_total,0.1))*100:.1f}%) | PLL={self.times_mode.get('PLL',0):.1f}s ({(self.times_mode.get('PLL',0)/max(self.t_total,0.1))*100:.1f}%)")
        print(f"[ Inversiones Búsqueda ] Flips confirmados: {self.flips_count}")

        print("[ Tiempo en Banda FP ]")
        for b in self.BANDS:
            pct = (self.t_in_band[b] / max(self.t_total, 0.1)) * 100
            t_1st = f"{self.first_time_in_band[b]:.1f}s" if self.first_time_in_band[b] is not None else "--"
            bar = "#" * int(pct / 4)
            print(f"   ±{b:.3f} : {pct:5.1f}%   1er: {t_1st:>5s}   {bar}")

        n_trims = len(self.trims_phi_delta)
        mean_trim = np.mean(np.abs(self.trims_phi_delta)) if n_trims > 0 else 0.0

        print(f"[ Estado Interno ]")
        print(f"   Trims Aplicados: n={n_trims} | |Δφ| promedio={mean_trim:.2f}°")
        print(f"   Pendiente Nominal: s={ctrl.slope:+.3f}")
        print(f"   Integrador Acumulado: integral_cmd={ctrl.integral_cmd:+.3f}°")
        print(f"   Comando Fase Final: fase_cmd={ctrl.fase_cmd:.3f}°")
        print("=" * 78)


# ==================== CONTROLADOR OPTIMIZADO ====================
class ControladorFP:
    def __init__(self):
        self.fase_cmd = 85.0  # Punto inicial cercano a 89.4271°
        self.modo = "BUSCAR"
        self.cnt = 0
        self.move_pending = False
        self.phi_prev_raw = None
        self.phi_fresh = 0.0
        self.error_fresh = error_fase(85.0)
        self.error_filt  = error_fase(85.0)
        self.phi_stale = True
        
        self.slope = -0.50
        self.last_fresh_time = time.time()
        self.n_outliers_consec = 0
        self.cooldown_trim = 0
        self.integral_cmd = 0.0

        # BUSCAR con antirruido
        self.buscar_dir = 1
        self.buscar_prev_err_abs = None
        self.buscar_start_time = time.time()
        self.bad_steps_count = 0

    def _r(self, accion, cambio_fase=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd,
                "cambio_fase": cambio_fase, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale}

    def _mover_fase(self, delta):
        if abs(delta) < DELTA_MIN_MOVER: return False
        self.fase_cmd = clamp(self.fase_cmd + delta, FASE_MIN, FASE_MAX)
        self.move_pending = True
        self.cnt = 0
        return True

    def _phi(self, phi_raw, stats):
        phi_w = envolver_fase(phi_raw)
        now = time.time()
        if self.phi_prev_raw is None:
            self.phi_fresh, self.error_fresh = phi_w, error_fase(phi_w)
            self.error_filt = self.error_fresh
            self.phi_prev_raw = phi_w
            self.phi_stale = False
            self.last_fresh_time = now
            return True

        step_prev = abs(envolver_fase(phi_w - self.phi_prev_raw))
        if step_prev > OUTLIER_JUMP:
            stats.n_outliers += 1
            self.n_outliers_consec += 1
            if self.n_outliers_consec < MAX_OUTLIERS_CONSEC:
                self.phi_prev_raw = phi_w
                self.phi_stale = False
                return False
        else:
            self.n_outliers_consec = 0

        self.phi_prev_raw = phi_w
        step_fresh = abs(envolver_fase(phi_w - self.phi_fresh))
        force_fresh = (now - self.last_fresh_time) > STALE_FORCE_SEC
        
        if step_fresh < STALE_TOL and not force_fresh:
            self.phi_stale = True
            return False

        self.phi_fresh, self.error_fresh = phi_w, error_fase(phi_w)
        self.error_filt = ERR_FILT_ALPHA * self.error_fresh + (1 - ERR_FILT_ALPHA) * self.error_filt
        self.phi_stale = False
        self.last_fresh_time = now
        return True

    def _cambiar(self, modo):
        if modo == self.modo: return
        self.modo = modo
        self.cnt = 0
        self.move_pending = False

        if modo == "BUSCAR":
            self.buscar_prev_err_abs = None
            self.buscar_start_time = time.time()
            self.bad_steps_count = 0
        elif modo == "PLL":
            self.error_filt = self.error_fresh

    def actualizar(self, fp, phi_med, dt, stats):
        if phi_med is None: return self._r("SIN_PHI")
        fresh = self._phi(phi_med, stats)
        self.cnt += 1
        stats.n_fresh += 1 if fresh else 0
        stats.n_stale += 0 if fresh else 1

        if fresh: self.cooldown_trim = max(0, self.cooldown_trim - 1)

        if self.modo == "PLL" and abs(self.error_fresh) > PHI_BUSCAR_ENTER:
            self._cambiar("BUSCAR")
        elif self.modo == "BUSCAR" and abs(self.error_fresh) < PHI_BUSCAR_EXIT:
            self._cambiar("PLL")

        return self._buscar(fresh, stats) if self.modo == "BUSCAR" else self._pll(fresh, dt, stats)

    def _buscar(self, fresh, stats):
        if not fresh: return self._r("BUSCAR-stale")

        if time.time() - self.buscar_start_time > BUSCAR_TIMEOUT_SEC:
            self._cambiar("PLL")
            return self._r("BUSCAR-timeout → PLL")

        if self.cnt < N_SETTLE_BUSCAR: return self._r("BUSCAR-settle")

        err = abs(self.error_fresh)

        if self.buscar_prev_err_abs is None:
            self.buscar_prev_err_abs = err
            d = self.buscar_dir * PASO_BUSCAR_DEG
            cambio = self._mover_fase(d)
            return self._r(f"BUSCAR φ{d:+.1f}° (init)", cambio_fase=cambio, fresh=True)

        delta_err = err - self.buscar_prev_err_abs

        if delta_err > FLIP_MIN_DELTA_DEG:
            self.bad_steps_count += 1
            if self.bad_steps_count >= FLIP_CONFIRM_COUNT:
                self.buscar_dir *= -1
                self.bad_steps_count = 0
                stats.flips_count += 1
        else:
            self.bad_steps_count = 0

        self.buscar_prev_err_abs = err
        d = self.buscar_dir * PASO_BUSCAR_DEG
        cambio = self._mover_fase(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio_fase=cambio, fresh=True)

    def _pll(self, fresh, dt, stats):
        if not fresh:
            return self._r(f"PLL ef={self.error_filt:+.2f}°", fresh=False)

        err_f = self.error_filt
        s = self.slope if abs(self.slope) > 0.05 else -0.50
        e = -err_f / s

        if abs(err_f) < INTEGRAL_ENABLE_BAND:
            self.integral_cmd = clamp(
                self.integral_cmd + KI_PHI_VEL * e * dt, 
                -INTEGRAL_CMD_LIMIT, 
                INTEGRAL_CMD_LIMIT
            )

        dP = KP_PHI_VEL * e
        d = clamp(dP + self.integral_cmd, -PHI_TRIM_MAX, PHI_TRIM_MAX)

        if abs(err_f) > PHI_TRIM_DEADBAND and self.cooldown_trim == 0:
            if abs(d) > PHI_TRIM_MIN:
                self._mover_fase(d)
                stats.registrar_trim(d)
                self.cooldown_trim = N_FRESH_COOLDOWN_TRIM
                return self._r(f"PLL φ{d:+.2f}°", cambio_fase=True, fresh=True)

        return self._r(f"PLL ef={self.error_filt:+.2f}° (I={self.integral_cmd:+.2f})", fresh=True)


# ==================== MAIN ====================
def main():
    fg = wt = None
    ctrl = ControladorFP()
    stats = StatsAvanzadas()

    try:
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        fg.conectar()
        wt.conectar()

        fg.extreme(canal=1, frecuencia_hz=FREC_NOMINAL, amplitud_vpp=AMPLITUD_FG,
                   offset_v=OFFSET_V_FG, fase_grados=ctrl.fase_cmd, encender_salida=True)
        wt.extreme(elemento_entrada=ELEMENTO_WT, incluir_potencias=True, configurar_salida=True)

        t_ctrl = t_prev = time.time()
        while time.time() - t_ctrl < TIEMPO_PRUEBA_SEG:
            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now

            try:
                m = wt.leer_mediciones_minimas()
            except Exception:
                time.sleep(INTERVALO_MUESTREO)
                continue

            if wt.is_outlier(): continue
            fp_med, phi_med = m.get("factor_potencia"), m.get("angulo_fase")
            if phi_med is None:
                time.sleep(INTERVALO_MUESTREO)
                continue

            fp_abs = abs(fp_med) if fp_med is not None else abs(np.cos(np.radians(phi_med)))
            res = ctrl.actualizar(fp_abs, phi_med, dt, stats)

            if res["cambio_fase"]:
                fg.establecer_fase(1, res["fase"])

            stats.registrar(fp_abs, res["error_fase"], res["modo"], dt)

            elapsed = time.time() - t_now
            if elapsed < INTERVALO_MUESTREO:
                time.sleep(INTERVALO_MUESTREO - elapsed)

        stats.resumen(ctrl)
    except KeyboardInterrupt:
        stats.resumen(ctrl)
    finally:
        if fg: fg.desconectar()
        if wt: wt.desconectar()

if __name__ == "__main__":
    main()