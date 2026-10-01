"""CONTROL DE FP v12.6 — Parametrizado por data.json

Cambios sobre v12.5:
  1. Normalización de φ al cuadrante del objetivo (elimina ambigüedad del WT)
  2. Trim solo si |slope| ≥ SLOPE_MIN_ABS_TRIM (antes solo umbral de inicio)
  3. Ganancias de trim escalan con K (antes fijas 0.20/0.40)
  4. PHI_TRIM_MAX = 0.75·BANDA (antes 1.0·BANDA)
  5. N_FRESH_COOLDOWN_TRIM = 12 (antes 6)
  6. N_MODE_DWELL = 60 (antes 30)
  7. N_SAME_SIGN_TRIM = 3 (antes 2)
  8. Stats: trims por dirección y descartados por slope bajo
"""

import argparse, json, sys, time, traceback
from collections import deque
from pathlib import Path
import numpy as np

try:
    from controllers.fg420controller import YokogawaFG420
    from controllers.wt3000controller import YokogawaWT3000
    HAVE_HW = True
except Exception as _e:
    print(f"[!] Controladores no disponibles: {_e}")
    YokogawaFG420 = YokogawaWT3000 = None
    HAVE_HW = False

CONFIG_PATH = Path("data.json")

# ==================== UTILS ====================
wrap   = lambda a: ((a + 180.0) % 360.0) - 180.0
clip   = lambda v, lo, hi: max(lo, min(hi, v))
phi_of = lambda fp: float(np.degrees(np.arccos(clip(fp, -1.0, 1.0))))

def banda_phi(fp, tol):
    s = abs(np.sin(np.radians(phi_of(fp))))
    if s > 0.1:
        return (tol / s) * (180.0 / np.pi)
    return float(np.degrees(np.sqrt(max(2.0 * tol, 1e-9))))

def normalizar_phi(phi, phi_obj):
    """Mapea φ al cuadrante de φ_obj preservando |cos(φ)|."""
    phi = wrap(phi)
    if abs(phi_obj) < 80.0 and abs(phi) > 90.0:
        phi = (180.0 - abs(phi)) * (1.0 if phi > 0 else -1.0)
    if phi_obj > 0 and phi < 0:  phi = -phi
    elif phi_obj < 0 and phi > 0: phi = -phi
    return phi

# ==================== CONFIG I/O (solo lectura) ====================
def cargar_config(p):
    with open(p, encoding="utf-8") as f: return json.load(f)

def perfiles(cfg): return list(cfg["perfiles"].keys())

def _tol_de(cfg, perfil):
    c = cfg["comunes"]; p = cfg["perfiles"][perfil]
    fp = float(p["FP"])
    pct = p.get("TOL_FP_PCT", None)
    if pct is None and "TOL_FP_PCT" in c: pct = float(c["TOL_FP_PCT"])
    if pct is not None:
        return abs(fp) * float(pct) / 100.0, float(pct)
    tol = float(p["TOL_FP"])
    return tol, 100.0 * tol / max(abs(fp), 1e-9)

def mostrar_perfiles(cfg):
    print("\n" + "=" * 78)
    print(" Perfiles disponibles en data.json")
    print("=" * 78)
    for i, k in enumerate(perfiles(cfg), 1):
        p = cfg["perfiles"][k]; fp = float(p["FP"])
        tol, tol_pct = _tol_de(cfg, k)
        b = banda_phi(fp, tol)
        print(f"  [{i:>2}] {k:<10} FP={fp:.4f}  φ={phi_of(fp):7.3f}°  "
              f"TOL={tol:.5f} ({tol_pct:.1f}%)  banda_φ={b:6.2f}°  K={p['K']:.3f}  "
              f"learn={'S' if p.get('SLOPE_LEARN') else 'n'}")
        if p.get("_descripcion"):
            print(f"        └─ {p['_descripcion']}")

def seleccionar_perfil(cfg, arg=None):
    ps = perfiles(cfg)
    if arg:
        if arg in cfg["perfiles"]: return arg
        try:
            fp = float(arg)
            for k in ps:
                if abs(float(cfg["perfiles"][k]["FP"]) - fp) < 1e-6: return k
        except ValueError: pass
        raise ValueError(f"Perfil '{arg}' no encontrado.")
    mostrar_perfiles(cfg)
    while True:
        try: sel = input(f"\nSelecciona perfil [1-{len(ps)}] o nombre/FP: ").strip()
        except (EOFError, KeyboardInterrupt): print("\n[!] Cancelado."); sys.exit(0)
        if not sel: continue
        if sel.isdigit() and 1 <= int(sel) <= len(ps): return ps[int(sel)-1]
        if sel in ps: return sel
        try:
            fp = float(sel)
            for k in ps:
                if abs(float(cfg["perfiles"][k]["FP"]) - fp) < 1e-6: return k
        except ValueError: pass
        print("  Entrada inválida.")

# ==================== PARÁMETROS ====================
class Parametros:
    def __init__(self, cfg, perfil):
        c, p = cfg["comunes"], cfg["perfiles"][perfil]
        self.cfg, self.perfil = cfg, perfil

        self.DIR_FG = c["DIR_FG"]; self.DIR_WT = c["DIR_WT"]
        for k in ("ELEMENTO_WT", "SLOPE_SAMPLES_INIT", "STALE_REFRESH_LIMIT",
                  "N_MAX_WAIT_MOVE", "N_SETTLE"):
            setattr(self, k, int(c[k]))
        for k in ("TIEMPO_SEG", "INTERVALO_S", "AMPLITUD_VPP", "FREC_HZ",
                  "FASE_MIN", "FASE_MAX", "DF_MAX", "DF_LP",
                  "EFF_DT_MAX", "SLOPE_MIN_ABS"):
            setattr(self, k, float(c[k]))
        self.STALE_TOL   = float(c.get("STALE_TOL", 0.15))
        self.PHI_ABS_MAX = float(c.get("PHI_ABS_MAX", 179.0))
        self.KP_BASE     = float(c["KP_PLL"])
        self.KI_BASE     = float(c["KI_PLL"])

        self.FP           = float(p["FP"])
        self.K            = float(p["K"])
        self.SLOPE_LEARN  = bool(p["SLOPE_LEARN"])
        self.PLL_DIR      = float(p["PLL_DIR"])
        self.PASO_MAX     = float(p["PASO_MAX"])
        self.OUTLIER_JUMP = float(p["OUTLIER"])
        self.PHI_FILTER   = int(p.get("PHI_FILTER", 1))
        slope_med = p.get("SLOPE_MEDIDO")
        self.SLOPE_INIT = (float(slope_med)
                           if (slope_med is not None and not self.SLOPE_LEARN)
                           else float(p["SLOPE_INIT"]))

        self.TOL_FP, self.TOL_FP_PCT = _tol_de(cfg, perfil)
        self.PHI_OBJETIVO = phi_of(self.FP)
        self.BANDA_PHI    = banda_phi(self.FP, self.TOL_FP)
        self.KP_PLL       = self.KP_BASE * self.K
        self.KI_PLL       = self.KI_BASE * self.K

        self.STALE_FORCE_SEC     = max(self.STALE_REFRESH_LIMIT * self.INTERVALO_S, 0.3)
        self.SLOPE_SETTLE_SEC    = 0.30
        self.MAX_OUTLIERS_CONSEC = 5
        self.DELTA_MIN_MOVER     = 0.2

        B = self.BANDA_PHI
        self.PHI_BUSCAR_ENTER  = clip(6.0 * B, 8.0,  40.0)
        self.PHI_BUSCAR_EXIT   = clip(1.0 * B, 0.8,  12.0)
        self.PHI_TRIM_DEADBAND = clip(0.50 * B, 0.30,  3.0)
        self.PHI_TRIM_MAX      = clip(0.75 * B, 0.25, 10.0)
        self.PHI_BIAS_MAX      = clip(1.5  * B, 0.40,  8.0)
        self.PHI_TRIM_MIN      = 0.03
        # Ganancias de trim escalan con K (referencia K=0.3 → 0.20/0.40)
        kfac = min(1.0, self.K / 0.30)
        self.TRIM_GAIN_FINE    = 0.20 * kfac
        self.TRIM_GAIN_COARSE  = 0.40 * kfac
        self.TRIM_COARSE_UMBRAL = clip(1.5 * B, 0.8, 12.0)
        self.N_FRESH_COOLDOWN_TRIM = 12
        self.N_FRESH_MODE_SWITCH   = 3
        self.N_MODE_DWELL          = 60
        self.N_SAME_SIGN_TRIM      = 3
        self.SLOPE_MIN_ABS_TRIM    = 0.30
        self.KI_BIAS               = 0.15
        self.FP_ESCAPE             = 10.0 * self.TOL_FP
        self.N_FRESH_ESCAPE        = 5
        self.CALIB_PASO            = min(6.0, self.PASO_MAX)

    def resumen(self):
        print("\n--- Parámetros efectivos ---")
        print(f"  Perfil={self.perfil}  FP_obj={self.FP:.4f}  φ_obj={self.PHI_OBJETIVO:.3f}°")
        print(f"  TOL_FP={self.TOL_FP:.5f} ({self.TOL_FP_PCT:.2f}% de FP)  "
              f"banda_φ={self.BANDA_PHI:.3f}°  K={self.K:.3f}")
        print(f"  Kp_eff={self.KP_PLL:.6f}  Ki_eff={self.KI_PLL:.8f}")
        print(f"  SLOPE_INIT={self.SLOPE_INIT:+.3f}  learn={self.SLOPE_LEARN}  "
              f"PLL_DIR={self.PLL_DIR:+.0f}")
        print(f"  PASO_MAX={self.PASO_MAX:.1f}°  OUTLIER={self.OUTLIER_JUMP:.1f}°  "
              f"FILTER={self.PHI_FILTER}  N_SETTLE={self.N_SETTLE}")
        print(f"  PHI_ABS_MAX={self.PHI_ABS_MAX:.1f}°  TRIM_MAX={self.PHI_TRIM_MAX:.2f}°  "
              f"DEADBAND={self.PHI_TRIM_DEADBAND:.2f}°  FP_ESC={self.FP_ESCAPE:.4f}")
        print(f"  TRIM_GAIN_FINE={self.TRIM_GAIN_FINE:.3f}  "
              f"TRIM_GAIN_COARSE={self.TRIM_GAIN_COARSE:.3f}  "
              f"SLOPE_MIN_TRIM={self.SLOPE_MIN_ABS_TRIM:.2f}")
        print(f"  ENTER_BUSCAR={self.PHI_BUSCAR_ENTER:.1f}°  "
              f"EXIT_BUSCAR={self.PHI_BUSCAR_EXIT:.1f}°  DWELL={self.N_MODE_DWELL} frescas")
        print(f"  Dur={self.TIEMPO_SEG:.0f}s  dt={self.INTERVALO_S:.3f}s  "
              f"STALE_FORCE={self.STALE_FORCE_SEC:.2f}s  STALE_TOL={self.STALE_TOL:.3f}")

# ==================== ESTADÍSTICAS ====================
class Stats:
    def __init__(self, par):
        self.par = par
        tol = par.TOL_FP
        self.bandas = [0.25*tol, 0.5*tol, tol, 2*tol, 4*tol]
        self.fp_hist, self.phi_err_hist = [], []
        self.times_mode = {"BUSCAR": 0.0, "PLL": 0.0}
        self.t_total = 0.0
        self.n_total = self.n_fresh = self.n_stale = self.n_outliers = 0
        self.n_trims = 0; self.trims_phi_delta = []
        self.trims_pos = 0; self.trims_neg = 0
        self.n_trims_no_slope = 0     # descartados por |slope| bajo
        self.n_trims_no_sign  = 0     # descartados por falta de firma
        self.n_phi_norm       = 0     # φ normalizado (>90°)
        self.t_in_band     = {b: 0.0 for b in self.bandas}
        self.t_first_band  = {b: None for b in self.bandas}
        self.t_in_band_cur = {b: 0.0 for b in self.bandas}
        self.t_in_band_max = {b: 0.0 for b in self.bandas}
        self.prev_sign = 0; self.n_zerocross = 0

    def registrar(self, t_rel, fp, phi_err, modo, dt):
        self.t_total += dt; self.n_total += 1
        if fp is None: return
        fp_abs = abs(fp); fp_err = abs(fp_abs - self.par.FP)
        self.fp_hist.append(fp_abs); self.phi_err_hist.append(abs(phi_err))
        self.times_mode[modo] = self.times_mode.get(modo, 0.0) + dt
        for b in self.bandas:
            if fp_err <= b:
                self.t_in_band[b] += dt; self.t_in_band_cur[b] += dt
                self.t_in_band_max[b] = max(self.t_in_band_max[b], self.t_in_band_cur[b])
                if self.t_first_band[b] is None: self.t_first_band[b] = t_rel
            else:
                self.t_in_band_cur[b] = 0.0
        s = 1 if phi_err > 0 else (-1 if phi_err < 0 else 0)
        if s and self.prev_sign and s != self.prev_sign: self.n_zerocross += 1
        if s: self.prev_sign = s

    def registrar_trim(self, dphi):
        self.n_trims += 1; self.trims_phi_delta.append(abs(dphi))
        if dphi > 0: self.trims_pos += 1
        elif dphi < 0: self.trims_neg += 1

    def _efectividad(self):
        t = max(self.t_total, 1e-6)
        return 100.0 * self.t_in_band[self.par.TOL_FP] / t

    def resumen(self, ctrl):
        par = self.par
        print("\n" + "=" * 78)
        print(f" RESUMEN — perfil '{par.perfil}'  FP_obj={par.FP}  "
              f"φ_obj={par.PHI_OBJETIVO:.3f}°")
        print("=" * 78)
        if not self.n_total:
            print("[!] Sin datos."); return
        t = max(self.t_total, 1e-6)

        print(f"\n[ Datos ] T={t:.1f}s  N={self.n_total}  tasa={self.n_total/t:.1f}/s  "
              f"frescas={self.n_fresh} ({100*self.n_fresh/self.n_total:.0f}%)  "
              f"stale={self.n_stale}  outliers={self.n_outliers}  "
              f"φ_normalizados={self.n_phi_norm}")

        if self.fp_hist:
            fp = np.array(self.fp_hist); err = np.abs(fp - par.FP)
            print(f"[ FP ] media={fp.mean():.4f} std={fp.std():.4f} "
                  f"min={fp.min():.4f} max={fp.max():.4f}")
            print(f"[ |FP-obj| ] media={err.mean():.5f} mediana={np.median(err):.5f} "
                  f"p90={np.percentile(err,90):.5f} max={err.max():.5f}")

        if self.phi_err_hist:
            pe = np.array(self.phi_err_hist)
            print(f"[ |φ err| ] media={pe.mean():.2f}° mediana={np.median(pe):.2f}° "
                  f"p90={np.percentile(pe,90):.2f}°  banda_obj={par.BANDA_PHI:.3f}°")

        efec = self._efectividad()
        print("\n" + "─" * 78)
        print(f" [ EFECTIVIDAD ]  {efec:6.2f}%   "
              f"(|FP−FP_obj| ≤ {par.TOL_FP:.5f}, i.e. {par.TOL_FP_PCT:.1f}% del objetivo)")
        print("─" * 78)

        print("\n[ Cobertura |FP−FP_obj| ≤ k·TOL ]")
        for b in self.bandas:
            pct = 100*self.t_in_band[b]/t
            rel = b / max(par.TOL_FP, 1e-12)
            t1s = (f"{self.t_first_band[b]:6.1f}s"
                   if self.t_first_band[b] is not None else "  NUNCA")
            print(f"  ≤ {rel:4.2f}·TOL ({b:.5f})  {pct:5.1f}%  1er:{t1s}  "
                  f"racha_max={self.t_in_band_max[b]:6.1f}s  {'#'*int(pct/2)}")

        tm = self.times_mode
        print(f"\n[ Modo ] BUSCAR={tm.get('BUSCAR',0):.1f}s "
              f"({100*tm.get('BUSCAR',0)/t:.0f}%)  "
              f"PLL={tm.get('PLL',0):.1f}s ({100*tm.get('PLL',0)/t:.0f}%)")
        print(f"\n[ Control ] trims={self.n_trims} "
              f"(+{self.trims_pos}/-{self.trims_neg}) "
              f"descartados_slope={self.n_trims_no_slope} "
              f"descartados_signo={self.n_trims_no_sign}  "
              f"zerocross={self.n_zerocross}  "
              f"slope={ctrl.slope:+.3f}({ctrl.slope_samples})  "
              f"bias={ctrl.phi_bias:+.3f}°  mov={ctrl.n_cambios_fase}")
        if self.trims_phi_delta:
            td = np.array(self.trims_phi_delta)
            print(f"[ Trims ] |Δφ| medio={td.mean():.3f}° máx={td.max():.3f}° "
                  f"frec={self.n_trims/t*60:.1f}/min")

        if self.fp_hist:
            print("\n[ Distribución |FP| ]")
            edges = np.linspace(0, max(1.01, par.FP*1.5), 12).tolist()
            fp_arr = np.array(self.fp_hist); tot = len(fp_arr)
            for i in range(len(edges)-1):
                c = int(np.sum((fp_arr >= edges[i]) & (fp_arr < edges[i+1])))
                pct = 100*c/tot
                print(f"  [{edges[i]:.3f}-{edges[i+1]:.3f})  {c:5d} ({pct:5.1f}%) "
                      f"{'#'*int(pct/2)}")

        pb  = efec
        p2b = 100*self.t_in_band[2*par.TOL_FP]/t
        v = ("EXCELENTE" if pb>=70 else "BUENO" if pb>=50 else
             "ACEPTABLE" if p2b>=50 else "POBRE" if p2b>=25 else "FALLO")
        print(f"\n[ Veredicto ] {v}   "
              f"(±TOL: {pb:.1f}% | ±2·TOL: {p2b:.1f}%)")
        print("=" * 78)

# ==================== CONTROLADOR ====================
class ControladorFP:
    def __init__(self, par):
        self.par = par
        self.fase_cmd = 0.0; self.delta_f = 0.0; self.df_integral = 0.0
        self.modo = "BUSCAR"; self.cnt = 0
        self.move_pending = False; self.fresh_after_move = False
        self.phi_prev_raw = None; self.phi_fresh = 0.0
        self.error_fresh = wrap(-par.PHI_OBJETIVO); self.phi_stale = True
        self.slope = par.SLOPE_INIT; self.slope_samples = par.SLOPE_SAMPLES_INIT
        self.fase_prev = None; self.phi_prev = None
        self.dir_buscar = +1.0 if par.PLL_DIR >= 0 else -1.0
        self.n_cambios_fase = 0; self.dt_since_fresh = 0.0
        self.n_enter_consec = 0; self.n_exit_consec = 0
        self.mejor_phi_abs = 180.0; self.fresh_since_trim = 999
        self.calib_done = False; self.move_time = 0.0
        self.last_fresh_time = time.time(); self.n_outliers_consec = 0
        self.phi_bias = 0.0; self.fp_escape_count = 0; self.n_stale_since_move = 0
        self.phi_buf = deque(maxlen=max(1, par.PHI_FILTER))
        self.fresh_since_mode_change = 0
        self.prev_err_sign = 0
        self.same_sign_count = 0

    def _r(self, accion, cambio=False, fresh=False):
        return {"accion": accion, "fase": self.fase_cmd, "delta_f": self.delta_f,
                "frecuencia": self.par.FREC_HZ + self.delta_f,
                "cambio_fase": cambio, "modo": self.modo,
                "phi": self.phi_fresh, "error_fase": self.error_fresh,
                "fresh": fresh, "stale": self.phi_stale, "slope": self.slope or 0.0}

    def _mover(self, d):
        if abs(d) < self.par.DELTA_MIN_MOVER: return False
        self.fase_prev = self.fase_cmd; self.phi_prev = self.phi_fresh
        self.fase_cmd = clip(wrap(self.fase_cmd + d),
                             self.par.FASE_MIN, self.par.FASE_MAX)
        self.n_cambios_fase += 1; self.move_pending = True
        self.fresh_after_move = False; self.move_time = time.time()
        self.cnt = 0; self.n_stale_since_move = 0
        return True

    def _phi(self, phi_raw, stats):
        # Normalizar al cuadrante del objetivo
        phi_norm = normalizar_phi(phi_raw, self.par.PHI_OBJETIVO)
        if abs(phi_raw - phi_norm) > 1e-6:
            stats.n_phi_norm += 1
        self.phi_buf.append(phi_norm)
        if len(self.phi_buf) < self.phi_buf.maxlen: return False
        phi_w = float(np.mean(self.phi_buf)); now = time.time()
        if self.phi_prev_raw is None:
            self.phi_fresh = phi_w
            self.error_fresh = wrap(phi_w - self.par.PHI_OBJETIVO)
            self.phi_prev_raw = phi_w; self.phi_stale = False
            self.last_fresh_time = now; return True
        step_prev = abs(wrap(phi_w - self.phi_prev_raw))
        is_outlier = step_prev > self.par.OUTLIER_JUMP
        if is_outlier:
            stats.n_outliers += 1; self.n_outliers_consec += 1
            if self.n_outliers_consec < self.par.MAX_OUTLIERS_CONSEC:
                self.phi_prev_raw = phi_w; self.phi_stale = False; return False
        else:
            self.n_outliers_consec = 0
        self.phi_prev_raw = phi_w
        step = abs(wrap(phi_w - self.phi_fresh))
        force = (now - self.last_fresh_time) > self.par.STALE_FORCE_SEC
        if not is_outlier and not force and step < self.par.STALE_TOL:
            self.phi_stale = True; return False
        self.phi_fresh = phi_w
        self.error_fresh = wrap(phi_w - self.par.PHI_OBJETIVO)
        self.phi_stale = False; self.last_fresh_time = now
        self.n_outliers_consec = 0
        if self.move_pending: self.fresh_after_move = True
        return True

    def _slope_update(self):
        if not self.par.SLOPE_LEARN or not self.move_pending: return
        if self.fase_prev is None or self.phi_prev is None:
            self.move_pending = False; return
        if time.time() - self.move_time < self.par.SLOPE_SETTLE_SEC: return
        if not self.fresh_after_move: return
        dphi = wrap(self.phi_fresh - self.phi_prev)
        dfase = wrap(self.fase_cmd - self.fase_prev)
        if abs(dfase) < 3.0 or abs(dphi) < 0.3:
            self.move_pending = False; return
        sn = dphi / dfase
        if not (0.05 <= abs(sn) <= 5.0):
            self.move_pending = False; return
        w = max(1.0/(self.slope_samples+1), 0.15)
        self.slope = (1-w)*self.slope + w*sn; self.slope_samples += 1
        self.move_pending = False

    def _cambiar(self, modo):
        if modo == self.modo: return
        self.modo = modo; self.cnt = 0
        self.move_pending = False; self.fresh_after_move = False
        self.n_enter_consec = 0; self.n_exit_consec = 0
        self.fp_escape_count = 0; self.n_stale_since_move = 0
        self.phi_buf.clear(); self.phi_prev_raw = None
        self.df_integral = 0.0; self.delta_f = 0.0
        self.fresh_since_mode_change = 0
        self.same_sign_count = 0
        self.prev_err_sign = 0

    def actualizar(self, fp, phi_med, dt, stats):
        if phi_med is None:
            return self._r("SIN_PHI")
        if abs(phi_med) > self.par.PHI_ABS_MAX:
            stats.n_outliers += 1
            return self._r("PHI>MAX(discard)", fresh=False)

        fresh = self._phi(phi_med, stats)
        self.cnt += 1
        self.dt_since_fresh = 0.0 if fresh else self.dt_since_fresh + dt
        if fresh:
            stats.n_fresh += 1
            self.fresh_since_mode_change += 1
            s = 1 if self.error_fresh > 0 else (-1 if self.error_fresh < 0 else 0)
            if s != 0 and s == self.prev_err_sign:
                self.same_sign_count += 1
            else:
                self.same_sign_count = 1
            if s != 0: self.prev_err_sign = s
        else:
            stats.n_stale += 1
        if fresh and abs(self.error_fresh) < self.mejor_phi_abs:
            self.mejor_phi_abs = abs(self.error_fresh)
        self._slope_update()

        if self.move_pending and not fresh:
            self.n_stale_since_move += 1
            if self.n_stale_since_move > self.par.N_MAX_WAIT_MOVE:
                self.move_pending = False; self._cambiar("BUSCAR")

        if fresh:
            err = abs(self.error_fresh)
            if err > self.par.PHI_BUSCAR_ENTER: self.n_enter_consec += 1
            else:                               self.n_enter_consec = 0
            if err < self.par.PHI_BUSCAR_EXIT:  self.n_exit_consec += 1
            else:                               self.n_exit_consec = 0
            if fp is not None:
                if abs(abs(fp) - self.par.FP) > self.par.FP_ESCAPE:
                    self.fp_escape_count += 1
                else:
                    self.fp_escape_count = 0

        N = self.par.N_FRESH_MODE_SWITCH
        allow_switch = self.fresh_since_mode_change >= self.par.N_MODE_DWELL
        if allow_switch:
            if self.modo == "PLL" and (
                    self.n_enter_consec >= N
                    or self.fp_escape_count >= self.par.N_FRESH_ESCAPE):
                self._cambiar("BUSCAR")
            elif self.modo == "BUSCAR" and self.n_exit_consec >= N:
                self._cambiar("PLL")
        return self._buscar(fresh) if self.modo == "BUSCAR" else self._pll(fresh, stats)

    def _buscar(self, fresh):
        if not fresh: return self._r("BUSCAR-stale")
        if self.cnt < self.par.N_SETTLE: return self._r("BUSCAR-settle")
        self.delta_f = 0.0
        if not self.calib_done:
            d = self.dir_buscar * self.par.CALIB_PASO
            cambio = self._mover(d)
            if cambio: self.calib_done = True
            return self._r(f"CALIB φ{d:+.0f}°", cambio=cambio, fresh=True)
        err = self.error_fresh; slope = self.slope
        if slope and abs(slope) > self.par.SLOPE_MIN_ABS:
            d = -err / slope
            if abs(err) > self.par.PHI_BUSCAR_EXIT and abs(d) < 4.0:
                d = np.sign(d) * 4.0
        else:
            d = self.dir_buscar * self.par.PASO_MAX
        d = clip(d, -self.par.PASO_MAX, self.par.PASO_MAX)
        cambio = self._mover(d)
        return self._r(f"BUSCAR φ{d:+.1f}°", cambio=cambio, fresh=True)

    def _pll(self, fresh, stats):
        par = self.par
        if not fresh:
            return self._r(f"PLL df={self.delta_f:+.5f}", fresh=False)
        s = self.slope if (self.slope and abs(self.slope) > par.SLOPE_MIN_ABS) \
            else par.SLOPE_INIT
        err = self.error_fresh
        self.fresh_since_trim += 1
        eff_dt = max(min(self.dt_since_fresh, par.EFF_DT_MAX), 0.05)

        if 0.1 < abs(err) < par.PHI_TRIM_DEADBAND and self.fresh_since_trim > 1:
            self.phi_bias = clip(self.phi_bias + par.KI_BIAS*err*eff_dt,
                                 -par.PHI_BIAS_MAX, par.PHI_BIAS_MAX)
        else:
            self.phi_bias *= 0.95
        err_eff = err + self.phi_bias

        grande  = abs(err) >= par.TRIM_COARSE_UMBRAL
        min_m   = 1 if grande else par.N_FRESH_COOLDOWN_TRIM
        gan     = par.TRIM_GAIN_COARSE if grande else par.TRIM_GAIN_FINE
        sign_ok = self.same_sign_count >= par.N_SAME_SIGN_TRIM
        slope_ok = abs(s) >= par.SLOPE_MIN_ABS_TRIM

        puede_trim = (abs(err_eff) > par.PHI_TRIM_DEADBAND
                      and self.fresh_since_trim >= min_m)
        if puede_trim and not slope_ok:
            stats.n_trims_no_slope += 1
        elif puede_trim and not sign_ok:
            stats.n_trims_no_sign += 1

        if puede_trim and sign_ok and slope_ok:
            d = clip(-gan*err/s, -par.PHI_TRIM_MAX, par.PHI_TRIM_MAX)
            if abs(d) > par.PHI_TRIM_MIN:
                self._mover(d); self.fresh_since_trim = 0
                self.phi_bias = 0.0; self.same_sign_count = 0
                stats.registrar_trim(d)
                return self._r(f"PLL-trim φ{d:+.2f}°", cambio=True, fresh=True)

        err_int = -err / s
        self.df_integral = clip(self.df_integral + par.KI_PLL*err_int*eff_dt,
                                -par.DF_MAX, par.DF_MAX)
        df_p = clip(par.KP_PLL*err_int, -par.DF_MAX, par.DF_MAX)
        df_out = df_p + self.df_integral
        self.delta_f = clip((1-par.DF_LP)*self.delta_f + par.DF_LP*df_out,
                            -par.DF_MAX, par.DF_MAX)
        return self._r(f"PLL df={self.delta_f:+.5f}", fresh=True)

# ==================== LOGGING ====================
def print_row(t, fp_abs, res):
    mark = "*" if res.get("fresh") else ("-" if res.get("stale") else " ")
    print(f"{t:>4} | {fp_abs:>7.4f} | {res['phi']:>+8.2f} | "
          f"{res['error_fase']:>+7.2f} |{mark}| "
          f"{res['fase']:>+8.2f} | {res['delta_f']:>+9.5f} | "
          f"{res['frecuencia']:>9.4f} | {res['modo']:>7} | "
          f"{res['slope']:>+6.3f} | {res['accion']:>24}")

def banner_tabla():
    print("-" * 124)
    print(f"{'t':>4} | {'|FP|':>7} | {'PHI':>8} | {'errφ':>7} |*| "
          f"{'Fase':>8} | {'Δf':>9} | {'FrecFG':>9} | {'Modo':>7} | "
          f"{'Slope':>6} | {'Acción':>24}")
    print("-" * 124)

# ==================== MAIN ====================
def main():
    ap = argparse.ArgumentParser(description="Control de FP v12.6")
    ap.add_argument("--config", default=str(CONFIG_PATH))
    ap.add_argument("--perfil", default=None,
                    help="Nombre (pf_0_5) o FP (0.5). Si se omite, pregunta.")
    ap.add_argument("--sim", action="store_true",
                    help="Simulación sin hardware.")
    args = ap.parse_args()

    if not Path(args.config).exists():
        print(f"[X] No existe: {args.config}"); sys.exit(1)
    cfg = cargar_config(args.config)
    perfil = seleccionar_perfil(cfg, args.perfil)
    par = Parametros(cfg, perfil)

    print("=" * 78)
    print(f" CONTROL FP v12.6 — perfil '{perfil}'  FP_obj={par.FP}  "
          f"φ_obj={par.PHI_OBJETIVO:.3f}°  TOL={par.TOL_FP:.5f} "
          f"({par.TOL_FP_PCT:.1f}%)")
    print("=" * 78)
    par.resumen()

    ctrl = ControladorFP(par); stats = Stats(par)
    total = 0; fg = wt = None

    def _bucle(leer):
        nonlocal total
        t0 = time.time(); t_prev = t0
        while time.time() - t0 < par.TIEMPO_SEG:
            tn = time.time(); dt = tn - t_prev; t_prev = tn
            data = leer()
            if data is None:
                time.sleep(par.INTERVALO_S); continue
            fp_med, phi_med = data
            fp_abs = (abs(fp_med) if fp_med is not None
                      else abs(np.cos(np.radians(phi_med))))
            total += 1
            res = ctrl.actualizar(fp_med, phi_med, dt, stats)
            if fg is not None:
                fg.establecer_frecuencia_extreme(1, res["frecuencia"])
                if res["cambio_fase"]: fg.establecer_fase(1, res["fase"])
            stats.registrar(tn - t0, fp_abs, res["error_fase"], res["modo"], dt)
            if res.get("fresh") or total % 8 == 0:
                print_row(int(tn - t0), fp_abs, res)
            el = time.time() - tn
            if el < par.INTERVALO_S: time.sleep(par.INTERVALO_S - el)

    try:
        if args.sim:
            print("\n[SIM] Modo simulación — sin hardware.")
            banner_tabla()
            def leer_sim():
                phi_model = par.SLOPE_INIT*ctrl.fase_cmd + np.random.normal(0, 0.15)
                return float(np.cos(np.radians(phi_model))), phi_model
            _bucle(leer_sim)
        else:
            if not HAVE_HW:
                raise RuntimeError("Controladores no disponibles. Usa --sim.")
            print("\n[1/3] Conectando equipos (EXTREME)...")
            fg = YokogawaFG420(par.DIR_FG, mode='extreme')
            wt = YokogawaWT3000(par.DIR_WT, mode='extreme')
            fg.conectar(); wt.conectar()
            print(f"  FG420:  {fg.obtener_idn()[:60]}")
            print(f"  WT3000: {wt.obtener_idn()[:60]}")
            fg.extreme(canal=1, frecuencia_hz=par.FREC_HZ,
                       amplitud_vpp=par.AMPLITUD_VPP, offset_v=0.0,
                       fase_grados=0.0, encender_salida=True)
            wt.extreme(elemento_entrada=par.ELEMENTO_WT,
                       incluir_potencias=True, configurar_salida=True)
            print("  EXTREME OK. Estabilizando 5 s...")
            for _ in range(10): time.sleep(0.5); print(".", end="", flush=True)
            print(" OK")
            print("\n[2/3] INICIANDO CONTROL\n")
            banner_tabla()
            def leer_hw():
                try: m = wt.leer_mediciones_minimas()
                except Exception as e: print(f"[X] {e}"); return None
                if wt.is_outlier(): return None
                phi = m.get("angulo_fase")
                if phi is None or m.get("frecuencia") is None: return None
                return m.get("factor_potencia"), phi
            _bucle(leer_hw)
    except KeyboardInterrupt:
        print("\n\n[!] Detenido por usuario.")
    except Exception as e:
        print(f"\n[X] Error fatal: {e}"); traceback.print_exc()
    finally:
        print("\n[3/3] Apagando...")
        if fg is not None:
            try: fg.establecer_salida(1, False); fg.desconectar()
            except Exception: pass
        if wt is not None:
            try: wt.desconectar()
            except Exception: pass
        stats.resumen(ctrl)
        print("[✓] Listo.")

if __name__ == "__main__":
    main()