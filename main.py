"""Control de FP compacto — perfiles definidos en data.json."""
"""Control V2 main"""
import json, time, sys, argparse
import numpy as np
from collections import deque
from math import degrees, acos
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000


# ----------------------------- utilidades -----------------------------
def wrap(a):  return ((a + 180.0) % 360.0) - 180.0
def clamp(v, lo, hi):  return max(lo, min(hi, v))


def derivar(p):
    """Expande el perfil compacto al conjunto completo de parámetros."""
    fp = float(p["FP"]); phi = degrees(acos(fp))
    tol = float(p["TOL_FP"]); k = float(p["K"]); pmax = float(p["PASO_MAX"])
    return {
        "FP": fp, "PHI": phi, "TOL_FP": tol,
        "FP_MIN": fp - 3 * tol, "FP_MAX": fp + 3 * tol,
        "PHI_ENTER": float(p["PHI_ENTER"]),
        "PHI_EXIT":  float(p["PHI_EXIT"]),
        "PHI_LOCK":  float(p["PHI_EXIT"]) / 3.0,
        "PHI_FAR":   float(p["PHI_ENTER"]) * 0.5,
        "PASO_INI":  pmax * 0.5, "PASO_MAX": pmax,
        "PASO_FAR":  pmax * 0.25, "PASO_NEAR": pmax * 0.02,
        "KP": 0.0025 * k, "KI": 0.00030 * k, "DF_MAX": 0.15,
        "OUTLIER": float(p["OUTLIER"]),
        "PHI_ABS_MAX": 89.99, "STALE_TOL": 0.02,
        "SLOPE_INIT": -0.8, "SLOPE_MIN": 0.20,
        "PHI_FILTER": int(p.get("PHI_FILTER", 1)),
        "DF_LP":      float(p.get("DF_LP", 0.30)),
    }


# ----------------------------- controlador -----------------------------
class Ctrl:
    def __init__(self, c):
        self.c = c
        self.fase = 0.0; self.df = 0.0; self.dfi = 0.0
        self.modo = "BUSCAR"; self.cnt = 0
        self.mov = False; self.post = False
        self.praw = None; self.phi = 0.0; self.stale = 0
        self.slope = c["SLOPE_INIT"]; self.slope_n = 20
        self.fprev = None; self.pprev = None
        self.paso = c["PASO_INI"]; self.n_cambios = 0; self.dt_since = 0.0
        self.enter = 0; self.exit = 0
        self.mejor_err = 180.0; self.mejor_fase = 0.0; self.mejor_fp = 0.0
        self.buf = deque(maxlen=c["PHI_FILTER"])

    def _err(self, phi):  return wrap(phi - self.c["PHI"])

    def _phi_ctl(self):
        if not self.buf:  return self.phi
        return sum(self.buf) / len(self.buf)

    def _mover(self, d):
        if abs(d) < 0.02:  return False
        self.fprev = self.fase; self.pprev = self.phi
        self.fase = clamp(wrap(self.fase + d), -180.0, 180.0)
        self.n_cambios += 1; self.mov = True; self.post = False; self.cnt = 0
        return True

    def _upd_phi(self, raw):
        p = wrap(raw)
        if self.praw is None:
            self.phi = p; self.praw = p; self.stale = 0
            self.buf.append(self.phi); return True
        if abs(p - self.praw) < self.c["STALE_TOL"]:
            self.stale += 1
            if (not self.mov and self.stale >= 20) or (self.mov and self.stale >= 25):
                self.phi = p; self.praw = p; self.stale = 0
                self.mov = False; self.post = False
                self.buf.append(self.phi); return True
            return False
        if abs(wrap(p - self.phi)) > self.c["OUTLIER"]:
            self.praw = p; return False
        self.phi = p; self.praw = p; self.stale = 0
        self.buf.append(self.phi)
        if self.mov:  self.post = True
        return True

    def _upd_slope(self):
        if not self.mov or not self.post:  return
        if self.fprev is None or self.pprev is None:
            self.mov = False; return
        dphi = wrap(self.phi - self.pprev)
        dfase = wrap(self.fase - self.fprev)
        if abs(dfase) < 0.05 or abs(dphi) < 0.02:
            self.mov = False; return
        s = dphi / dfase
        if 0.05 < abs(s) < 5.0:
            w = max(1.0 / (self.slope_n + 1), 0.15)
            self.slope = (1 - w) * self.slope + w * s; self.slope_n += 1
        self.mov = False

    def _modo(self, m):
        if m == self.modo:  return
        self.modo = m; self.cnt = 0; self.mov = False; self.post = False
        self.enter = 0; self.exit = 0

    def update(self, fp_abs, phi_raw, dt):
        fresco = self._upd_phi(phi_raw)
        self.cnt += 1; self.dt_since += dt
        if fresco:  self.dt_since = 0.0
        err = self._err(self.phi)
        if fresco and abs(err) < self.mejor_err:
            self.mejor_err = abs(err); self.mejor_fase = self.fase; self.mejor_fp = fp_abs
        self._upd_slope()

        usable = fresco or (not self.mov and self.stale == 0)
        if usable:
            self.enter = self.enter + 1 if abs(err) > self.c["PHI_ENTER"] else 0
            self.exit  = self.exit  + 1 if abs(err) < self.c["PHI_EXIT"]  else 0
        if self.modo == "PLL" and self.enter >= 3:      self._modo("BUSCAR")
        elif self.modo == "BUSCAR" and self.exit >= 2:  self._modo("PLL")
        return self._buscar(fresco) if self.modo == "BUSCAR" else self._pll(fresco)

    def _buscar(self, fresco):
        if self.cnt < 6:                       return self._resp("settle", fresco)
        if self.mov and not self.post:         return self._resp("espera", fresco)
        self.df = 0.0
        err = self._err(self._phi_ctl()); pabs = abs(self.phi)

        # Recuperación: arrancamos fuera del rango operativo (>92°)
        if pabs > 92.0:
            s = self.slope if abs(self.slope) > self.c["SLOPE_MIN"] else self.c["SLOPE_INIT"]
            d = clamp(-err / s, -15.0, 15.0)
            self.paso = abs(d)
            return self._resp(f"RECUP φ{d:+.2f}°", fresco, self._mover(d))

        if abs(self.slope) > self.c["SLOPE_MIN"]:
            d = -err / self.slope
            pmin = self.c["PASO_FAR"] if abs(err) > self.c["PHI_FAR"] else self.c["PASO_NEAR"]
            d = (1 if d >= 0 else -1) * max(abs(d), pmin)
        else:
            d = self.c["PASO_INI"]
        d = clamp(d, -self.c["PASO_MAX"], self.c["PASO_MAX"])

        # Guarda anti-vértice: sólo si ya estamos dentro
        if pabs <= self.c["PHI_ABS_MAX"]:
            s = self.slope if abs(self.slope) > self.c["SLOPE_MIN"] else self.c["SLOPE_INIT"]
            if abs(wrap(self.phi + s * d)) > self.c["PHI_ABS_MAX"]:
                margen = self.c["PHI_ABS_MAX"] - pabs
                if margen <= 0.02:
                    self.paso = 0.0
                    return self._resp("bloq", fresco, False)
                d = (1 if d >= 0 else -1) * min(abs(d), margen / abs(s))

        self.paso = abs(d)
        return self._resp(f"φ{d:+.3f}°", fresco, self._mover(d))

    def _pll(self, fresco):
        if fresco:
            edt = max(min(self.dt_since, 0.6), 0.05)
            s = self.slope if abs(self.slope) > self.c["SLOPE_MIN"] else self.c["SLOPE_INIT"]
            e = -self._err(self._phi_ctl()) / s
            m = self.c["DF_MAX"]
            self.dfi = clamp(self.dfi + self.c["KI"] * e * edt, -m, m)
            dfp = clamp(self.c["KP"] * e, -m, m)
            a = self.c["DF_LP"]
            self.df = clamp((1 - a) * self.df + a * (dfp + self.dfi), -m, m)
        return self._resp(f"df={self.df:+.6f}", fresco, False)

    def _resp(self, accion, fresco, cambio=False):
        return {"accion": accion, "fase": self.fase, "df": self.df,
                "frec": 60.0 + self.df, "cambio": cambio, "modo": self.modo,
                "phi": self.phi, "phi_f": self._phi_ctl(),
                "phi_err": self._err(self._phi_ctl()),
                "fresco": fresco, "mejor_fp": self.mejor_fp,
                "slope": self.slope}


# ----------------------------- estadísticas -----------------------------
class Stats:
    def __init__(self, c):
        self.c = c
        self.fps = []; self.errs = []; self.tiempos = {"BUSCAR": 0.0, "PLL": 0.0}
        self.n = 0; self.n_rango = 0
        self.t_lock = None; self.t_tight = None
        self.racha = 0; self.racha_max = 0; self.rachas = []

    def reg(self, fp, err, modo, dt, t):
        self.fps.append(fp); self.errs.append(err)
        self.tiempos[modo] = self.tiempos.get(modo, 0.0) + dt
        self.n += 1
        if self.c["FP_MIN"] <= fp <= self.c["FP_MAX"]:
            self.n_rango += 1; self.racha += 1
            self.racha_max = max(self.racha_max, self.racha)
        else:
            if self.racha > 0:  self.rachas.append(self.racha)
            self.racha = 0
        if self.t_lock is None and abs(err) < self.c["PHI_LOCK"]:  self.t_lock = t
        if self.t_tight is None and abs(fp - self.c["FP"]) < self.c["TOL_FP"]:  self.t_tight = t

    def resumen(self, ctrl, t_total):
        c = self.c
        print("\n" + "=" * 70)
        print(f" RESUMEN — FP={c['FP']:.4f}  PHI={c['PHI']:.4f}°")
        print("=" * 70)
        if self.n == 0:  print(" sin datos"); return
        ef = 100 * self.n_rango / self.n
        errs = np.abs(self.errs)
        print(f" t={t_total:.1f}s | iter={self.n} | tasa={self.n/max(t_total,1e-6):.1f}Hz")
        print(f" FP: {np.mean(self.fps):.5f} ± {np.std(self.fps):.5f}")
        print(f" |PHI-PHI_obj|: media={errs.mean():.4f}°  mediana={np.median(errs):.4f}°")
        print(f" rango [{c['FP_MIN']:.4f},{c['FP_MAX']:.4f}]: {self.n_rango} ({ef:.1f}%)")
        print(f" racha máx: {self.racha_max} muestras | nº rachas: {len(self.rachas)}")
        for m, t in self.tiempos.items():
            print(f"   {m}: {t:.1f}s ({100*t/max(t_total,1e-6):.1f}%)")
        print(f" cambios fase={ctrl.n_cambios} | slope={ctrl.slope:+.3f}")
        if self.t_lock  is not None:  print(f" 1er lock (|err|<{c['PHI_LOCK']:.2f}°): {self.t_lock:.2f}s")
        if self.t_tight is not None:  print(f" 1er tight (|FP-obj|<{c['TOL_FP']:.4f}): {self.t_tight:.2f}s")
        v = "EXCELENTE" if ef >= 80 else "ACEPTABLE" if ef >= 50 else "POBRE" if ef >= 20 else "FALLO"
        print(f" veredicto: {v}")
        print("=" * 70)


# ----------------------------- menú -----------------------------
def menu(cfg):
    nombres = list(cfg["perfiles"].keys())
    print("\n" + "=" * 68)
    print(" PERFILES DISPONIBLES")
    print("=" * 68)
    print(f" {'#':>3} | {'Perfil':<10} | {'FP':>8} | {'PHI':>9} | {'Tol':>7} | "
          f"{'K':>5} | {'Paso máx':>9} | {'Filtro':>6} | {'DF_LP':>5}")
    print("-" * 68)
    for i, n in enumerate(nombres, 1):
        p = cfg["perfiles"][n]
        phi = degrees(acos(p["FP"]))
        pf = p.get("PHI_FILTER", 1); dl = p.get("DF_LP", 0.30)
        print(f" {i:>3} | {n:<10} | {p['FP']:>8.4f} | {phi:>8.3f}° | "
              f"±{p['TOL_FP']:<6.4f} | {p['K']:>5.2f} | {p['PASO_MAX']:>7.1f}° | "
              f"{pf:>6} | {dl:>5.2f}")
    print("=" * 68)
    while True:
        try:
            e = input(f"\n Selecciona [1-{len(nombres)}] (q=salir): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if e in ("q", "salir", "exit"):      return None
        if e.isdigit() and 1 <= int(e) <= len(nombres):
            return nombres[int(e) - 1]
        if e in cfg["perfiles"]:             return e
        print(" opción inválida")


# ----------------------------- ejecución -----------------------------
def ejecutar(cfg, nombre):
    C = cfg["comunes"]; P = derivar(cfg["perfiles"][nombre])
    print("\n" + "=" * 68)
    print(f" PERFIL: {nombre}   →   FP = {P['FP']:.4f}   PHI = {P['PHI']:.4f}°")
    print("=" * 68)
    print(f" FP rango       [{P['FP_MIN']:.4f}, {P['FP_MAX']:.4f}]   tol tight ±{P['TOL_FP']:.4f}")
    print(f" PHI histéresis  ENTER>{P['PHI_ENTER']}°   EXIT<{P['PHI_EXIT']}°   LOCK<{P['PHI_LOCK']:.3f}°")
    print(f" PASO BUSCAR     ini={P['PASO_INI']:.2f}°  far={P['PASO_FAR']:.2f}°  "
          f"near={P['PASO_NEAR']:.3f}°  max={P['PASO_MAX']:.1f}°")
    print(f" PLL             Kp={P['KP']:.6f}  Ki={P['KI']:.7f}  DF_MAX={P['DF_MAX']}Hz  "
          f"DF_LP={P['DF_LP']:.2f}  PHI_FILTER={P['PHI_FILTER']}")
    print("=" * 68)

    fg = wt = None; ctrl = Ctrl(P); st = Stats(P); t_ctrl = None; total = 0
    try:
        print("\n Conectando equipos...")
        fg = YokogawaFG420(C["DIR_FG"], mode='extreme');  fg.conectar()
        wt = YokogawaWT3000(C["DIR_WT"], mode='extreme'); wt.conectar()
        print(f"  FG420:  {fg.obtener_idn()[:55]}")
        print(f"  WT3000: {wt.obtener_idn()[:55]}")
        fg.extreme(canal=1, frecuencia_hz=C["FREC_HZ"],
                   amplitud_vpp=C["AMPLITUD_VPP"], offset_v=0.0,
                   fase_grados=0.0, encender_salida=True)
        wt.extreme(elemento_entrada=C["ELEMENTO_WT"],
                   incluir_potencias=True, configurar_salida=True)
        print("\n Estabilizando 5s", end="", flush=True)
        for _ in range(10):  time.sleep(0.5); print(".", end="", flush=True)
        print(" ok\n")

        print("-" * 155)
        print(f"{'t':>4} | {'FP':>9} | {'PHI':>10} | {'PHI_f':>10} | {'*':>1} | "
              f"{'Fase':>9} | {'Δf':>11} | {'Modo':>6} | {'Slope':>7} | "
              f"{'phi_err':>10} | {'MejorFP':>9} | {'Acción':>16}")
        print("-" * 155)

        t_ctrl = time.time(); t_prev = t_ctrl
        while time.time() - t_ctrl < C["TIEMPO_SEG"]:
            t_now = time.time(); dt = t_now - t_prev; t_prev = t_now
            try:
                m = wt.leer_mediciones_minimas()
            except Exception as e:
                print(f"[X] {e}"); time.sleep(C["INTERVALO_S"]); continue
            if wt.is_outlier():  continue
            fp = m.get("factor_potencia"); phi = m.get("angulo_fase"); f = m.get("frecuencia")
            if phi is None or f is None:  time.sleep(C["INTERVALO_S"]); continue

            fp_abs = abs(fp) if fp is not None else abs(np.cos(np.radians(phi)))
            total += 1
            res = ctrl.update(fp_abs, phi, dt)
            fg.establecer_frecuencia_extreme(1, res["frec"])
            if res["cambio"]:  fg.establecer_fase(1, res["fase"])
            t_rel = t_now - t_ctrl
            st.reg(fp_abs, res["phi_err"], res["modo"], dt, t_rel)

            if res["fresco"] or total % 8 == 0:
                mark = "*" if res["fresco"] else ("-" if ctrl.mov or ctrl.stale > 0 else " ")
                print(f"{int(t_rel):>4} | {fp_abs:>9.6f} | {res['phi']:>+10.4f} | "
                      f"{res['phi_f']:>+10.4f} | {mark:>1} | {res['fase']:>+9.4f} | "
                      f"{res['df']:>+11.6f} | {res['modo']:>6} | {res['slope']:>+7.3f} | "
                      f"{res['phi_err']:>+10.4f} | {res['mejor_fp']:>9.6f} | "
                      f"{res['accion']:>16}")
            elapsed = time.time() - t_now
            if elapsed < C["INTERVALO_S"]:  time.sleep(C["INTERVALO_S"] - elapsed)

        st.resumen(ctrl, time.time() - t_ctrl)
    except KeyboardInterrupt:
        print("\n\n[!] Interrumpido.")
        if total > 0 and t_ctrl:  st.resumen(ctrl, time.time() - t_ctrl)
    except Exception as e:
        print(f"\n[X] Error: {e}");  import traceback; traceback.print_exc()
    finally:
        print("\n[!] Apagando...")
        if fg is not None:
            try:  fg.establecer_salida(1, False); fg.desconectar()
            except Exception:  pass
        if wt is not None:
            try:  wt.desconectar()
            except Exception:  pass
        print("[✓] Listo.")


# ----------------------------- main -----------------------------
def main():
    ap = argparse.ArgumentParser(description="Control FP compacto")
    ap.add_argument("-p", "--perfil", help="nombre del perfil")
    ap.add_argument("-l", "--list", action="store_true", help="solo listar")
    ap.add_argument("-j", "--json", default="data.json", help="ruta del JSON")
    args = ap.parse_args()

    try:
        cfg = json.load(open(args.json, encoding="utf-8"))
    except Exception as e:
        print(f"[X] No se pudo leer {args.json}: {e}");  return 1

    if args.list:  menu(cfg); return 0

    nombre = args.perfil
    if nombre is None:
        nombre = menu(cfg)
        if nombre is None:  print("[i] Cancelado.");  return 0
    if nombre not in cfg["perfiles"]:
        print(f"[X] Perfil '{nombre}' no existe");  return 1

    ejecutar(cfg, nombre)
    return 0


if __name__ == "__main__":
    sys.exit(main())