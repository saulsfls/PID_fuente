from __future__ import annotations

import hashlib
import importlib
import json
import tkinter as tk
from pathlib import Path
from tkinter import messagebox


# ======================================================================
# Constantes de comportamiento
# ======================================================================
# Si True → pide la contraseña en CADA acción (más estricto).
# Si False → pide una sola vez por sesión (más cómodo). Default.
PEDIR_SIEMPRE = False

# Contraseña por defecto: SOLO se usa la primera vez que se ejecuta,
# antes de que exista config_secret.json. Se avisa al usuario y se
# recomienda cambiarla.
PASSWORD_DEFAULT = "hv2024"

# Longitud mínima permitida para una nueva contraseña
PASSWORD_MIN_LEN = 4

# Nombre del subdirectorio y archivos auxiliares
_PARAMS_DIRNAME    = "params_algoritmos"
_SECRET_FILENAME   = "config_secret.json"
_ARRANQUE_FILENAME = "config_arranque.json"

# Valores por defecto de arranque (solo se usan si no existe el JSON).
# Coinciden con los del main.py para no romper nada la primera vez.
_DEFAULT_VOLTAJE_INICIAL = 0.0010
_DEFAULT_PASO_ARRANQUE   = 0.0001
_DECIMALES_VRMS          = 4       # nº de decimales que usa el main.py


# ======================================================================
# Paleta (duplicada de main.py para que config.py sea autónomo)
# ======================================================================
C_BG          = "#0f1419"
C_PANEL       = "#1a212b"
C_PANEL_HI    = "#243040"
C_BORDER      = "#2d3d52"
C_TEXT        = "#d4dce6"
C_TEXT_DIM    = "#7b8a9e"
C_ACCENT      = "#4ea1ff"
C_GREEN       = "#3ddc84"
C_YELLOW      = "#ffd166"
C_RED         = "#ff5c7a"

BTN_START_BG      = "#1e7d4e"
BTN_STOP_BG       = "#8b1a1a"
BTN_NEUTRAL_BG    = "#1e2a3a"
BTN_CONNECT_BG    = "#1a4a7a"
BTN_TEXT          = "#ffffff"

FONT_TITLE = ("Segoe UI Semibold", 14)
FONT_LABEL = ("Segoe UI Semibold", 10)
FONT_MONO  = ("Consolas", 9)
FONT_BTN   = ("Segoe UI Semibold", 10)


# ======================================================================
# Helpers
# ======================================================================
def _hash_pwd(txt: str) -> str:
    return hashlib.sha256(txt.encode("utf-8")).hexdigest()


def _sanear_nombre(nombre: str) -> str:
    return "".join(c if (c.isalnum() or c in "-_") else "_"
                   for c in nombre)[:80]


# ======================================================================
# Clase principal
# ======================================================================
class Configuracion:
    """
    Puerta de entrada única a la configuración del panel.

    Parámetros
    ----------
    app : MainApp
        Referencia a la aplicación. Debe exponer:
          - .root            (Tk root)
          - .menubar         (tk.Menu principal)
          - .running         (bool)
          - .algoritmo       (instancia del algoritmo activo)
          - ._alg_nombre_sel (str, nombre del algoritmo activo)
          - .ref_source      (str)
          - ._log_threadsafe (callable)
          - ._refresh_algorithm_ui (callable, opcional)
          - ._paso_prev      (float, paso actual del voltaje)
          - .voltaje_vrms    (float, voltaje actual)
          - .var_paso        (tk.StringVar, opcional)
          - .var_voltaje     (tk.StringVar, opcional)
          - .lbl_voltage_top (tk.Label, opcional)
          - ._clamp_vrms     (callable, opcional)
          - ._actualizar_entry (callable, opcional)
    menubar : tk.Menu
        Barra de menú donde se insertará la cascada "Configuración".
    """

    def __init__(self, app, menubar):
        self.app = app
        self.menubar = menubar
        self.root = app.root

        self._desbloqueado = False

        base_dir = Path(__file__).resolve().parent
        self._params_dir = base_dir / _PARAMS_DIRNAME
        self._params_dir.mkdir(exist_ok=True)
        self._secret_path = base_dir / _SECRET_FILENAME

        self._pwd_hash = self._leer_o_crear_hash()
        self._menu = None

    # ------------------------------------------------------------------
    # Contraseña
    # ------------------------------------------------------------------
    def _leer_o_crear_hash(self) -> str:
        """Lee el hash guardado. Si no existe, lo crea con el default y
        avisa al usuario por consola."""
        if self._secret_path.exists():
            try:
                with open(self._secret_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                h = data.get("pwd_hash")
                if isinstance(h, str) and len(h) == 64:
                    return h
            except Exception:
                pass

        h = _hash_pwd(PASSWORD_DEFAULT)
        try:
            with open(self._secret_path, "w", encoding="utf-8") as f:
                json.dump({"pwd_hash": h}, f, indent=2)
        except Exception:
            pass

        self._log("=" * 78)
        self._log(f"[config] Primera ejecución: contraseña por defecto = "
                  f"'{PASSWORD_DEFAULT}'")
        self._log(f"[config] Cámbiela en: Configuración → Cambiar contraseña…")
        self._log("=" * 78)
        return h

    def _guardar_hash(self, nuevo_hash: str) -> None:
        self._pwd_hash = nuevo_hash
        try:
            with open(self._secret_path, "w", encoding="utf-8") as f:
                json.dump({"pwd_hash": nuevo_hash}, f, indent=2)
        except Exception as e:
            self._log(f"[config] No se pudo guardar la contraseña: {e}")

    def _verificar(self, txt: str) -> bool:
        return _hash_pwd(txt) == self._pwd_hash

    # ------------------------------------------------------------------
    # Diálogo de contraseña (modal)
    # ------------------------------------------------------------------
    def _pedir_password(self, motivo: str = "acceder a la configuración") -> bool:
        if self._desbloqueado and not PEDIR_SIEMPRE:
            return True

        dlg = tk.Toplevel(self.root)
        dlg.title("Contraseña requerida")
        dlg.geometry("400x190")
        dlg.configure(bg=C_BG)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.resizable(False, False)

        tk.Label(dlg, text="  🔒  Autenticación",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE,
                 anchor="w", padx=14, pady=10).pack(fill="x")

        body = tk.Frame(dlg, bg=C_BG)
        body.pack(fill="both", expand=True, padx=18, pady=(10, 6))

        tk.Label(body, text=f"Se requiere contraseña para {motivo}.",
                 bg=C_BG, fg=C_TEXT_DIM,
                 font=("Segoe UI", 9), anchor="w").pack(fill="x", pady=(0, 8))

        var_pwd = tk.StringVar()
        ent = tk.Entry(body, textvariable=var_pwd, show="•",
                       bg="#0a0e14", fg=C_GREEN, insertbackground=C_GREEN,
                       font=("Consolas", 12), relief="flat", bd=0)
        ent.pack(fill="x", ipady=6)
        ent.focus_set()

        lbl_err = tk.Label(body, text="", bg=C_BG, fg=C_RED,
                           font=("Segoe UI", 9))
        lbl_err.pack(fill="x", pady=(4, 0))

        resultado = {"ok": False}

        def _aceptar(event=None):
            if self._verificar(var_pwd.get()):
                resultado["ok"] = True
                if not PEDIR_SIEMPRE:
                    self._desbloqueado = True
                self._log("[config] Sesión desbloqueada.")
                dlg.destroy()
            else:
                lbl_err.config(text="✗  Contraseña incorrecta.")
                var_pwd.set("")
                ent.focus_set()

        def _cancelar():
            dlg.destroy()

        ent.bind("<Return>", _aceptar)
        dlg.bind("<Escape>", lambda e: _cancelar())

        bar = tk.Frame(dlg, bg=C_PANEL)
        bar.pack(fill="x", side="bottom")

        tk.Button(bar, text="Cancelar", command=_cancelar,
                  bg=BTN_STOP_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=14, pady=6,
                  cursor="hand2").pack(side="right", padx=(6, 10), pady=8)
        tk.Button(bar, text="Aceptar", command=_aceptar,
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=14, pady=6,
                  cursor="hand2").pack(side="right", padx=(6, 0), pady=8)

        self.root.wait_window(dlg)
        return resultado["ok"]

    def bloquear(self) -> None:
        """Re-bloquea la sesión. El próximo acceso pedirá contraseña."""
        self._desbloqueado = False
        self._log("[config] Sesión bloqueada.")
        try:
            messagebox.showinfo("Bloqueado",
                                "La configuración se ha bloqueado.\n"
                                "Se pedirá contraseña la próxima vez.",
                                parent=self.root)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Menú
    # ------------------------------------------------------------------
    def instalar_menu(self) -> None:
        """Inserta la cascada 'Configuración' en la barra de menú."""
        self._menu = tk.Menu(self.menubar, tearoff=0)
        self._menu.add_command(
            label="⚙  Editar parámetros del algoritmo…",
            command=self._on_edit_params)
        self._menu.add_command(
            label="💾  Guardar parámetros del algoritmo",
            command=self._on_save_params)
        self._menu.add_command(
            label="↺  Restaurar parámetros guardados",
            command=self._on_restore_params)
        self._menu.add_command(
            label="🚀  Parámetros de arranque…",
            command=self._on_edit_startup)
        self._menu.add_separator()
        self._menu.add_command(
            label="⟳  Recargar módulo del algoritmo",
            command=self._on_reload_module)
        self._menu.add_separator()
        self._menu.add_command(
            label="🔑  Cambiar contraseña…",
            command=self._on_change_pwd)
        self._menu.add_command(
            label="🔒  Bloquear ahora",
            command=self.bloquear)

        self.menubar.add_cascade(label="Configuración", menu=self._menu)

    # ------------------------------------------------------------------
    # Introspection: atributos numéricos editables
    # ------------------------------------------------------------------
    @staticmethod
    def _attrs_editables(obj):
        """
        Devuelve [(nombre, valor_actual)] de atributos públicos numéricos
        (int/float, no bool, no dunder).
        """
        out = []
        for k, v in vars(obj).items():
            if k.startswith("_"):
                continue
            if isinstance(v, bool):
                continue
            if isinstance(v, (int, float)):
                out.append((k, v))
        out.sort(key=lambda kv: kv[0])
        return out

    # ------------------------------------------------------------------
    # A. Editar parámetros en vivo
    # ------------------------------------------------------------------
    def _on_edit_params(self):
        if self.app.running:
            messagebox.showwarning(
                "Prueba en curso",
                "Detén la prueba antes de editar parámetros.",
                parent=self.root)
            return
        if not self._pedir_password("editar los parámetros del algoritmo"):
            return
        self._abrir_dialogo_params()

    def _abrir_dialogo_params(self):
        alg = self.app.algoritmo
        attrs = self._attrs_editables(alg)
        if not attrs:
            messagebox.showinfo(
                "Sin parámetros",
                f"El algoritmo '{alg.NOMBRE}' no expone atributos\n"
                f"numéricos públicos editables.",
                parent=self.root)
            return

        dlg = tk.Toplevel(self.root)
        dlg.title(f"Parámetros · {alg.NOMBRE}")
        dlg.geometry("600x580")
        dlg.configure(bg=C_BG)
        dlg.transient(self.root)
        dlg.grab_set()

        tk.Label(dlg, text="  ⚙  Parámetros editables del algoritmo",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE,
                 anchor="w", padx=14, pady=10).pack(fill="x")

        tk.Label(dlg,
                 text="  Cambia los valores y pulsa «Aplicar» (afecta en vivo).\n"
                      "  «Guardar» también los persiste en disco para la próxima\n"
                      "  sesión. «Recargar módulo» hace importlib.reload del .py.",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 9),
                 justify="left", anchor="w").pack(fill="x", padx=18, pady=(6, 8))

        # Contenedor scrollable
        cont = tk.Frame(dlg, bg=C_BG)
        cont.pack(fill="both", expand=True, padx=18, pady=(0, 8))
        canvas = tk.Canvas(cont, bg=C_BG, highlightthickness=0, bd=0)
        sb = tk.Scrollbar(cont, orient="vertical", command=canvas.yview)
        inner = tk.Frame(canvas, bg=C_BG)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        inner.columnconfigure(1, weight=1)

        vars_entries = {}
        for i, (k, v) in enumerate(attrs):
            tk.Label(inner, text=k, bg=C_BG, fg=C_TEXT,
                     font=("Consolas", 10)).grid(
                row=i, column=0, sticky="w", padx=(4, 12), pady=3)
            var = tk.StringVar(value=f"{v!r}")
            ent = tk.Entry(inner, textvariable=var, width=20,
                           bg="#0a0e14", fg=C_GREEN,
                           insertbackground=C_GREEN,
                           font=("Consolas", 11), relief="flat", bd=0)
            ent.grid(row=i, column=1, sticky="we", pady=3, ipady=3)
            vars_entries[k] = (var, v, type(v))

        bar = tk.Frame(dlg, bg=C_PANEL)
        bar.pack(fill="x", side="bottom")

        estado = tk.Label(bar, text="", bg=C_PANEL, fg=C_TEXT_DIM,
                          font=("Segoe UI", 9))
        estado.pack(side="left", padx=12)

        def _aplicar(guardar=False):
            cambios = 0
            errores = []
            for k, (var, _orig, tipo_orig) in vars_entries.items():
                txt = var.get().strip()
                try:
                    nuevo = float(txt)
                    if tipo_orig is int and float(nuevo).is_integer():
                        nuevo = int(nuevo)
                except ValueError:
                    errores.append(f"{k}: '{txt}' no es número")
                    continue
                try:
                    setattr(alg, k, nuevo)
                    cambios += 1
                except Exception as e:
                    errores.append(f"{k}: {e}")

            if errores:
                estado.config(text=f"⚠ {len(errores)} error(es). Ver consola.",
                              fg=C_YELLOW)
                for e in errores:
                    self._log(f"[config] {e}")
            else:
                estado.config(text=f"✓ {cambios} parámetros aplicados.",
                              fg=C_GREEN)
                self._log(f"[config] {cambios} parámetros aplicados en vivo a "
                          f"'{alg.NOMBRE}'.")

            if guardar:
                self._guardar_params()
                if not errores:
                    estado.config(text=f"✓ {cambios} aplicados y guardados.",
                                  fg=C_GREEN)

        def _restaurar():
            for k, (var, orig, _t) in vars_entries.items():
                var.set(f"{orig!r}")
            estado.config(text="↺ Valores restaurados (sin aplicar).",
                          fg=C_TEXT_DIM)

        def _recargar():
            self._reload_module_en_dialogo(dlg, estado)

        tk.Button(bar, text="Restaurar", command=_restaurar,
                  bg=BTN_NEUTRAL_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=10, pady=5, cursor="hand2").pack(
            side="left", padx=(6, 0))
        tk.Button(bar, text="⟳ Recargar módulo", command=_recargar,
                  bg=BTN_CONNECT_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=10, pady=5, cursor="hand2").pack(
            side="left", padx=(6, 0))

        tk.Button(bar, text="Cerrar", command=dlg.destroy,
                  bg=BTN_STOP_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 10), pady=8)
        tk.Button(bar, text="Guardar", command=lambda: _aplicar(True),
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 0), pady=8)
        tk.Button(bar, text="Aplicar", command=lambda: _aplicar(False),
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 0), pady=8)

    # ------------------------------------------------------------------
    # B. Persistir / restaurar parámetros del algoritmo
    # ------------------------------------------------------------------
    def _ruta_params(self, nombre_alg: str) -> Path:
        return self._params_dir / f"{_sanear_nombre(nombre_alg)}.json"

    def _guardar_params(self):
        alg = self.app.algoritmo
        attrs = self._attrs_editables(alg)
        if not attrs:
            return
        data = {k: v for k, v in attrs}
        ruta = self._ruta_params(self.app._alg_nombre_sel)
        try:
            with open(ruta, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self._log(f"[config] Guardados en {ruta.name}")
        except Exception as e:
            self._log(f"[config] No se pudo guardar {ruta.name}: {e}")

    def cargar_params_algoritmo(self):
        """
        Aplica al algoritmo activo los parámetros persistidos en disco.
        NO pide contraseña (se usa internamente al arrancar/cambiar alg).
        """
        alg = self.app.algoritmo
        ruta = self._ruta_params(self.app._alg_nombre_sel)
        if not ruta.exists():
            return
        try:
            with open(ruta, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self._log(f"[config] No se pudo leer {ruta.name}: {e}")
            return
        aplicados = 0
        for k, v in data.items():
            if hasattr(alg, k):
                try:
                    setattr(alg, k, v)
                    aplicados += 1
                except Exception:
                    pass
        if aplicados:
            self._log(f"[config] Aplicados {aplicados} parámetros desde "
                      f"{ruta.name}")

    def _on_save_params(self):
        if self.app.running:
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de guardar.",
                                   parent=self.root)
            return
        if not self._pedir_password("guardar los parámetros del algoritmo"):
            return
        self._guardar_params()
        messagebox.showinfo("Guardado",
                            "Parámetros guardados en disco.",
                            parent=self.root)

    def _on_restore_params(self):
        if self.app.running:
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de restaurar.",
                                   parent=self.root)
            return
        if not self._pedir_password("restaurar los parámetros del algoritmo"):
            return
        ruta = self._ruta_params(self.app._alg_nombre_sel)
        if not ruta.exists():
            messagebox.showinfo(
                "Sin archivo",
                f"No hay parámetros guardados para este algoritmo\n"
                f"({ruta.name}).",
                parent=self.root)
            return
        self.cargar_params_algoritmo()
        messagebox.showinfo("Restaurado",
                            "Parámetros restaurados desde disco.",
                            parent=self.root)

    # ------------------------------------------------------------------
    # D. Parámetros de arranque (voltaje inicial + paso de levantamiento)
    # ------------------------------------------------------------------
    def _ruta_arranque(self) -> Path:
        return Path(__file__).resolve().parent / _ARRANQUE_FILENAME

    def _leer_arranque(self) -> dict:
        ruta = self._ruta_arranque()
        if not ruta.exists():
            return {}
        try:
            with open(ruta, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _guardar_arranque(self, v_ini: float, p_ar: float) -> None:
        ruta = self._ruta_arranque()
        try:
            with open(ruta, "w", encoding="utf-8") as f:
                json.dump(
                    {"voltaje_inicial": float(v_ini),
                     "paso_arranque":   float(p_ar)},
                    f, indent=2, ensure_ascii=False)
            self._log(f"[config] Arranque guardado en {ruta.name}")
        except Exception as e:
            self._log(f"[config] No se pudo guardar arranque: {e}")

    def _modulo_principal(self):
        """Devuelve el módulo donde vive la clase del app (main)."""
        import sys as _sys
        try:
            return _sys.modules[type(self.app).__module__]
        except Exception:
            return None

    def cargar_arranque(self) -> None:
        """
        Aplica al app los parámetros de arranque persistidos.

        Llamar desde main.py DESPUÉS de construir la UI (para poder
        actualizar los Entry del voltaje y del paso).

        No pide contraseña: es solo lectura.
        """
        data = self._leer_arranque()
        if not data:
            return
        try:
            v_ini = float(data.get("voltaje_inicial", _DEFAULT_VOLTAJE_INICIAL))
            p_ar  = float(data.get("paso_arranque",   _DEFAULT_PASO_ARRANQUE))
        except Exception:
            return
        self._aplicar_arranque(v_ini, p_ar)
        self._log(f"[config] Arranque aplicado: "
                  f"V_ini={v_ini:.{_DECIMALES_VRMS}f} Vrms, "
                  f"paso={p_ar:.{_DECIMALES_VRMS}f} Vrms")

    def _aplicar_arranque(self, v_ini: float, p_ar: float) -> None:
        """Aplica voltaje inicial + paso al app, y parchea los constantes
        del módulo main para que `start_test` use los nuevos valores."""
        # 1) Paso de levantamiento
        try:
            self.app._paso_prev = float(p_ar)
            if hasattr(self.app, "var_paso"):
                self.app.var_paso.set(f"{p_ar:.{_DECIMALES_VRMS}f}")
        except Exception:
            pass

        # 2) Voltaje inicial (usa el clamp del app si está disponible)
        try:
            if hasattr(self.app, "_clamp_vrms"):
                self.app.voltaje_vrms = self.app._clamp_vrms(
                    v_ini, contexto="arranque")
            else:
                self.app.voltaje_vrms = float(v_ini)
            if hasattr(self.app, "_actualizar_entry"):
                self.app._actualizar_entry()
            if hasattr(self.app, "lbl_voltage_top"):
                self.app.lbl_voltage_top.config(
                    text=f"Vrms: {self.app.voltaje_vrms:.{_DECIMALES_VRMS}f} V")
        except Exception:
            pass

        # 3) Parchear constantes del módulo principal para start_test
        mod = self._modulo_principal()
        if mod is not None:
            try:
                mod.VOLTAJE_INICIAL_VRMS = float(v_ini)
                mod.PASO_VRMS_DEFAULT     = float(p_ar)
            except Exception:
                pass

    def _on_edit_startup(self):
        if self.app.running:
            messagebox.showwarning(
                "Prueba en curso",
                "Detén la prueba antes de editar los parámetros de arranque.",
                parent=self.root)
            return
        if not self._pedir_password("editar los parámetros de arranque"):
            return
        self._abrir_dialogo_startup()

    def _abrir_dialogo_startup(self):
        # --- Leer valores actuales ---
        paso_actual = float(getattr(self.app, "_paso_prev",
                                    _DEFAULT_PASO_ARRANQUE))
        v_actual    = float(getattr(self.app, "voltaje_vrms",
                                    _DEFAULT_VOLTAJE_INICIAL))

        # --- Leer límites desde el módulo main ---
        mod = self._modulo_principal()
        PASO_MIN = getattr(mod, "PASO_VRMS_MIN",    0.0001) if mod else 0.0001
        V_MIN    = getattr(mod, "VOLTAJE_VRMS_MIN", 0.0)    if mod else 0.0
        V_MAX    = getattr(mod, "VOLTAJE_VRMS_MAX", 7.0710) if mod else 7.0710

        # --- Diálogo ---
        dlg = tk.Toplevel(self.root)
        dlg.title("Parámetros de arranque")
        dlg.geometry("560x360")
        dlg.configure(bg=C_BG)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.resizable(False, False)

        tk.Label(dlg, text="  🚀  Parámetros de arranque",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE,
                 anchor="w", padx=14, pady=10).pack(fill="x")

        tk.Label(dlg,
                 text="  Define el estado inicial del ensayo:\n"
                      "    · Voltaje inicial (Vrms) al pulsar «Iniciar control».\n"
                      "    · Paso de levantamiento (▲ / ▼) del voltaje.",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 9),
                 justify="left", anchor="w").pack(fill="x", padx=18, pady=(6, 10))

        body = tk.Frame(dlg, bg=C_BG)
        body.pack(fill="both", expand=True, padx=18, pady=(0, 8))
        body.columnconfigure(1, weight=1)

        # Voltaje inicial
        tk.Label(body, text="Voltaje inicial (Vrms):",
                 bg=C_BG, fg=C_TEXT, font=FONT_LABEL).grid(
            row=0, column=0, sticky="w", pady=(4, 0))
        var_v = tk.StringVar(value=f"{v_actual:.{_DECIMALES_VRMS}f}")
        tk.Entry(body, textvariable=var_v, width=20,
                 bg="#0a0e14", fg=C_GREEN, insertbackground=C_GREEN,
                 font=("Consolas", 12), relief="flat", bd=0).grid(
            row=0, column=1, sticky="w", padx=(12, 0), pady=(4, 0), ipady=4)
        tk.Label(body, text=f"(rango {V_MIN:.{_DECIMALES_VRMS}f} … "
                            f"{V_MAX:.{_DECIMALES_VRMS}f})",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 8)).grid(
            row=1, column=1, sticky="w", padx=(12, 0), pady=(0, 10))

        # Paso de levantamiento
        tk.Label(body, text="Paso de levantamiento (Vrms):",
                 bg=C_BG, fg=C_TEXT, font=FONT_LABEL).grid(
            row=2, column=0, sticky="w", pady=(4, 0))
        var_p = tk.StringVar(value=f"{paso_actual:.{_DECIMALES_VRMS}f}")
        tk.Entry(body, textvariable=var_p, width=20,
                 bg="#0a0e14", fg=C_YELLOW, insertbackground=C_YELLOW,
                 font=("Consolas", 12), relief="flat", bd=0).grid(
            row=2, column=1, sticky="w", padx=(12, 0), pady=(4, 0), ipady=4)
        tk.Label(body, text=f"(mínimo {PASO_MIN:.{_DECIMALES_VRMS}f})",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 8)).grid(
            row=3, column=1, sticky="w", padx=(12, 0), pady=(0, 10))

        lbl_err = tk.Label(body, text="", bg=C_BG, fg=C_RED,
                           font=("Segoe UI", 9), justify="left", anchor="w",
                           wraplength=480)
        lbl_err.grid(row=4, column=0, columnspan=2, sticky="we", pady=(8, 0))

        # ---------- Barra inferior ----------
        bar = tk.Frame(dlg, bg=C_PANEL)
        bar.pack(fill="x", side="bottom")

        def _leer_campos():
            try:
                v_ini = float(var_v.get().strip().replace(",", "."))
                p_ar  = float(var_p.get().strip().replace(",", "."))
            except ValueError:
                return None, None, "✗  Valores no numéricos."
            if not (V_MIN <= v_ini <= V_MAX):
                return None, None, (f"✗  Voltaje inicial fuera de rango "
                                    f"({V_MIN:.{_DECIMALES_VRMS}f} … "
                                    f"{V_MAX:.{_DECIMALES_VRMS}f}).")
            if p_ar < PASO_MIN:
                return None, None, (f"✗  Paso menor que el mínimo "
                                    f"({PASO_MIN:.{_DECIMALES_VRMS}f}).")
            return v_ini, p_ar, ""

        def _aplicar(guardar=False):
            v_ini, p_ar, err = _leer_campos()
            if err:
                lbl_err.config(text=err, fg=C_RED)
                return
            self._aplicar_arranque(v_ini, p_ar)
            self._log(f"[config] Arranque: V_ini="
                      f"{v_ini:.{_DECIMALES_VRMS}f} Vrms, "
                      f"paso={p_ar:.{_DECIMALES_VRMS}f} Vrms")
            if guardar:
                self._guardar_arranque(v_ini, p_ar)
                lbl_err.config(
                    text="✓ Aplicado y guardado. Se usará en el próximo "
                         "arranque.",
                    fg=C_GREEN)
            else:
                lbl_err.config(
                    text="✓ Aplicado en esta sesión (no guardado).",
                    fg=C_GREEN)

        def _restaurar():
            var_v.set(f"{v_actual:.{_DECIMALES_VRMS}f}")
            var_p.set(f"{paso_actual:.{_DECIMALES_VRMS}f}")
            lbl_err.config(text="↺ Valores restaurados (sin aplicar).",
                           fg=C_TEXT_DIM)

        tk.Button(bar, text="Restaurar", command=_restaurar,
                  bg=BTN_NEUTRAL_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=10, pady=5, cursor="hand2").pack(
            side="left", padx=(6, 0), pady=8)

        tk.Button(bar, text="Cerrar", command=dlg.destroy,
                  bg=BTN_STOP_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 10), pady=8)
        tk.Button(bar, text="Guardar", command=lambda: _aplicar(True),
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 0), pady=8)
        tk.Button(bar, text="Aplicar", command=lambda: _aplicar(False),
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=12, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 0), pady=8)

    # ------------------------------------------------------------------
    # C. Recargar módulo .py del algoritmo
    # ------------------------------------------------------------------
    def _reload_alg_core(self):
        """Recarga el módulo .py y recrea la instancia. No pide contraseña."""
        alg = self.app.algoritmo
        mod_name = type(alg).__module__
        cls_name = type(alg).__name__
        mod = importlib.import_module(mod_name)
        importlib.reload(mod)
        cls = getattr(mod, cls_name)
        self.app.algoritmo = cls(log_cb=self.app._log_threadsafe,
                                 ref_source=self.app.ref_source)
        self.cargar_params_algoritmo()
        try:
            self.app._refresh_algorithm_ui()
        except Exception:
            pass
        self._log(f"[config] Módulo '{mod_name}' recargado.")
        return mod_name

    def _on_reload_module(self):
        if self.app.running:
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de recargar.",
                                   parent=self.root)
            return
        if not self._pedir_password("recargar el módulo del algoritmo"):
            return
        try:
            mod_name = self._reload_alg_core()
            messagebox.showinfo(
                "Recargado",
                f"Módulo '{mod_name}' recargado correctamente.",
                parent=self.root)
        except Exception as e:
            self._log(f"[config] Falló recarga: {e}")
            messagebox.showerror("Error",
                                 f"No se pudo recargar el módulo:\n{e}",
                                 parent=self.root)

    def _reload_module_en_dialogo(self, dlg, estado_label):
        try:
            mod_name = self._reload_alg_core()
            estado_label.config(text=f"⟳ Módulo '{mod_name}' recargado.",
                                fg=C_GREEN)
            dlg.after(800, dlg.destroy)
        except Exception as e:
            self._log(f"[config] Falló recarga: {e}")
            estado_label.config(text=f"⚠ Falló recarga: {e}", fg=C_RED)

    # ------------------------------------------------------------------
    # Cambio de contraseña
    # ------------------------------------------------------------------
    def _on_change_pwd(self):
        if not self._pedir_password("cambiar la contraseña"):
            return

        dlg = tk.Toplevel(self.root)
        dlg.title("Cambiar contraseña")
        dlg.geometry("440x300")
        dlg.configure(bg=C_BG)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.resizable(False, False)

        tk.Label(dlg, text="  🔑  Cambiar contraseña",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE,
                 anchor="w", padx=14, pady=10).pack(fill="x")

        body = tk.Frame(dlg, bg=C_BG)
        body.pack(fill="both", expand=True, padx=18, pady=(10, 6))

        def _campo(etiqueta):
            tk.Label(body, text=etiqueta, bg=C_BG, fg=C_TEXT,
                     font=FONT_LABEL, anchor="w").pack(fill="x", pady=(0, 2))
            v = tk.StringVar()
            tk.Entry(body, textvariable=v, show="•",
                     bg="#0a0e14", fg=C_GREEN, insertbackground=C_GREEN,
                     font=("Consolas", 11), relief="flat", bd=0).pack(
                fill="x", ipady=4, pady=(0, 8))
            return v

        var_act  = _campo("Contraseña actual:")
        var_new  = _campo("Contraseña nueva:")
        var_conf = _campo("Confirmar nueva:")

        lbl_err = tk.Label(body, text="", bg=C_BG, fg=C_RED,
                           font=("Segoe UI", 9))
        lbl_err.pack(fill="x")

        def _aplicar():
            if not self._verificar(var_act.get()):
                lbl_err.config(text="✗  La contraseña actual no es correcta.")
                return
            nueva = var_new.get()
            if len(nueva) < PASSWORD_MIN_LEN:
                lbl_err.config(text=f"✗  Mínimo {PASSWORD_MIN_LEN} caracteres.")
                return
            if nueva != var_conf.get():
                lbl_err.config(text="✗  Las contraseñas nuevas no coinciden.")
                return
            self._guardar_hash(_hash_pwd(nueva))
            self._log("[config] Contraseña actualizada.")
            messagebox.showinfo("OK", "Contraseña actualizada.",
                                parent=dlg)
            dlg.destroy()

        bar = tk.Frame(dlg, bg=C_PANEL)
        bar.pack(fill="x", side="bottom")

        tk.Button(bar, text="Cancelar", command=dlg.destroy,
                  bg=BTN_STOP_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=14, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 10), pady=8)
        tk.Button(bar, text="Aceptar", command=_aplicar,
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=FONT_BTN, padx=14, pady=6, cursor="hand2").pack(
            side="right", padx=(6, 0), pady=8)

    # ------------------------------------------------------------------
    # Log helper
    # ------------------------------------------------------------------
    def _log(self, msg: str):
        try:
            self.app._log_threadsafe(msg)
        except Exception:
            try:
                print(msg)
            except Exception:
                pass