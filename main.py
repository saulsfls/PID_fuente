"""
main.py — GUI principal v11.31.0
=========================================================================
Interfaz gráfica que:
  * Permite seleccionar el FACTOR DE POTENCIA objetivo (menu + combobox).
  * Permite seleccionar la REFERENCIA (Voltaje/Corriente) con un TOGGLE
    estilo Bootstrap form-switch en el panel principal.
  * Controla la TENSIÓN del FG (Vrms) con flechas ↑/↓ y campo editable.
  * Muestra el PF medido en grande + LED de estado + consola + stats.
  * Apagado SEGURO y ROBUSTO del FG.
  * NUEVO: Integración con el módulo de ESTADÍSTICAS ROBUSTAS del PF.

NOVEDADES v11.31.0
------------------
  * Integración del módulo `estadisticas_robustas`:
      - Pregunta al usuario (messagebox) si desea recopilar estadísticas.
      - Alimenta muestras (t, fp, phi, modo) al objeto EstadisticasRobustas.
      - Al finalizar, imprime el reporte en la consola y pregunta si guardar
        el Excel (misma funcionalidad que la versión CLI, pero sin bloquear
        el hilo de Tk).
  * Slider de REFERENCIA reemplazado por un ToggleSwitch con la misma
    apariencia que un `<input type="checkbox" role="switch">` de Bootstrap.
    Texto y lógica se mantienen: VOLTAJE (FU) ← → CORRIENTE (FI).

NOVEDADES v11.30.1 (heredadas)
------------------------------
  * FIX CRÍTICO: las flechas ↑/↓ dejaban de funcionar para el voltaje
    después de modificar el paso.
  * PASO_VRMS_DEFAULT = 0.0001.
"""
import io
import math
import threading
import sys
import time
from pathlib import Path
import traceback
sys.path.append(str(Path(__file__).resolve().parent.parent))

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

try:
    import pyvisa
    PYVISA_OK = True
except ImportError:
    pyvisa = None
    PYVISA_OK = False

# ==============================================================================
# IMPORT ROBUSTO DE ALGORITMOS
# ==============================================================================
_ALGORITMOS_DEF = [
    ("PF1 · FP=1.000 (PROBE+PLL)",               "module.pf1",     "AlgoritmoPF1"),
    ("PF050 · FP=0.500 (FF frec. v11.27 · 3%)",  "module.pf050",   "AlgoritmoPF050"),
    ("PF020 · FP=0.200 (v11.10 sesgo/BUSCAR)",   "module.pf020",   "AlgoritmoPF020"),
    ("PF010 · FP=0.100 (PI posición)",           "module.pf010",     "AlgoritmoPF010"),
    ("PF0050 · FP=0.050 (FF frec. v11.24-05)",   "module.pf0050",   "AlgoritmoPF0050"),
    ("PF0010 · FP=0.010 (FF frec. v11.24-01)",   "module.pf0010",   "AlgoritmoPF0010"),
    ("PF00010 · FP=0.001 (FF+integrador v11.27)","module.pf00010", "AlgoritmoPF00010"),
]

ALGORITMOS_DISPONIBLES = {}
_MODULOS_FALTANTES = []

for _nombre, _mod, _cls in _ALGORITMOS_DEF:
    try:
        _m = __import__(_mod, fromlist=[_cls])
        _c = getattr(_m, _cls)
        ALGORITMOS_DISPONIBLES[_nombre] = _c
    except Exception as _e:
        _MODULOS_FALTANTES.append((_mod, _cls, str(_e)))

if not ALGORITMOS_DISPONIBLES:
    raise RuntimeError(
        "No se pudo cargar ningún algoritmo. Revise los imports:\n  " +
        "\n  ".join(f"{m}.{c}: {e}" for m, c, e in _MODULOS_FALTANTES)
    )

DEFAULT_ALG = next(iter(ALGORITMOS_DISPONIBLES))

# ==============================================================================
# IMPORT ROBUSTO DEL MÓDULO DE ESTADÍSTICAS ROBUSTAS
# ==============================================================================
# Se intenta importar con varios nombres posibles (según cómo se haya
# guardado el archivo del módulo). Si no se encuentra, la GUI sigue
# funcionando, simplemente sin recopilar estadísticas.
EstadisticasRobustas = None
STATS_OK = False
_STATS_ERR = ""

for _stats_mod_name in (
    "estadisticas_robustas",
    "estadisticas",
    "stats_robustas",
    "stats_pf",
):
    try:
        _stats_mod = __import__(_stats_mod_name, fromlist=["EstadisticasRobustas"])
        EstadisticasRobustas = getattr(_stats_mod, "EstadisticasRobustas")
        STATS_OK = True
        _STATS_ERR = ""
        break
    except Exception as _e:
        _STATS_ERR = str(_e)

# ==============================================================================
# CONFIGURACIÓN
# ==============================================================================
# ---- Voltaje en Vrms (v11.30.1) ----
PASO_VRMS_DEFAULT    = 0.0001
PASO_VRMS_MIN        = 0.0001
VOLTAJE_INICIAL_VRMS = 0.0010
VOLTAJE_VRMS_MIN     = 0.0000

# ---- Límite máximo del generador ----
VPP_MAX              = 20.0
VOLTAJE_VRMS_MAX     = 7.0710
VOLTAJE_VRMS_DECIMALES = 4

TIEMPO_PRUEBA_SEG  = 300
INTERVALO_MUESTREO = 0.05
FREC_NOMINAL       = 60.0
OFFSET_V_FG        = 0.0

DEFAULT_DIR_FG = "GPIB1::2::INSTR"
DEFAULT_DIR_WT = "GPIB0::1::INSTR"
ELEMENTO_WT    = 1

SETTLE_VPP_OFF_S    = 0.15
CONEXION_TIMEOUT_MS = 2000
MAX_ERRORES_LECTURA_CONSEC = 100

MAX_NAN_CONSEC = 300
HEARTBEAT_S = 2.0

REF_VOLTAGE = "voltage"
REF_CURRENT = "current"
DEFAULT_REF_SOURCE = REF_VOLTAGE
REF_SLIDER_VOLTAGE = 0
REF_SLIDER_CURRENT = 1

# ==============================================================================
# PALETA
# ==============================================================================
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
C_GRAY_LED    = "#55606f"
C_CONSOLE_BG  = "#0a0e14"
C_CONSOLE_FG  = "#7ee787"
C_STATS_BG    = C_CONSOLE_BG
C_STATS_FG    = C_CONSOLE_FG

BTN_START_BG      = "#1e7d4e"
BTN_START_HOVER   = "#16663e"
BTN_STOP_BG       = "#8b1a1a"
BTN_STOP_HOVER    = "#6b1414"
BTN_NEUTRAL_BG    = "#1e2a3a"
BTN_NEUTRAL_HOVER = "#16202c"
BTN_CONNECT_BG    = "#1a4a7a"
BTN_CONNECT_HOVER = "#123457"
BTN_TEXT          = "#ffffff"

FONT_TITLE    = ("Segoe UI Semibold", 15)
FONT_SUB      = ("Segoe UI", 10)
FONT_LABEL    = ("Segoe UI Semibold", 10)
FONT_PF_HUGE  = ("Consolas", 46, "bold")
FONT_PF_STATE = ("Segoe UI Semibold", 13)
FONT_MONO     = ("Consolas", 9)
FONT_BTN      = ("Segoe UI Semibold", 10)
FONT_LED      = ("Segoe UI Semibold", 10)
FONT_SLIDER   = ("Segoe UI Semibold", 11)
FONT_BIG      = ("Consolas", 20, "bold")


def _es_finito(x):
    if x is None:
        return False
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


# ==============================================================================
# TOGGLE SWITCH (apariencia Bootstrap form-switch)
# ==============================================================================
class ToggleSwitch(tk.Canvas):
    """
    Switch on/off dibujado sobre un Canvas, imitando el aspecto de un
    <input type="checkbox" role="switch" class="form-check-input"> de
    Bootstrap.

    - variable : tk.IntVar (0 = off, 1 = on)
    - command  : callback invocado con el nuevo valor entero (0/1)
    - set_enabled(False) deshabilita el switch sin cambiar la variable.

    NOTA: los atributos de tamaño se guardan como `_sw_w` / `_sw_h` para
    NO colisionar con `tk.Canvas._w` (ruta interna del widget) ni con
    `tk.Canvas._h` (usado por YView).
    """
    def __init__(self, parent, variable=None, command=None,
                 on_color=C_ACCENT, off_color=C_GRAY_LED,
                 knob_color="#ffffff", width=48, height=24, **kwargs):
        try:
            bg = parent.cget("bg")
        except Exception:
            bg = C_PANEL
        super().__init__(parent, width=width, height=height, bg=bg,
                         highlightthickness=0, bd=0, **kwargs)
        self._sw_w = int(width)
        self._sw_h = int(height)
        self.var = variable if variable is not None else tk.IntVar()
        self.command = command
        self.on_color = on_color
        self.off_color = off_color
        self.knob_color = knob_color
        self._enabled = True
        self.configure(cursor="hand2")

        self.bind("<Button-1>", self._on_click)
        self.bind("<space>",    self._on_click)
        self.var.trace_add("write", lambda *a: self._draw())
        self._draw()

    def _draw(self):
        self.delete("all")
        w, h = self._sw_w, self._sw_h
        r = h // 2
        encendido = bool(self.var.get())
        color = self.on_color if encendido else self.off_color

        # Riel (rectángulo redondeado = 2 óvalos + 1 rect central)
        self.create_oval(0, 0, h, h, fill=color, outline=color)
        self.create_oval(w - h, 0, w, h, fill=color, outline=color)
        self.create_rectangle(r, 0, w - r, h, fill=color, outline=color)

        # Knob (círculo blanco)
        pad = 3
        d = h - 2 * pad
        x0 = (w - h + pad) if encendido else pad
        self.create_oval(x0, pad, x0 + d, h - pad,
                         fill=self.knob_color, outline=self.knob_color)

    def _on_click(self, event=None):
        if not self._enabled:
            return "break"
        self.var.set(0 if self.var.get() else 1)
        if self.command is not None:
            try:
                self.command(self.var.get())
            except Exception:
                pass
        return "break"

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)
        try:
            self.configure(cursor=("hand2" if enabled else "arrow"))
        except Exception:
            pass


# ==============================================================================
# APLICACIÓN PRINCIPAL
# ==============================================================================
class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control FP · v11.31.0")
        self.root.geometry("1240x870")
        self.root.minsize(1000, 720)
        self.root.configure(bg=C_BG)

        self.voltaje_vrms = VOLTAJE_INICIAL_VRMS
        self._paso_prev = PASO_VRMS_DEFAULT
        self._confirmando_paso = False
        self._shutdown_done = False

        self._fg_lock = threading.RLock()

        self.dir_fg = DEFAULT_DIR_FG
        self.dir_wt = DEFAULT_DIR_WT
        self.estado_conexion = "desconocido"

        self.ref_source = DEFAULT_REF_SOURCE
        self.var_ref = tk.IntVar(value=REF_SLIDER_VOLTAGE)

        self._alg_nombre_sel = DEFAULT_ALG
        self.algoritmo = ALGORITMOS_DISPONIBLES[DEFAULT_ALG](
            log_cb=self._log_threadsafe,
            ref_source=self.ref_source,
        )
        self.var_alg = tk.StringVar(value=self._alg_nombre_sel)

        self.hilo = None
        self.running = False
        self.fg = None
        self.wt = None

        self._last_hb = 0.0
        self._ultimo_aviso_max = 0.0

        # --- Estadísticas robustas ---
        self.stats = None
        self._ultimo_t_rel = 0.0

        self._vcmd_voltaje = (self.root.register(self._validar_entry_voltaje), "%P")
        self._vcmd_paso    = (self.root.register(self._validar_entry_paso), "%P")

        self._build_menubar()
        self._build_ui()
        self._bind_keys()
        self._refresh_algorithm_ui()

        self._log("=" * 110)
        self._log(f"  Algoritmo activo : {self.algoritmo.NOMBRE}")
        self._log(f"  Referencia       : {self.ref_source.upper()}")
        self._log(f"  Dirección FG420  : {self.dir_fg}")
        self._log(f"  Dirección WT3000 : {self.dir_wt}")
        self._log(f"  V inicial (Vrms) : "
                  f"{self.voltaje_vrms:.{VOLTAJE_VRMS_DECIMALES}f} Vrms")
        self._log(f"  Rango Vrms       : "
                  f"{VOLTAJE_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f} … "
                  f"{VOLTAJE_VRMS_MAX:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                  f"(Vpp_max ≈ {VPP_MAX:.2f} V)")
        self._log(f"  Paso (Vrms)      : "
                  f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                  f"(mínimo {PASO_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f})")
        self._log(f"  Filtro NaN/Inf   : ACTIVO")
        self._log(f"  Heartbeat        : ACTIVO ({HEARTBEAT_S:.1f} s)")
        if STATS_OK:
            self._log(f"  Estadísticas     : módulo disponible ✓")
        else:
            self._log(f"  Estadísticas     : módulo NO disponible "
                      f"({_STATS_ERR})")
        self._log("=" * 110)

        if not PYVISA_OK:
            self._log("[!] pyvisa no disponible.")
        if _MODULOS_FALTANTES:
            self._log("[!] Módulos NO disponibles:")
            for m, c, e in _MODULOS_FALTANTES:
                self._log(f"      {m}.{c}  →  {e}")

        self._set_led("desconocido")

    # ==================================================================
    # MENÚ
    # ==================================================================
    def _build_menubar(self):
        self.menubar = tk.Menu(self.root)

        m_archivo = tk.Menu(self.menubar, tearoff=0)
        m_archivo.add_command(label="Limpiar consola", command=self._limpiar)
        m_archivo.add_separator()
        m_archivo.add_command(label="Salir", command=self.root.quit)
        self.menubar.add_cascade(label="Archivo", menu=m_archivo)

        self.menu_dir = tk.Menu(self.menubar, tearoff=0)
        self.menu_dir.add_command(label="Modificar direcciones...",
                                   command=self._open_address_dialog)
        self.menu_dir.add_command(label="Actualizar lista de recursos VISA",
                                   command=self._log_recursos_visa)
        self.menu_dir.add_separator()
        self.menu_dir.add_command(label="Probar conexión ahora",
                                   command=self._conectar_instrumentos)
        self.menubar.add_cascade(label="Direcciones", menu=self.menu_dir)

        self.menu_pf = tk.Menu(self.menubar, tearoff=0)
        for nombre in ALGORITMOS_DISPONIBLES.keys():
            self.menu_pf.add_radiobutton(
                label=nombre, variable=self.var_alg, value=nombre,
                command=self._on_alg_change_menu,
            )
        self.menubar.add_cascade(label="Factor de Potencia", menu=self.menu_pf)

        m_ayuda = tk.Menu(self.menubar, tearoff=0)
        m_ayuda.add_command(label="Acerca de...", command=self._about)
        self.menubar.add_cascade(label="Ayuda", menu=m_ayuda)

        self.root.config(menu=self.menubar)

    def _about(self):
        messagebox.showinfo(
            "Acerca de",
            "Control de Factor de Potencia\n"
            "Versión GUI: v11.31.0\n\n"
            f"Algoritmo : {self.algoritmo.NOMBRE}\n"
            f"Referencia: {self.ref_source.upper()}\n"
            f"Rango Vrms: {VOLTAJE_VRMS_MIN:.4f} … "
            f"{VOLTAJE_VRMS_MAX:.4f} Vrms\n\n"
            f"FG420 : {self.dir_fg}\n"
            f"WT3000: {self.dir_wt}\n\n"
            f"Estadísticas robustas: "
            f"{'disponible ✓' if STATS_OK else 'no disponible ✗'}"
        )

    def _on_alg_change_menu(self):
        nuevo = self.var_alg.get()
        if nuevo == self._alg_nombre_sel:
            return
        if self.running:
            self.var_alg.set(self._alg_nombre_sel)
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de cambiar de algoritmo.")
            return
        self._alg_nombre_sel = nuevo
        self.algoritmo = ALGORITMOS_DISPONIBLES[nuevo](
            log_cb=self._log_threadsafe, ref_source=self.ref_source,
        )
        try:
            self.combo_alg.set(nuevo)
        except Exception:
            pass
        self._refresh_algorithm_ui()
        self._log("=" * 110)
        self._log(f"  Algoritmo cambiado → {self.algoritmo.NOMBRE}")
        self._log(f"  Referencia mantenida → {self.ref_source.upper()}")
        self._log("=" * 110)
        self.status.config(text=f"  Algoritmo: {self.algoritmo.NOMBRE}")

    # ==================================================================
    # TOGGLE DE REFERENCIA (reemplaza al slider)
    # ==================================================================
    def _on_ref_slider_change(self, value):
        """Mismo callback que usaba el Scale: recibe 0 (voltaje) o 1 (corriente)."""
        val = int(round(float(value)))
        nuevo = REF_CURRENT if val == REF_SLIDER_CURRENT else REF_VOLTAGE
        if nuevo == self.ref_source:
            return
        self.ref_source = nuevo
        try:
            self.algoritmo.set_ref_source(nuevo)
        except Exception as e:
            self._log(f"[!] No se pudo cambiar la referencia: {e}")
        self._refresh_algorithm_ui()
        self._log("=" * 110)
        self._log(f"  Referencia de frecuencia cambiada → {nuevo.upper()}")
        self._log("=" * 110)
        self.status.config(text=f"  Referencia: {nuevo.upper()}")

    def _set_slider_habilitado(self, habilitado):
        try:
            self.toggle_ref.set_enabled(habilitado)
        except Exception:
            pass
        self._actualizar_color_slider()

    def _actualizar_color_slider(self):
        activo = C_ACCENT
        inactivo = C_TEXT_DIM
        try:
            if self.ref_source == REF_VOLTAGE:
                self.lbl_ref_left.configure(fg=activo)
                self.lbl_ref_right.configure(fg=inactivo)
                self.lbl_ref_valor.configure(text="VOLTAJE (FU)", fg=activo)
            else:
                self.lbl_ref_left.configure(fg=inactivo)
                self.lbl_ref_right.configure(fg=activo)
                self.lbl_ref_valor.configure(text="CORRIENTE (FI)", fg=activo)
        except Exception:
            pass

    # ==================================================================
    # UI
    # ==================================================================
    def _build_ui(self):
        header = tk.Frame(self.root, bg=C_PANEL, height=64)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)

        left = tk.Frame(header, bg=C_PANEL)
        left.pack(side="left", fill="y", padx=(14, 0))

        tk.Label(left, text="Control de Factor de Potencia",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE).pack(
            side="top", anchor="w", pady=(6, 0))

        row_alg = tk.Frame(left, bg=C_PANEL)
        row_alg.pack(side="top", anchor="w")
        tk.Label(row_alg, text="Factor de potencia objetivo:",
                 bg=C_PANEL, fg=C_TEXT_DIM,
                 font=("Segoe UI Semibold", 9)).pack(side="left", padx=(0, 4))

        self.combo_alg = ttk.Combobox(
            row_alg, textvariable=self.var_alg,
            values=list(ALGORITMOS_DISPONIBLES.keys()),
            state="readonly", width=42, font=("Segoe UI", 9))
        self.combo_alg.pack(side="left", padx=2)
        self.combo_alg.bind("<<ComboboxSelected>>", self._on_alg_change_combo)

        right = tk.Frame(header, bg=C_PANEL)
        right.pack(side="right", fill="y", padx=(0, 20))

        led_block = tk.Frame(right, bg=C_PANEL)
        led_block.pack(side="right", padx=(16, 0))
        self.lbl_led_dot = tk.Label(
            led_block, text="●", bg=C_PANEL, fg=C_GRAY_LED,
            font=("Segoe UI", 16, "bold"))
        self.lbl_led_dot.pack(side="left")
        self.lbl_led_text = tk.Label(
            led_block, text="DESCONOCIDO", bg=C_PANEL, fg=C_GRAY_LED,
            font=FONT_LED)
        self.lbl_led_text.pack(side="left", padx=(4, 0))

        self.btn_conectar = tk.Button(
            right, text="🔌  Conectar",
            command=self._conectar_instrumentos,
            bg=BTN_CONNECT_BG, fg=BTN_TEXT,
            activebackground=BTN_CONNECT_HOVER, activeforeground=BTN_TEXT,
            disabledforeground="#8a8a8a",
            relief="flat", bd=0, highlightthickness=0,
            font=FONT_BTN, cursor="hand2", padx=12, pady=6)
        self.btn_conectar.pack(side="right", padx=(12, 0))

        self.lbl_voltage_top = tk.Label(
            right, text=f"Vrms: {self.voltaje_vrms:.{VOLTAJE_VRMS_DECIMALES}f} V",
            bg=C_PANEL, fg=C_ACCENT, font=("Consolas", 10, "bold"))
        self.lbl_voltage_top.pack(side="right", padx=(12, 0))

        self._build_ref_strip()

        cards = tk.Frame(self.root, bg=C_BG)
        cards.pack(fill="x", padx=12, pady=(6, 6))
        cards.columnconfigure(0, weight=1, uniform="c")
        cards.columnconfigure(1, weight=1, uniform="c")
        cards.columnconfigure(2, weight=2, uniform="c")

        # --- Tarjeta 1: Voltaje ---
        c1 = self._card(cards, "⚡ VOLTAJE  (Vrms)")
        c1.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        vrow = tk.Frame(c1.inner, bg=C_PANEL)
        vrow.pack(fill="x", padx=14, pady=(6, 12))

        self.btn_up = tk.Button(vrow, text="▲", command=self._on_up,
                  bg=C_PANEL_HI, fg=C_ACCENT, activebackground=C_ACCENT,
                  activeforeground="white", relief="flat", bd=0,
                  font=("Segoe UI", 14, "bold"), cursor="hand2", width=3)
        self.btn_up.pack(side="left", padx=(0, 8))

        self.var_voltaje = tk.StringVar(
            value=f"{self.voltaje_vrms:.{VOLTAJE_VRMS_DECIMALES}f}")
        self.entry_voltaje = tk.Entry(
            vrow, textvariable=self.var_voltaje, justify="center",
            font=FONT_BIG, width=10,
            bg="#0a0e14", fg=C_GREEN, insertbackground=C_GREEN,
            relief="flat", bd=0,
            validate="key", validatecommand=self._vcmd_voltaje)
        self.entry_voltaje.pack(side="left", padx=4, ipady=6, expand=True, fill="x")
        self.entry_voltaje.bind("<Return>", self._on_manual_entry)
        self.entry_voltaje.bind("<FocusOut>", self._on_manual_entry)
        self.entry_voltaje.bind("<KeyPress-minus>", lambda e: "break")
        self.entry_voltaje.bind("<KeyPress-KP_Subtract>", lambda e: "break")

        self.btn_down = tk.Button(vrow, text="▼", command=self._on_down,
                  bg=C_PANEL_HI, fg=C_ACCENT, activebackground=C_ACCENT,
                  activeforeground="white", relief="flat", bd=0,
                  font=("Segoe UI", 14, "bold"), cursor="hand2", width=3)
        self.btn_down.pack(side="left", padx=(8, 0))

        tk.Label(c1.inner,
                 text=f"↑ / ↓  ajustan en pasos · rango "
                      f"{VOLTAJE_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f} … "
                      f"{VOLTAJE_VRMS_MAX:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                      f"(Vpp_máx ≈ {VPP_MAX:.2f} V)",
                 bg=C_PANEL, fg=C_TEXT_DIM, font=("Segoe UI", 8)).pack(pady=(0, 10))

        # --- Tarjeta 2: Paso ---
        c2 = self._card(cards, "⚙ PASO  (Vrms / pulsación)")
        c2.grid(row=0, column=1, sticky="nsew", padx=(6, 6))

        prow = tk.Frame(c2.inner, bg=C_PANEL)
        prow.pack(fill="x", padx=14, pady=(6, 12))

        self.var_paso = tk.StringVar(
            value=f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f}")
        self.entry_paso = tk.Entry(
            prow, textvariable=self.var_paso, justify="center",
            font=FONT_BIG, width=10,
            bg="#0a0e14", fg=C_YELLOW, insertbackground=C_YELLOW,
            relief="flat", bd=0,
            validate="key", validatecommand=self._vcmd_paso)
        self.entry_paso.pack(expand=True, fill="x", ipady=6)
        self.entry_paso.bind("<Return>", self._on_paso_commit)
        self.entry_paso.bind("<Escape>", self._on_paso_escape)
        self.entry_paso.bind("<Up>",   self._on_paso_spin_up)
        self.entry_paso.bind("<Down>", self._on_paso_spin_down)
        self.entry_paso.bind("<KeyPress-minus>", lambda e: "break")
        self.entry_paso.bind("<KeyPress-KP_Subtract>", lambda e: "break")

        self.lbl_paso_info = tk.Label(
            c2.inner,
            text=f"Enter confirma · Esc cancela · mínimo "
                 f"{PASO_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f} Vrms · "
                 f"↑/↓ dentro del campo ajustan el paso",
            bg=C_PANEL, fg=C_TEXT_DIM, font=("Segoe UI", 8))
        self.lbl_paso_info.pack(pady=(0, 10))

        # --- Tarjeta 3: PF medido ---
        c3 = self._card(cards, "◉ PF MEDIDO")
        c3.grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        self.lbl_pf_valor = tk.Label(
            c3.inner, text="—.————",
            bg=C_PANEL, fg=C_TEXT_DIM, font=FONT_PF_HUGE)
        self.lbl_pf_valor.pack(pady=(4, 0))

        self.lbl_pf_estado = tk.Label(
            c3.inner, text="ESPERANDO",
            bg=C_PANEL, fg=C_TEXT_DIM, font=FONT_PF_STATE)
        self.lbl_pf_estado.pack(pady=(0, 4))

        self.lbl_pf_target = tk.Label(
            c3.inner, text="objetivo: —",
            bg=C_PANEL, fg=C_TEXT_DIM, font=("Segoe UI", 9, "italic"))
        self.lbl_pf_target.pack(pady=(0, 2))

        self.pf_ranges_frame = tk.Frame(c3.inner, bg=C_PANEL)
        self.pf_ranges_frame.pack(pady=(0, 10))

        # ---------- Botones ----------
        btns = tk.Frame(self.root, bg=C_BG)
        btns.pack(fill="x", padx=12, pady=(2, 8))

        self.btn_start = self._mk_btn(btns, "▶  Iniciar control FP",
                                      self.start_test,
                                      BTN_START_BG, BTN_START_HOVER)
        self.btn_start.pack(side="left", padx=(0, 8))

        self.btn_stop = self._mk_btn(btns, "■  Detener ",
                                     self.stop_test,
                                     BTN_STOP_BG, BTN_STOP_HOVER)
        self.btn_stop.pack(side="left", padx=(0, 8))
        self.btn_stop.configure(state="disabled")

        self._mk_btn(btns, "🗑  Limpiar consola",
                     self._limpiar,
                     BTN_NEUTRAL_BG, BTN_NEUTRAL_HOVER).pack(side="left", padx=(0, 8))

        tk.Label(btns, text="Los cambios de paso y voltaje se aplican en vivo · "
                            "algoritmo, dirección y referencia bloqueados en prueba",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 9)).pack(side="right")

        # ---------- Consola ----------
        cons_frame = tk.Frame(self.root, bg=C_BORDER)
        cons_frame.pack(fill="both", expand=True, padx=12, pady=(0, 6))

        tk.Label(cons_frame, text="  ▎CONSOLA DEL ALGORITMO — salida en vivo",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_LABEL,
                 anchor="w", padx=10, pady=4).pack(fill="x")

        self.console = scrolledtext.ScrolledText(
            cons_frame, height=14, wrap="none",
            font=FONT_MONO, bg=C_CONSOLE_BG, fg=C_CONSOLE_FG,
            insertbackground=C_CONSOLE_FG, relief="flat", bd=0)
        self.console.pack(fill="both", expand=True, padx=1, pady=(0, 1))

        # ---------- Estadísticas ----------
        stats_frame = tk.Frame(self.root, bg=C_BORDER)
        stats_frame.pack(fill="both", expand=False, padx=12, pady=(0, 6))

        tk.Label(stats_frame, text="  ▎ESTADÍSTICAS DEL ALGORITMO",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_LABEL,
                 anchor="w", padx=10, pady=4).pack(fill="x")

        self.stats_text = scrolledtext.ScrolledText(
            stats_frame, height=8, wrap="none",
            font=FONT_MONO, bg=C_STATS_BG, fg=C_STATS_FG,
            insertbackground=C_STATS_FG, relief="flat", bd=0)
        self.stats_text.pack(fill="both", expand=True, padx=1, pady=(0, 1))

        self.status = tk.Label(
            self.root, text="  Listo. Pulse ▶ Iniciar control FP.",
            bg=C_PANEL, fg=C_TEXT, anchor="w", padx=10, pady=4,
            font=("Segoe UI", 9))
        self.status.pack(fill="x", side="bottom")

    def _build_ref_strip(self):
        """
        Franja de REFERENCIA con un toggle estilo Bootstrap form-switch.
        Mantiene el texto VOLTAJE (FU) / CORRIENTE (FI) y la lógica
        original (var_ref: 0=voltaje, 1=corriente).
        """
        strip = tk.Frame(self.root, bg=C_PANEL)
        strip.pack(fill="x", padx=12, pady=(6, 0))

        tk.Label(strip, text="REFERENCIA:", bg=C_PANEL, fg=C_TEXT_DIM,
                 font=FONT_LABEL).pack(side="left", padx=(12, 8))

        mid = tk.Frame(strip, bg=C_PANEL)
        mid.pack(side="left", padx=(0, 12), pady=6)

        self.lbl_ref_left = tk.Label(
            mid, text="VOLTAJE (FU)", bg=C_PANEL, fg=C_ACCENT,
            font=FONT_SLIDER)
        self.lbl_ref_left.pack(side="left", padx=(4, 10))

        # --- Toggle (reemplaza al antiguo tk.Scale) ---
        self.toggle_ref = ToggleSwitch(
            mid, variable=self.var_ref,
            command=self._on_ref_slider_change,
            on_color=C_ACCENT,
            off_color=C_GRAY_LED,
            width=48, height=24,
        )
        self.toggle_ref.pack(side="left", padx=4)

        self.lbl_ref_right = tk.Label(
            mid, text="CORRIENTE (FI)", bg=C_PANEL, fg=C_TEXT_DIM,
            font=FONT_SLIDER)
        self.lbl_ref_right.pack(side="left", padx=(10, 4))

        self.lbl_ref_valor = tk.Label(
            strip, text="VOLTAJE (FU)", bg=C_PANEL, fg=C_ACCENT,
            font=("Consolas", 10, "bold"))
        self.lbl_ref_valor.pack(side="right", padx=14)

        self._actualizar_color_slider()

    def _card(self, parent, title):
        outer = tk.Frame(parent, bg=C_BORDER)
        inner = tk.Frame(outer, bg=C_PANEL)
        inner.pack(fill="both", expand=True, padx=1, pady=1)
        tk.Label(inner, text=title, bg=C_PANEL, fg=C_ACCENT,
                 font=FONT_LABEL, anchor="w", padx=10, pady=6).pack(fill="x")
        outer.inner = inner
        return outer

    def _mk_btn(self, parent, text, cmd, bg, hover):
        b = tk.Button(
            parent, text=text, command=cmd,
            bg=bg, fg=BTN_TEXT,
            activebackground=hover, activeforeground=BTN_TEXT,
            disabledforeground="#8a8a8a",
            relief="flat", bd=0, highlightthickness=0,
            font=FONT_BTN, cursor="hand2", padx=14, pady=7)
        b.bind("<Enter>", lambda e, w=b, c=hover: w.configure(bg=c) if w["state"] != "disabled" else None)
        b.bind("<Leave>", lambda e, w=b, c=bg:    w.configure(bg=c) if w["state"] != "disabled" else None)
        return b

    def _set_led(self, estado):
        self.estado_conexion = estado
        colores = {
            "desconocido": (C_GRAY_LED, "DESCONOCIDO"),
            "probando":    (C_YELLOW,   "PROBANDO..."),
            "conectado":   (C_GREEN,    "CONECTADO"),
            "error":       (C_RED,      "ERROR"),
        }
        col, txt = colores.get(estado, (C_GRAY_LED, "DESCONOCIDO"))
        try:
            self.lbl_led_dot.configure(fg=col)
            self.lbl_led_text.configure(fg=col, text=txt)
        except Exception:
            pass

    # ==================================================================
    # VISA / Direcciones
    # ==================================================================
    def _listar_recursos_visa(self):
        if not PYVISA_OK:
            return []
        try:
            rm = pyvisa.ResourceManager()
            recursos = list(rm.list_resources())
            try:
                rm.close()
            except Exception:
                pass
            return recursos
        except Exception as e:
            self._log_threadsafe(f"[X] No se pudo listar recursos VISA: {e}")
            return []

    def _log_recursos_visa(self):
        recursos = self._listar_recursos_visa()
        if not recursos:
            self._log_threadsafe("[!] No se encontraron recursos VISA disponibles.")
            return
        self._log_threadsafe("─" * 60)
        self._log_threadsafe("  Recursos VISA disponibles:")
        for r in recursos:
            self._log_threadsafe(f"    · {r}")
        self._log_threadsafe("─" * 60)

    def _open_address_dialog(self):
        if self.running:
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de cambiar las direcciones.")
            return

        recursos = self._listar_recursos_visa()

        dlg = tk.Toplevel(self.root)
        dlg.title("Direcciones de instrumentos")
        dlg.geometry("720x430")
        dlg.configure(bg=C_BG)
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.resizable(False, False)

        tk.Label(dlg, text="  Configurar direcciones VISA",
                 bg=C_PANEL, fg=C_TEXT, font=FONT_TITLE,
                 anchor="w", padx=14, pady=10).pack(fill="x")

        body = tk.Frame(dlg, bg=C_BG)
        body.pack(fill="both", expand=True, padx=18, pady=(8, 8))

        tk.Label(body, text="FG420 (Generador de funciones):",
                 bg=C_BG, fg=C_TEXT, font=FONT_LABEL).grid(
            row=0, column=0, sticky="w", pady=(8, 4))

        var_fg = tk.StringVar(value=self.dir_fg)
        combo_fg = ttk.Combobox(body, textvariable=var_fg,
                                 values=recursos, width=52, font=("Consolas", 10))
        combo_fg.grid(row=0, column=1, sticky="we", padx=(10, 0), pady=(8, 4))

        tk.Label(body, text="WT3000 (Vatímetro / Analizador):",
                 bg=C_BG, fg=C_TEXT, font=FONT_LABEL).grid(
            row=1, column=0, sticky="w", pady=(8, 4))

        var_wt = tk.StringVar(value=self.dir_wt)
        combo_wt = ttk.Combobox(body, textvariable=var_wt,
                                 values=recursos, width=52, font=("Consolas", 10))
        combo_wt.grid(row=1, column=1, sticky="we", padx=(10, 0), pady=(8, 4))

        tk.Label(body, text="Recursos VISA detectados:",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI Semibold", 9)).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(14, 4))

        listbox_frame = tk.Frame(body, bg=C_BORDER)
        listbox_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(0, 6))
        body.rowconfigure(3, weight=1)
        body.columnconfigure(1, weight=1)

        lb = tk.Listbox(listbox_frame, height=6, bg="#0a0e14", fg=C_CONSOLE_FG,
                        font=("Consolas", 9), selectmode="browse",
                        relief="flat", bd=0, highlightthickness=0)
        lb.pack(fill="both", expand=True, padx=1, pady=1)

        if recursos:
            for r in recursos:
                lb.insert("end", r)
        else:
            lb.insert("end", "  (no se detectaron recursos)")

        def _aplicar_seleccion_a(destino):
            sel = lb.curselection()
            if not sel:
                messagebox.showinfo("Sin selección",
                                    "Seleccione primero un recurso de la lista.",
                                    parent=dlg)
                return
            valor = lb.get(sel[0])
            if destino == "fg":
                var_fg.set(valor)
            else:
                var_wt.set(valor)

        botones_lista = tk.Frame(body, bg=C_BG)
        botones_lista.grid(row=4, column=0, columnspan=2, sticky="w", pady=(0, 6))

        tk.Button(botones_lista, text="→ FG420",
                  command=lambda: _aplicar_seleccion_a("fg"),
                  bg=BTN_NEUTRAL_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=("Segoe UI Semibold", 9), padx=10, pady=4,
                  cursor="hand2").pack(side="left", padx=(0, 6))
        tk.Button(botones_lista, text="→ WT3000",
                  command=lambda: _aplicar_seleccion_a("wt"),
                  bg=BTN_NEUTRAL_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=("Segoe UI Semibold", 9), padx=10, pady=4,
                  cursor="hand2").pack(side="left", padx=(0, 6))

        tk.Label(body,
                 text="Puede escribir la dirección manualmente si no aparece en la lista.\n"
                      "Ejemplo GPIB:  GPIB0::1::INSTR   ·   Ejemplo TCPIP: TCPIP0::192.168.1.10::inst0::INSTR",
                 bg=C_BG, fg=C_TEXT_DIM, font=("Segoe UI", 8),
                 justify="left").grid(row=5, column=0, columnspan=2, sticky="w", pady=(6, 0))

        bar = tk.Frame(dlg, bg=C_PANEL)
        bar.pack(fill="x", side="bottom")

        def _aceptar():
            self.dir_fg = var_fg.get().strip() or self.dir_fg
            self.dir_wt = var_wt.get().strip() or self.dir_wt
            self._log("=" * 60)
            self._log(f"  Direcciones actualizadas:")
            self._log(f"      FG420  → {self.dir_fg}")
            self._log(f"      WT3000 → {self.dir_wt}")
            self._log("=" * 60)
            self._set_led("desconocido")
            dlg.destroy()

        def _cancelar():
            dlg.destroy()

        def _actualizar_lista():
            lb.delete(0, "end")
            nuevos = self._listar_recursos_visa()
            if nuevos:
                for r in nuevos:
                    lb.insert("end", r)
                combo_fg.configure(values=nuevos)
                combo_wt.configure(values=nuevos)
            else:
                lb.insert("end", "  (no se detectaron recursos)")

        tk.Button(bar, text="Actualizar lista", command=_actualizar_lista,
                  bg=BTN_NEUTRAL_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=("Segoe UI Semibold", 10), padx=14, pady=7,
                  cursor="hand2").pack(side="left", padx=10, pady=8)

        tk.Button(bar, text="Cancelar", command=_cancelar,
                  bg=BTN_STOP_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=("Segoe UI Semibold", 10), padx=14, pady=7,
                  cursor="hand2").pack(side="right", padx=(6, 10), pady=8)

        tk.Button(bar, text="Aceptar", command=_aceptar,
                  bg=BTN_START_BG, fg=BTN_TEXT, relief="flat", bd=0,
                  font=("Segoe UI Semibold", 10), padx=14, pady=7,
                  cursor="hand2").pack(side="right", padx=(6, 6), pady=8)

        combo_fg.focus_set()

    # ==================================================================
    # Prueba de conexión
    # ==================================================================
    def _probar_instrumento(self, addr, nombre):
        if not addr:
            return False, "dirección vacía"
        inst = None
        try:
            if nombre == "FG420":
                from controllers.fg420controller import YokogawaFG420
                inst = YokogawaFG420(addr, mode='extreme')
            elif nombre == "WT3000":
                from controllers.wt3000controller import YokogawaWT3000
                inst = YokogawaWT3000(addr, timeout=CONEXION_TIMEOUT_MS,
                                      mode='extreme')
            else:
                return False, f"instrumento desconocido: {nombre}"

            idn = inst.conectar()
            if not idn:
                return False, "respuesta vacía a *IDN?"
            return True, idn
        except ImportError as e:
            return False, f"módulo del controlador no disponible: {e}"
        except Exception as e:
            s = str(e).lower()
            if "timeout" in s or "-1073807339" in s:
                return False, f"timeout de comunicación con {addr}"
            if "not found" in s or "-1073807343" in s:
                return False, f"recurso no encontrado ({addr})"
            if "not responding" in s:
                return False, "instrumento no responde"
            return False, f"{type(e).__name__}: {e}"
        finally:
            try:
                if inst is not None:
                    inst.desconectar()
            except Exception:
                pass

    def _conectar_instrumentos(self):
        if self.running:
            messagebox.showwarning("Prueba en curso",
                                   "Detén la prueba antes de probar conexión.")
            return
        self._set_led("probando")
        self.btn_conectar.configure(state="disabled")
        self.status.config(text="  Probando conexión con los instrumentos...")

        def worker():
            self._log_threadsafe("─" * 80)
            self._log_threadsafe("  [Prueba de conexión] Iniciando *IDN? …")
            self._log_threadsafe(f"      FG420  → {self.dir_fg}")
            self._log_threadsafe(f"      WT3000 → {self.dir_wt}")

            ok_fg, msg_fg = self._probar_instrumento(self.dir_fg, "FG420")
            if ok_fg:
                self._log_threadsafe(f"  [✓] FG420  OK → {msg_fg[:80]}")
            else:
                self._log_threadsafe(f"  [✗] FG420  FALLÓ ({self.dir_fg}): {msg_fg}")

            ok_wt, msg_wt = self._probar_instrumento(self.dir_wt, "WT3000")
            if ok_wt:
                self._log_threadsafe(f"  [✓] WT3000 OK → {msg_wt[:80]}")
            else:
                self._log_threadsafe(f"  [✗] WT3000 FALLÓ ({self.dir_wt}): {msg_wt}")

            if ok_fg and ok_wt:
                self.root.after(0, lambda: self._set_led("conectado"))
                self.root.after(0, lambda: self.status.config(
                    text="  Ambos instrumentos responden correctamente."))
            else:
                self.root.after(0, lambda: self._set_led("error"))
                fallos = []
                if not ok_fg: fallos.append(f"FG420 ({msg_fg})")
                if not ok_wt: fallos.append(f"WT3000 ({msg_wt})")
                self.root.after(0, lambda: self.status.config(
                    text="  Error de conexión: " + " | ".join(fallos)))

            self.root.after(0, lambda: self.btn_conectar.configure(state="normal"))
            self._log_threadsafe("─" * 80)

        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------------------
    def _refresh_algorithm_ui(self):
        target = getattr(self.algoritmo, "TARGET_FP", 1.0)
        self.lbl_pf_target.config(text=f"objetivo FP = {target:.4f}")

        for w in self.pf_ranges_frame.winfo_children():
            w.destroy()
        ranges = getattr(self.algoritmo, "DISPLAY_RANGES", None)
        if ranges is None:
            ranges = [("±0.01 ✓✓", C_GREEN), ("±0.03 ✓", C_GREEN),
                      ("±0.05 ◐", C_YELLOW), (">0.05 ✗", C_RED)]
        for txt, col in ranges:
            tk.Label(self.pf_ranges_frame, text=txt, bg=C_PANEL, fg=col,
                     font=("Segoe UI", 8, "bold")).pack(side="left", padx=6)

        self.lbl_pf_valor.config(text="—.————", fg=C_TEXT_DIM)
        self.lbl_pf_estado.config(text="ESPERANDO", fg=C_TEXT_DIM)
        self._actualizar_color_slider()

    def _on_alg_change_combo(self, event=None):
        self._on_alg_change_menu()

    def _set_selector_habilitado(self, habilitado):
        estado = "normal" if habilitado else "disabled"
        try:
            self.menubar.entryconfigure("Factor de Potencia", state=estado)
            self.menubar.entryconfigure("Direcciones", state=estado)
        except Exception:
            pass
        try:
            self.combo_alg.configure(
                state=("readonly" if habilitado else "disabled"))
        except Exception:
            pass
        try:
            self.btn_conectar.configure(state=estado)
        except Exception:
            pass
        self._set_slider_habilitado(habilitado)

    # ==================================================================
    # Validadores / teclado / tensión (Vrms)
    # ==================================================================
    def _validar_entry_voltaje(self, tp):
        if tp == "":
            return True
        if tp.startswith("-"):
            self.root.bell(); return False
        if tp.count(".") > 1:
            self.root.bell(); return False
        for ch in tp:
            if ch not in "0123456789.":
                self.root.bell(); return False
        return True

    def _validar_entry_paso(self, tp):
        if tp == "":
            return True
        if tp.startswith("-"):
            self.root.bell(); return False
        if tp.count(".") > 1:
            self.root.bell(); return False
        for ch in tp:
            if ch not in "0123456789.":
                self.root.bell(); return False
        return True

    def _bind_keys(self):
        self.root.bind("<Up>",   self._on_up)
        self.root.bind("<Down>", self._on_down)

    def _focus_en_paso(self):
        return self.root.focus_get() == self.entry_paso

    # ---- helper de clamp + aviso "máximo alcanzado" ----
    def _clamp_vrms(self, valor, contexto="operación"):
        try:
            v = float(valor)
        except (TypeError, ValueError):
            return self.voltaje_vrms

        if v > VOLTAJE_VRMS_MAX:
            ahora = time.time()
            if ahora - self._ultimo_aviso_max > 1.0:
                self._ultimo_aviso_max = ahora
                self.status.config(
                    text=f"  ⚠ Voltaje máximo alcanzado "
                         f"({VOLTAJE_VRMS_MAX:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                         f"≈ {VPP_MAX:.2f} Vpp).")
                self._log(
                    f"[límite] {contexto}: se solicitó "
                    f"{v:.{VOLTAJE_VRMS_DECIMALES}f} Vrms → clamp a "
                    f"{VOLTAJE_VRMS_MAX:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                    f"(máx. del FG)")
            return VOLTAJE_VRMS_MAX

        if v < VOLTAJE_VRMS_MIN:
            return VOLTAJE_VRMS_MIN

        return v

    def _on_up(self, event=None):
        if self._focus_en_paso():
            return
        paso = self._get_paso_vrms()
        nuevo = self.voltaje_vrms + paso
        clamped = self._clamp_vrms(nuevo, contexto="flecha ↑")
        self.voltaje_vrms = clamped
        self._actualizar_entry()
        self._aplicar_voltaje_actual()
        if clamped >= VOLTAJE_VRMS_MAX - 1e-9 and nuevo > VOLTAJE_VRMS_MAX:
            self.root.bell()
        return "break"

    def _on_down(self, event=None):
        if self._focus_en_paso():
            return
        paso = self._get_paso_vrms()
        nuevo = self.voltaje_vrms - paso
        if nuevo < VOLTAJE_VRMS_MIN:
            self.voltaje_vrms = VOLTAJE_VRMS_MIN
            self._actualizar_entry()
            self._aplicar_voltaje_actual()
            self.status.config(
                text=f"  ⚠ Voltaje mínimo alcanzado "
                     f"({VOLTAJE_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f} Vrms).")
            self.root.bell()
            return "break"
        self.voltaje_vrms = nuevo
        self._actualizar_entry()
        self._aplicar_voltaje_actual()
        return "break"

    def _get_paso_vrms(self):
        try:
            v = float(self.var_paso.get())
            return max(PASO_VRMS_MIN, v)
        except (ValueError, tk.TclError):
            return self._paso_prev

    def _on_manual_entry(self, event=None):
        try:
            valor = float(self.var_voltaje.get())
        except (ValueError, tk.TclError):
            valor = VOLTAJE_INICIAL_VRMS

        clamped = self._clamp_vrms(valor, contexto="entrada manual")

        if valor < VOLTAJE_VRMS_MIN:
            self.status.config(text="  ⚠ Voltaje negativo no permitido.")
            self.root.bell()

        self.voltaje_vrms = clamped
        self._actualizar_entry()
        self._aplicar_voltaje_actual()

    def _actualizar_entry(self):
        self.var_voltaje.set(
            f"{self.voltaje_vrms:.{VOLTAJE_VRMS_DECIMALES}f}")

    # ---------------- Spin dentro del Entry del paso ----------------
    def _on_paso_spin_up(self, event=None):
        try:
            actual = float(self.var_paso.get())
        except (ValueError, tk.TclError):
            actual = self._paso_prev
        nuevo = round(actual + 10.0 ** (-VOLTAJE_VRMS_DECIMALES),
                      VOLTAJE_VRMS_DECIMALES)
        self.var_paso.set(f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f}")
        return "break"

    def _on_paso_spin_down(self, event=None):
        try:
            actual = float(self.var_paso.get())
        except (ValueError, tk.TclError):
            actual = self._paso_prev
        nuevo = round(actual - 10.0 ** (-VOLTAJE_VRMS_DECIMALES),
                      VOLTAJE_VRMS_DECIMALES)
        if nuevo < PASO_VRMS_MIN:
            nuevo = PASO_VRMS_MIN
        self.var_paso.set(f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f}")
        return "break"

    def _on_paso_escape(self, event=None):
        self.var_paso.set(f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f}")
        self.status.config(text="  Edición de paso cancelada.")
        try:
            self.entry_voltaje.focus_set()
        except Exception:
            self.root.focus_set()
        return "break"

    def _on_paso_commit(self, event=None):
        if self._confirmando_paso:
            return

        txt = self.var_paso.get().strip()
        try:
            nuevo = float(txt)
        except (ValueError, tk.TclError):
            self.var_paso.set(
                f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f}")
            self._devolver_foco_a_voltaje()
            return

        if nuevo < PASO_VRMS_MIN:
            self._log(f"[paso] Valor {nuevo:.{VOLTAJE_VRMS_DECIMALES}f} Vrms "
                      f"por debajo del mínimo permitido "
                      f"({PASO_VRMS_MIN:.{VOLTAJE_VRMS_DECIMALES}f} Vrms) → "
                      f"se ajusta al mínimo.")
            nuevo = PASO_VRMS_MIN

        if abs(nuevo - self._paso_prev) < 1e-12:
            self.var_paso.set(
                f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f}")
            self._devolver_foco_a_voltaje()
            return

        self._confirmando_paso = True
        try:
            ok = messagebox.askyesno(
                "Confirmar cambio de paso",
                f"¿Confirma cambiar el paso de "
                f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f} Vrms a "
                f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f} Vrms?")
            if ok:
                self._paso_prev = nuevo
                self.var_paso.set(
                    f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f}")
                self.status.config(
                    text=f"  ✔ Paso actualizado a "
                         f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f} Vrms. "
                         f"Las flechas ↑/↓ ya aplican este paso.")
                self._log(
                    f"[paso] Nuevo paso aplicado: "
                    f"{nuevo:.{VOLTAJE_VRMS_DECIMALES}f} Vrms")
            else:
                self.var_paso.set(
                    f"{self._paso_prev:.{VOLTAJE_VRMS_DECIMALES}f}")
                self.status.config(text="  Cambio de paso cancelado.")
                self._log("[paso] Cambio cancelado por el usuario.")
        finally:
            self._confirmando_paso = False
            self._devolver_foco_a_voltaje()

    def _devolver_foco_a_voltaje(self):
        try:
            self.entry_voltaje.focus_set()
        except Exception:
            try:
                self.root.focus_set()
            except Exception:
                pass

    def _aplicar_voltaje_actual(self):
        vrms = self._clamp_vrms(self.voltaje_vrms,
                                contexto="_aplicar_voltaje_actual")
        self.voltaje_vrms = vrms

        self.lbl_voltage_top.config(
            text=f"Vrms: {vrms:.{VOLTAJE_VRMS_DECIMALES}f} V")
        if self.fg is not None:
            try:
                self._aplicar_amplitud_fg(vrms)
                self.status.config(
                    text=f"  Vrms → {vrms:.{VOLTAJE_VRMS_DECIMALES}f} V")
            except Exception as e:
                self.status.config(text=f"  [!] Error aplicando Vrms: {e}")

    def _aplicar_amplitud_fg(self, vrms):
        try:
            vrms = float(vrms)
        except (TypeError, ValueError):
            vrms = 0.0

        if vrms < VOLTAJE_VRMS_MIN:
            vrms = VOLTAJE_VRMS_MIN
        if vrms > VOLTAJE_VRMS_MAX:
            vrms = VOLTAJE_VRMS_MAX

        with self._fg_lock:
            if self.fg is None:
                return
            self.fg.escribir(f":SOURce1:VOLTage {vrms:.4f}VRMS")

    # ==================================================================
    # Apagado robusto
    # ==================================================================
    def _apagar_salidas_robusto(self):
        fg = self.fg
        if fg is None:
            return True, "sin instrumento"

        if hasattr(fg, "apagar_salidas"):
            try:
                fg.apagar_salidas()
                return True, "apagar_salidas()"
            except Exception as e:
                self._log_threadsafe(
                    f"[!] apagar_salidas() falló: {e} — probando fallback...")

        if hasattr(fg, "escribir"):
            try:
                fg.escribir(":OUTPut1 OFF")
                fg.escribir(":OUTPut2 OFF")
                return True, "escribir(:OUTPut OFF)"
            except Exception as e:
                self._log_threadsafe(
                    f"[!] escribir(:OUTPut OFF) falló: {e} — probando fallback...")

        if hasattr(fg, "_write_raw"):
            try:
                fg._write_raw(":OUTPut1 OFF;:OUTPut2 OFF")
                return True, "_write_raw(:OUTPut OFF)"
            except Exception as e:
                self._log_threadsafe(
                    f"[!] _write_raw(:OUTPut OFF) falló: {e}")

        inst = getattr(fg, "instrumento", None)
        if inst is not None:
            try:
                inst.write(":OUTPut1 OFF")
                inst.write(":OUTPut2 OFF")
                return True, "instrumento.write(:OUTPut OFF)"
            except Exception as e:
                self._log_threadsafe(
                    f"[!] instrumento.write(:OUTPut OFF) falló: {e}")

        return False, "ningún método disponible"

    def _apagar_fg_seguro(self, motivo=""):
        if self.fg is None:
            return True
        if self._shutdown_done:
            return True
        self._shutdown_done = True

        tag = f" [{motivo}]" if motivo else ""
        self._log_threadsafe(
            f"[!] Apagado seguro del FG{tag}: Vrms→0 → "
            f"{SETTLE_VPP_OFF_S*1000:.0f} ms → salida OFF → desconectar")

        with self._fg_lock:
            try:
                self._aplicar_amplitud_fg(0.0)
            except Exception as e:
                self._log_threadsafe(f"[!] No se pudo poner Vrms=0: {e}")

            try:
                time.sleep(SETTLE_VPP_OFF_S)
            except Exception:
                pass

            off_ok = False
            metodo = "?"
            for intento in range(3):
                off_ok, metodo = self._apagar_salidas_robusto()
                if off_ok:
                    break
                self._log_threadsafe(
                    f"[!] Intento {intento+1}/3 de apagar salida falló — "
                    f"reintentando...")
                time.sleep(0.1)

            if off_ok:
                self._log_threadsafe(
                    f"[✓] Salida del FG apagada correctamente (método: {metodo}).")
            else:
                self._log_threadsafe(
                    "[✗] ¡NO SE PUDO APAGAR LA SALIDA con ningún método! "
                    "Apagar manualmente el FG420.")

            try:
                self.fg.desconectar()
            except Exception as e:
                self._log_threadsafe(f"[!] Error al desconectar FG: {e}")
            finally:
                self.fg = None
        return True

    # ==================================================================
    # Logging / PF
    # ==================================================================
    def _log(self, msg):
        self.console.insert("end", msg + "\n")
        self.console.see("end")

    def _log_threadsafe(self, msg):
        self.root.after(0, lambda m=msg: self._log(m))

    def _set_stats_text(self, txt):
        self.stats_text.delete("1.0", "end")
        self.stats_text.insert("end", txt)

    def _update_pf_display(self, fp):
        target   = getattr(self.algoritmo, "TARGET_FP", 1.0)
        tol_ok   = getattr(self.algoritmo, "TOL_FP",    0.03)
        tol_half = getattr(self.algoritmo, "TOL_HALF",  tol_ok / 3.0)
        tol_loose= getattr(self.algoritmo, "TOL_LOOSE", 0.05)

        if not _es_finito(fp):
            self.lbl_pf_valor.config(text="—.————", fg=C_TEXT_DIM)
            self.lbl_pf_estado.config(text="SIN DATO", fg=C_TEXT_DIM)
            return

        if target >= 0.1:
            txt_fp = f"{fp:.4f}"
        elif target >= 0.01:
            txt_fp = f"{fp:.5f}"
        else:
            txt_fp = f"{fp:.6f}"
        self.lbl_pf_valor.config(text=txt_fp)

        err = abs(fp - target)
        if err <= tol_half:
            col, txt = C_GREEN, "✓✓ EXCELENTE"
        elif err <= tol_ok:
            col, txt = C_GREEN, "✓  EN RANGO"
        elif err <= tol_loose:
            col, txt = C_YELLOW, "◐  CERCA"
        else:
            col, txt = C_RED, "✗  FUERA"
        self.lbl_pf_valor.config(fg=col)
        self.lbl_pf_estado.config(text=txt, fg=col)

    def _limpiar(self):
        self.console.delete("1.0", "end")
        self.stats_text.delete("1.0", "end")

    # ==================================================================
    # Control
    # ==================================================================
    def start_test(self):
        if self.running:
            return

        val = int(round(float(self.var_ref.get())))
        self.ref_source = REF_CURRENT if val == REF_SLIDER_CURRENT else REF_VOLTAGE
        try:
            self.algoritmo.set_ref_source(self.ref_source)
        except Exception:
            pass

        self.voltaje_vrms = self._clamp_vrms(VOLTAJE_INICIAL_VRMS,
                                             contexto="start_test")
        self._actualizar_entry()
        self._aplicar_voltaje_actual()
        self.lbl_pf_valor.config(fg=C_TEXT_DIM, text="—.————")
        self.lbl_pf_estado.config(text="ESPERANDO", fg=C_TEXT_DIM)

        self._shutdown_done = False
        self._last_hb = 0.0
        self._limpiar()
        self.algoritmo.reset()
        try:
            self.algoritmo.set_ref_source(self.ref_source)
        except Exception:
            pass

        # ---------- Preguntar por estadísticas robustas ----------
        self.stats = None
        self._ultimo_t_rel = 0.0
        if STATS_OK:
            try:
                tomar_stats = messagebox.askyesno(
                    "Estadísticas robustas",
                    "¿Recopilar estadísticas robustas del PF en esta corrida?\n\n"
                    "Se registrarán métricas detalladas (rachas, excursiones,\n"
                    "estabilidad, dispersión, régimen, etc.). Al finalizar se\n"
                    "podrá guardar un reporte en Excel.",
                    parent=self.root,
                )
            except Exception:
                tomar_stats = False

            if tomar_stats:
                target = float(getattr(self.algoritmo, "TARGET_FP", 1.0))
                tol    = float(getattr(self.algoritmo, "TOL_FP", 0.03))
                tol_f  = float(getattr(self.algoritmo, "TOL_HALF",
                                       max(tol / 3.0, 0.01)))
                try:
                    self.stats = EstadisticasRobustas(
                        fp_target=target,
                        tolerancia=tol,
                        tolerancia_fina=tol_f,
                        prefijo_archivo=None,
                    )
                    self._log("=" * 110)
                    self._log(f"  [stats] Estadísticas robustas ACTIVADAS  "
                              f"(objetivo={target:.4f}, "
                              f"banda=±{tol:.4f}, fina=±{tol_f:.4f})")
                    self._log("=" * 110)
                except Exception as e:
                    self._log(f"[stats] No se pudo activar: {e}")
                    self.stats = None
            else:
                self._log("[stats] Estadísticas robustas DESACTIVADAS.")
        else:
            if _STATS_ERR:
                self._log(f"[stats] Módulo no disponible: {_STATS_ERR}")

        self.running = True
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self._set_selector_habilitado(False)
        self.status.config(text=f"  Iniciando prueba (ref={self.ref_source.upper()})...")

        self.hilo = threading.Thread(target=self._run_loop, daemon=True)
        self.hilo.start()

    def stop_test(self):
        if not self.running and self.fg is None:
            return
        self.running = False
        self.voltaje_vrms = 0.0
        self._actualizar_entry()
        self.lbl_voltage_top.config(
            text=f"Vrms: {0.0:.{VOLTAJE_VRMS_DECIMALES}f} V")
        self.btn_stop.configure(state="disabled")
        self.status.config(text="  Deteniendo: Vrms→0 y apagando salida...")
        try:
            self._apagar_fg_seguro(motivo="Detener")
            self.status.config(text="  Detenido. Salida apagada.")
        except Exception as e:
            self._log_threadsafe(f"[X] Error en apagado inmediato: {e}")
            self.status.config(text=f"  [!] Error en apagado: {e}")

    # ==================================================================
    # HILO DE CONTROL
    # ==================================================================
    def _run_loop(self):
        try:
            from controllers.fg420controller import YokogawaFG420
            from controllers.wt3000controller import YokogawaWT3000

            self._log_threadsafe("─" * 110)
            self._log_threadsafe("[1/4] Conectando equipos (modo EXTREME)...")
            self._log_threadsafe(f"      FG420  → {self.dir_fg}")
            self._log_threadsafe(f"      WT3000 → {self.dir_wt}")
            self._log_threadsafe(f"      Referencia → {self.ref_source.upper()}")

            try:
                self.fg = YokogawaFG420(self.dir_fg, mode='extreme')
                self.fg.conectar()
            except Exception as e:
                self._log_threadsafe(f"[X] Fallo al conectar FG420 ({self.dir_fg}): {e}")
                self.root.after(0, lambda: self._set_led("error"))
                raise

            try:
                self.wt = YokogawaWT3000(self.dir_wt,
                                         timeout=CONEXION_TIMEOUT_MS,
                                         mode='extreme')
                self.wt.conectar()
            except Exception as e:
                self._log_threadsafe(f"[X] Fallo al conectar WT3000 ({self.dir_wt}): {e}")
                self.root.after(0, lambda: self._set_led("error"))
                raise

            self._log_threadsafe(f"  FG420 : {self.fg.obtener_idn()[:80]}")
            self._log_threadsafe(f"  WT3000: {self.wt.obtener_idn()[:80]}")
            self.root.after(0, lambda: self._set_led("conectado"))

            self.fg.extreme(
                canal=1, frecuencia_hz=FREC_NOMINAL,
                amplitud_vpp=0.0, offset_v=OFFSET_V_FG,
                fase_grados=0.0, encender_salida=True,
            )
            self.wt.extreme(
                elemento_entrada=ELEMENTO_WT,
                incluir_potencias=True, configurar_salida=True,
            )
            self._log_threadsafe("  Modo EXTREME aplicado (Vrms=0 inicial).")

            self._log_threadsafe("[2/4] Estabilizando 3 s...")
            for _ in range(6):
                if not self.running:
                    break
                time.sleep(0.5)
            self._log_threadsafe("      OK")

            vrms_ini = self._clamp_vrms(self.voltaje_vrms,
                                        contexto="run_loop Vrms inicial")
            self._aplicar_amplitud_fg(vrms_ini)
            self._log_threadsafe(
                f"[3/4] Vrms inicial aplicado: "
                f"{vrms_ini:.{VOLTAJE_VRMS_DECIMALES}f} V "
                f"(máx. permitido {VOLTAJE_VRMS_MAX:.{VOLTAJE_VRMS_DECIMALES}f} V)")

            self._log_threadsafe(f"[4/4] INICIANDO CONTROL · {self.algoritmo.NOMBRE}")
            header_txt = None
            try:
                if hasattr(self.algoritmo, "header_text"):
                    header_txt = self.algoritmo.header_text()
            except Exception:
                header_txt = None
            if header_txt:
                for line in header_txt.split("\n"):
                    self._log_threadsafe(line)
            else:
                self._log_threadsafe("─" * 110)
                self._log_threadsafe(
                    f"{'t[s]':>7} | {'FP':>7} | * | {'PHI':>8} | "
                    f"{'Fase':>8} | {'Δf':>10} | {'dφ':>7} | "
                    f"{'Modo':>6} | Acción")
                self._log_threadsafe("─" * 110)

            t_ctrl = time.time()
            t_prev = t_ctrl
            errores_consec = 0
            nan_consec = 0
            n_descartadas = 0
            self._last_hb = time.time()

            while self.running and (time.time() - t_ctrl) < TIEMPO_PRUEBA_SEG:
                t_now = time.time()
                dt = t_now - t_prev
                t_prev = t_now
                t_rel = t_now - t_ctrl

                try:
                    m = self.wt.leer_mediciones_minimas()
                    errores_consec = 0
                except Exception as e:
                    errores_consec += 1
                    self._log_threadsafe(
                        f"[X] Error lectura ({errores_consec}/"
                        f"{MAX_ERRORES_LECTURA_CONSEC}): {e}")
                    if errores_consec >= MAX_ERRORES_LECTURA_CONSEC:
                        self._log_threadsafe(
                            "[X] Demasiados errores consecutivos — abortando la prueba.")
                        self.root.after(0, lambda: self._set_led("error"))
                        break
                    time.sleep(INTERVALO_MUESTREO)
                    continue

                if self.wt.is_outlier():
                    continue

                fp_med_raw   = m.get("factor_potencia")
                phi_med_raw  = m.get("angulo_fase")
                f_med_u_raw  = m.get("frecuencia")
                f_med_i_raw  = m.get("frecuencia_corriente")

                fp_med  = fp_med_raw  if _es_finito(fp_med_raw)  else None
                phi_med = phi_med_raw if _es_finito(phi_med_raw) else None
                f_med_u = f_med_u_raw if _es_finito(f_med_u_raw) else None
                f_med_i = f_med_i_raw if _es_finito(f_med_i_raw) else None

                if phi_med is None or (f_med_u is None and f_med_i is None):
                    nan_consec += 1
                    n_descartadas += 1
                    if nan_consec == 1:
                        self._log_threadsafe(
                            f"[!] Muestra con datos no finitos descartada "
                            f"(FP={fp_med_raw}, PHI={phi_med_raw}, "
                            f"FU={f_med_u_raw}, FI={f_med_i_raw})")
                    if nan_consec >= MAX_NAN_CONSEC:
                        self._log_threadsafe(
                            f"[X] {MAX_NAN_CONSEC} muestras no finitas consecutivas — "
                            f"abortando. Revise la señal de entrada o la "
                            f"configuración del WT3000.")
                        self.root.after(0, lambda: self._set_led("error"))
                        break
                    time.sleep(INTERVALO_MUESTREO)
                    continue
                else:
                    nan_consec = 0

                f_legacy = f_med_u if f_med_u is not None else f_med_i

                try:
                    comando = self.algoritmo.procesar(
                        fp_med, phi_med, f_legacy, dt, t_rel,
                        f_med_u=f_med_u, f_med_i=f_med_i,
                    )
                except TypeError as e:
                    msg = str(e)
                    if "unexpected keyword" in msg or "positional argument" in msg:
                        comando = self.algoritmo.procesar(
                            fp_med, phi_med, f_legacy, dt, t_rel)
                    else:
                        raise

                self.root.after(0, lambda v=comando["fp"]:
                                self._update_pf_display(v))

                # ---------- ESTADÍSTICAS ROBUSTAS ----------
                if self.stats is not None:
                    try:
                        _modo = (comando.get('modo', '')
                                 if isinstance(comando, dict) else '')
                        self.stats.agregar(t_rel, fp_med, phi_med, _modo)
                    except Exception as e:
                        self._log_threadsafe(
                            f"[stats] Error agregando muestra: {e}")
                self._ultimo_t_rel = t_rel

                if time.time() - self._last_hb > HEARTBEAT_S:
                    self._last_hb = time.time()
                    try:
                        _modo  = comando.get('modo', '?')
                        _fp    = comando.get('fp', float('nan'))
                        _phi   = comando.get('phi', float('nan'))
                        _fase  = comando.get('fase', 0.0)
                        _fresh = comando.get('fresh', False)
                        _frec  = comando.get('frecuencia', FREC_NOMINAL)
                        self._log_threadsafe(
                            f"[hb] t={t_rel:6.1f}s  modo={_modo:6}  "
                            f"FP={_fp:.4f}  PHI={_phi:+.4f}  "
                            f"Fase={_fase:+7.2f}°  f={_frec:.3f} Hz  "
                            f"fresh={_fresh}"
                        )
                    except Exception:
                        pass

                with self._fg_lock:
                    try:
                        self.fg.establecer_frecuencia_extreme(1, comando["frecuencia"])
                        if comando["cambio_fase"]:
                            self.fg.establecer_fase(1, comando["fase"])
                    except Exception:
                        pass

                elapsed = time.time() - t_now
                if elapsed < INTERVALO_MUESTREO:
                    time.sleep(INTERVALO_MUESTREO - elapsed)

            if n_descartadas > 0:
                self._log_threadsafe(
                    f"[i] Muestras descartadas por NaN/Inf: {n_descartadas}")

            txt = self.algoritmo.resumen_str()
            self.root.after(0, lambda t=txt: self._set_stats_text(t))
            self._log_threadsafe("")
            self._log_threadsafe("[✓] Prueba finalizada.")

        except Exception as e:
            self._log_threadsafe(f"\n[X] Error fatal: {e}")
            self._log_threadsafe(traceback.format_exc())
            self.root.after(0, lambda: self._set_led("error"))
        finally:
            try:
                self._apagar_fg_seguro(motivo="fin worker")
            except Exception:
                pass
            try:
                if self.wt is not None:
                    self.wt.desconectar()
            except Exception:
                pass
            finally:
                self.wt = None

            # --- Finalizar estadísticas robustas (reporte en consola) ---
            if self.stats is not None:
                try:
                    self.stats._cerrar(t_final=self._ultimo_t_rel)
                    self._log_threadsafe("")
                    for linea in self.stats._lineas_reporte():
                        self._log_threadsafe(linea)
                except Exception as e:
                    self._log_threadsafe(f"[stats] Error al finalizar: {e}")

            self.running = False
            self.root.after(0, self._on_test_done)

    def _on_test_done(self):
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        self._set_selector_habilitado(True)
        self.combo_alg.set(self._alg_nombre_sel)
        self.status.config(text="  Prueba finalizada. Listo.")
        if self.fg is not None:
            self._shutdown_done = False
            try:
                self._apagar_fg_seguro(motivo="backup on_test_done")
            except Exception:
                pass

        # ---------- Preguntar por guardado del reporte de stats ----------
        if self.stats is not None:
            stats_local = self.stats
            self.stats = None
            try:
                hay_datos = bool(stats_local.fp_all)
            except Exception:
                hay_datos = False

            if hay_datos:
                try:
                    guardar = messagebox.askyesno(
                        "Guardar reporte",
                        "¿Guardar el reporte de estadísticas robustas del PF "
                        "en Excel?",
                        parent=self.root,
                    )
                except Exception:
                    guardar = False

                if guardar:
                    buf = io.StringIO()
                    old_stdout = sys.stdout
                    sys.stdout = buf
                    try:
                        ok = stats_local._guardar()
                    except Exception as e:
                        ok = False
                        buf.write(f"[stats] Error al guardar: {e}\n")
                    finally:
                        sys.stdout = old_stdout

                    for line in buf.getvalue().splitlines():
                        self._log(line)
                    if ok:
                        self.status.config(text="  Reporte guardado.")
                else:
                    self._log("[stats] Reporte no guardado.")
            else:
                self._log("[stats] No hay muestras suficientes para guardar.")


# ==============================================================================
# ENTRY POINT
# ==============================================================================
def main():
    root = tk.Tk()
    MainApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()