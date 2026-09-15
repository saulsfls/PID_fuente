"""
benchmark_instrumentos.py

Mide la tasa máxima de actualización del FG420 y de lectura del WT3000
en cada uno de los modos soportados (extreme, streaming, fast, balanced,
precise).

También mide la tasa combinada del lazo de control (FG write + WT read).

Uso:
    python benchmark_instrumentos.py
    python benchmark_instrumentos.py --dur 5 --warmup 1.0 --canal 1
    python benchmark_instrumentos.py --solo fg
    python benchmark_instrumentos.py --solo wt
"""
import argparse
import time
import numpy as np
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000


DIR_FG = "GPIB1::2::INSTR"
DIR_WT = "GPIB0::1::INSTR"

MODOS = ('extreme', 'streaming', 'fast', 'balanced', 'precise')


# ==============================================================================
# UTILIDADES
# ==============================================================================
def stats_latencia(times_s):
    if not times_s:
        return None
    arr = np.asarray(times_s) * 1e3
    return {
        "n": int(len(arr)),
        "mean_ms": float(np.mean(arr)),
        "std_ms": float(np.std(arr)),
        "p50_ms": float(np.percentile(arr, 50)),
        "p95_ms": float(np.percentile(arr, 95)),
        "p99_ms": float(np.percentile(arr, 99)),
        "max_ms": float(np.max(arr)),
    }


def separador(char="=", n=110):
    print(char * n)


def print_header(titulo):
    print()
    separador("=")
    print(f" {titulo}")
    separador("=")


def print_tabla_header():
    print(f"{'Modo':<11} | {'Dur':>6} | {'Ops':>7} | {'Tasa(Hz)':>9} | "
          f"{'Lat med':>9} | {'Lat p50':>9} | {'Lat p95':>9} | "
          f"{'Lat p99':>9} | {'Lat max':>9}")
    print("-" * 110)


def print_tabla_fila(modo, dur, n, rate, stats):
    if stats is None:
        print(f"{modo:<11} | {dur:>5.2f}s | {n:>7d} | {rate:>8.2f}  | "
              f"(sin latencias)")
        return
    print(f"{modo:<11} | {dur:>5.2f}s | {n:>7d} | {rate:>8.2f}  | "
          f"{stats['mean_ms']:>7.3f}ms | {stats['p50_ms']:>7.3f}ms | "
          f"{stats['p95_ms']:>7.3f}ms | {stats['p99_ms']:>7.3f}ms | "
          f"{stats['max_ms']:>7.3f}ms")


# ==============================================================================
# BENCHMARKS
# ==============================================================================
def benchmark_fg420(fg, modo, duracion_s, canal):
    """Tasa de escritura de frecuencia para un modo dado."""
    fg.set_mode(modo, sleep_scale=1.0)

    if modo in ('extreme', 'streaming'):
        def op(i):
            fg.establecer_frecuencia_extreme(canal, 60.0 + 0.001 * (i % 5))
    else:
        def op(i):
            fg.establecer_frecuencia(canal, 60.0 + 0.001 * (i % 5))

    # Warmup
    for i in range(20):
        op(i)

    latencias = []
    n = 0
    t0 = time.perf_counter()
    t_end = t0 + duracion_s
    while time.perf_counter() < t_end:
        ta = time.perf_counter()
        op(n)
        latencias.append(time.perf_counter() - ta)
        n += 1
    elapsed = time.perf_counter() - t0
    return n, elapsed, n / elapsed, stats_latencia(latencias)


def benchmark_wt3000(wt, modo, duracion_s):
    """Tasa de lectura mínima para un modo dado."""
    wt.set_mode(modo, sleep_scale=1.0)
    if wt._salida_config != "minima":
        wt.configurar_salida_minima(elemento_entrada=1, incluir_potencias=True)

    # Warmup
    for _ in range(10):
        try:
            wt.leer_mediciones_minimas()
        except Exception:
            pass

    latencias = []
    n = 0
    errores = 0
    t0 = time.perf_counter()
    t_end = t0 + duracion_s
    while time.perf_counter() < t_end:
        ta = time.perf_counter()
        try:
            wt.leer_mediciones_minimas()
            latencias.append(time.perf_counter() - ta)
            n += 1
        except Exception:
            errores += 1
    elapsed = time.perf_counter() - t0
    return n, errores, elapsed, n / max(elapsed, 1e-9), stats_latencia(latencias)


def benchmark_lazo_combinado(fg, wt, modo, duracion_s, canal):
    """FG write + WT read en el mismo bucle (lo que ve el PLL)."""
    fg.set_mode(modo, sleep_scale=1.0)
    wt.set_mode(modo, sleep_scale=1.0)
    if wt._salida_config != "minima":
        wt.configurar_salida_minima(elemento_entrada=1, incluir_potencias=True)

    # Warmup
    for i in range(10):
        if modo in ('extreme', 'streaming'):
            fg.establecer_frecuencia_extreme(canal, 60.0)
        else:
            fg.establecer_frecuencia(canal, 60.0)
        try:
            wt.leer_mediciones_minimas()
        except Exception:
            pass

    latencias = []
    n = 0
    t0 = time.perf_counter()
    t_end = t0 + duracion_s
    while time.perf_counter() < t_end:
        ta = time.perf_counter()
        if modo in ('extreme', 'streaming'):
            fg.establecer_frecuencia_extreme(canal, 60.0 + 0.001 * (n % 5))
        else:
            fg.establecer_frecuencia(canal, 60.0 + 0.001 * (n % 5))
        try:
            wt.leer_mediciones_minimas()
            latencias.append(time.perf_counter() - ta)
            n += 1
        except Exception:
            pass
    elapsed = time.perf_counter() - t0
    return n, elapsed, n / max(elapsed, 1e-9), stats_latencia(latencias)


# ==============================================================================
# MAIN
# ==============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="Benchmark FG420 + WT3000")
    p.add_argument("--dur", type=float, default=5.0,
                   help="Duración de cada prueba (s). Por defecto 5.")
    p.add_argument("--warmup", type=float, default=1.0,
                   help="Estabilización inicial antes del primer test (s).")
    p.add_argument("--canal", type=int, default=1,
                   help="Canal del FG420 (1 o 2). Por defecto 1.")
    p.add_argument("--solo", choices=["fg", "wt", "loop", "todos"],
                   default="todos",
                   help="Ejecutar solo un grupo de pruebas.")
    return p.parse_args()


def main():
    args = parse_args()

    print()
    separador("=")
    print(" BENCHMARK DE INSTRUMENTOS - FG420 / WT3000")
    print(f" Duración por prueba: {args.dur:.2f}s | Canal FG: {args.canal}")
    separador("=")

    fg = None
    wt = None

    try:
        print("\n[1/4] Conectando FG420...")
        fg = YokogawaFG420(DIR_FG, mode='extreme')
        fg.conectar()

        print("[2/4] Conectando WT3000...")
        wt = YokogawaWT3000(DIR_WT, mode='extreme')
        wt.conectar()

        print("[3/4] Configurando FG420 (60 Hz, 5 Vpp, salida ON)...")
        fg.extreme(canal=args.canal, frecuencia_hz=60.0,
                   amplitud_vpp=5.0, offset_v=0.0,
                   fase_grados=0.0, encender_salida=True)

        print("[4/4] Configurando WT3000 (salida numérica mínima)...")
        wt.extreme(elemento_entrada=1, incluir_potencias=True,
                   configurar_salida=True)

        print(f"\n Estabilizando {args.warmup:.1f}s...")
        time.sleep(args.warmup)

        # ------------------------------------------------------------------
        # FG420
        # ------------------------------------------------------------------
        if args.solo in ("fg", "todos"):
            print_header("FG420 - Tasa de actualización (escritura de frecuencia)")
            print_tabla_header()
            for modo in MODOS:
                try:
                    n, el, rate, st = benchmark_fg420(
                        fg, modo, args.dur, args.canal)
                    print_tabla_fila(modo, el, n, rate, st)
                except Exception as e:
                    print(f"{modo:<11} | ERROR: {e}")

        # ------------------------------------------------------------------
        # WT3000
        # ------------------------------------------------------------------
        if args.solo in ("wt", "todos"):
            print_header("WT3000 - Tasa de lectura (mediciones mínimas)")
            print_tabla_header()
            for modo in MODOS:
                try:
                    n, err, el, rate, st = benchmark_wt3000(
                        wt, modo, args.dur)
                    print_tabla_fila(modo, el, n, rate, st)
                    if err:
                        print(f"            ({err} errores de lectura)")
                except Exception as e:
                    print(f"{modo:<11} | ERROR: {e}")

        # ------------------------------------------------------------------
        # Lazo combinado
        # ------------------------------------------------------------------
        if args.solo in ("loop", "todos"):
            print_header("LAZO COMBINADO - FG write + WT read (lo que ve el PLL)")
            print_tabla_header()
            for modo in MODOS:
                try:
                    n, el, rate, st = benchmark_lazo_combinado(
                        fg, wt, modo, args.dur, args.canal)
                    print_tabla_fila(modo, el, n, rate, st)
                except Exception as e:
                    print(f"{modo:<11} | ERROR: {e}")

        # ------------------------------------------------------------------
        # Notas
        # ------------------------------------------------------------------
        print()
        separador("-")
        print(" Notas:")
        print("  - 'extreme' y 'streaming' comparten sleeps=0 en ambos equipos,")
        print("    por lo que sus tasas deben ser idénticas.")
        print("  - En FG420 'precise' se ejecuta *OPC? tras cada write de freq,")
        print("    lo que añade una latencia significativa.")
        print("  - En WT3000 'balanced' (avg=4) y 'precise' (avg=16) el valor")
        print("    devuelto corresponde a un promedio; la tasa de queries no")
        print("    cambia, pero la novedad real del dato sí.")
        separador("-")

    except KeyboardInterrupt:
        print("\n[!] Interrumpido por el usuario.")

    except Exception as e:
        print(f"\n[X] Error fatal: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[!] Apagando...")
        if fg is not None:
            try:
                fg.establecer_salida(args.canal, False)
            except Exception:
                pass
            try:
                fg.desconectar()
            except Exception:
                pass
        if wt is not None:
            try:
                wt.desconectar()
            except Exception:
                pass
        print("[OK] Listo.")


if __name__ == "__main__":
    main()