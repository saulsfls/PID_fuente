"""
medir_latencia.py
Mide la latencia del lazo WT3000 -> proceso -> FG420.
Reporta estadísticos por tramo y por combinación de modos.
"""
import time
import statistics as st
import pyvisa
from controllers.fg420controller import YokogawaFG420
from controllers.wt3000controller import YokogawaWT3000

GPIB_WT = "GPIB0::1::INSTR" #Wattmetro
GPIB_FG = "GPIB1::2::INSTR" #Generador de onda arbitraria

def percentiles(xs, ps=(50, 90, 95, 99, 100)):
    s = sorted(xs)
    n = len(s)
    return {p: s[min(n - 1, int(round(p / 100 * n)) - 1)] for p in ps}


def medir_latencia(n_iter=300, modo_wt='fast', modo_fg='fast', pausa_entre=0.03):
    """Mide latencias por tramo y totales."""
    wt = YokogawaWT3000(GPIB_WT, mode=modo_wt)
    fg = YokogawaFG420(GPIB_FG, mode=modo_fg)
    wt.conectar()
    fg.conectar()
    wt.configurar_salida_numerica_estandar(elemento_entrada=1)

    # Aseguramos salida activa a 60 Hz para no cambiar el estado físico
    fg.configurar_canal(1, "SIN", 60.0, 5.0, 0.0, 0.0, activar_salida=True)

    t_query, t_proc, t_write, t_total = [], [], [], []

    # Precalentamiento (descartamos primeras N para evitar JIT/cache de VISA)
    for _ in range(10):
        m = wt.leer_mediciones_estandar()
        fg.establecer_frecuencia(1, m["frecuencia"])
        time.sleep(0.02)

    for i in range(n_iter):
        t0 = time.perf_counter()
        m = wt.leer_mediciones_estandar()
        t1 = time.perf_counter()

        f_medida = m["frecuencia"] or 60.0
        # Simulamos el cálculo del PLL: en el lazo real aquí va el PI
        f_nueva = f_medida + 0.001 * ((i % 5) - 2)  # perturbación pequeña
        t2 = time.perf_counter()

        fg.establecer_frecuencia(1, f_nueva)
        t3 = time.perf_counter()

        t_query.append((t1 - t0) * 1e3)
        t_proc.append((t2 - t1) * 1e3)
        t_write.append((t3 - t2) * 1e3)
        t_total.append((t3 - t0) * 1e3)

        time.sleep(pausa_entre)

    fg.establecer_frecuencia(1, 60.0)
    fg.desconectar()
    wt.desconectar()

    def resumen(nombre, xs):
        return {
            "tramo": nombre,
            "mean_ms": st.mean(xs),
            "std_ms": st.stdev(xs) if len(xs) > 1 else 0.0,
            "min_ms": min(xs),
            **{f"p{p}_ms": v for p, v in percentiles(xs).items()},
        }

    print(f"\n=== Modo WT={modo_wt} | FG={modo_fg} | N={n_iter} ===")
    filas = [
        resumen("WT query (mediciones)", t_query),
        resumen("Procesamiento SW", t_proc),
        resumen("FG write", t_write),
        resumen("TOTAL SW->SW", t_total),
    ]
    for r in filas:
        print(
            f"{r['tramo']:<24} "
            f"mean={r['mean_ms']:7.2f}  std={r['std_ms']:6.2f}  "
            f"min={r['min_ms']:6.2f}  "
            f"p50={r['p50_ms']:6.2f}  p95={r['p95_ms']:6.2f}  "
            f"p99={r['p99_ms']:6.2f}  max={r['p100_ms']:6.2f}"
        )
    return filas


if __name__ == "__main__":
    print("Benchmark de latencia WT3000 -> FG420")
    for modo_wt, modo_fg in [("fast", "fast"),
                             ("balanced", "fast"),
                             ("fast", "balanced"),
                             ("balanced", "balanced"),
                             ("precise", "precise")]:
        try:
            medir_latencia(n_iter=200, modo_wt=modo_wt, modo_fg=modo_fg)
        except Exception as e:
            print(f"  [X] {modo_wt}/{modo_fg}: {e}")