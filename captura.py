import time
from datetime import datetime
import openpyxl
import pandas as pd
import pyvisa

def parse_float(val_str):
    """Convierte cadenas a float, manejando valores de fuera de rango u 'OVER' típicos de analizadores."""
    try:
        val = float(val_str)
        # Los analizadores Yokogawa suelen entregar valores como 9.9E37 cuando hay sobreescala
        if val > 1e30 or val < -1e30:
            return None
        return val
    except ValueError:
        return None
    
def exportar_a_excel(datos, nombre_archivo):
    """Genera un archivo Excel con formato profesional a partir de los datos recolectados."""
    if not datos:
        print("No se registraron datos para exportar.")
        return

    df = pd.DataFrame(datos)

    # Crear el escritor de Excel con openpyxl
    with pd.ExcelWriter(nombre_archivo, engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="Lecturas", index=False)

        # Acceder al libro y hoja de trabajo para aplicar formato
        workbook = writer.book
        worksheet = writer.sheets["Lecturas"]

        # Estilos visuales
        from openpyxl.styles import Alignment, Border, Font, PatternFill, Side

        header_fill = PatternFill(
            start_color="1F4E78", end_color="1F4E78", fill_type="solid"
        )
        header_font = Font(name="Calibri", size=11, bold=True, color="FFFFFF")
        data_font = Font(name="Calibri", size=10)

        thin_border = Border(
            left=Side(style="thin", color="D9D9D9"),
            right=Side(style="thin", color="D9D9D9"),
            top=Side(style="thin", color="D9D9D9"),
            bottom=Side(style="thin", color="D9D9D9"),
        )

        # Formatear Encabezados
        for col_num, col_name in enumerate(df.columns, 1):
            cell = worksheet.cell(row=1, column=col_num)
            cell.fill = header_fill
            cell.font = header_font
            cell.alignment = Alignment(
                horizontal="center", vertical="center", wrap_text=True
            )

        # Formatear Celdas de Datos
        for row in range(2, len(df) + 2):
            for col in range(1, len(df.columns) + 1):
                cell = worksheet.cell(row=row, column=col)
                cell.font = data_font
                cell.border = thin_border
                cell.alignment = Alignment(vertical="center")

                # Formatos numéricos específicos por columna
                col_title = df.columns[col - 1]
                if "Tiempo" in col_title:
                    cell.alignment = Alignment(horizontal="center")
                elif any(
                    k in col_title
                    for k in ["Voltaje", "Corriente", "Frecuencia"]
                ):
                    cell.number_format = "0.0000"
                    cell.alignment = Alignment(horizontal="right")
                elif any(k in col_title for k in ["Potencia", "Ángulo"]):
                    cell.number_format = "0.00"
                    cell.alignment = Alignment(horizontal="right")
                elif "Factor" in col_title:
                    cell.number_format = "0.000"
                    cell.alignment = Alignment(horizontal="right")

        # Agregar filas de resumen estadístico (Promedio, Mínimo, Máximo)
        last_data_row = len(df) + 1
        stats = [("Promedio", "AVERAGE"), ("Mínimo", "MIN"), ("Máximo", "MAX")]

        stat_font = Font(name="Calibri", size=10, bold=True)
        stat_fill = PatternFill(
            start_color="F2F2F2", end_color="F2F2F2", fill_type="solid"
        )

        for idx, (label, func) in enumerate(stats, start=1):
            stat_row = last_data_row + idx
            # Etiqueta en la primera columna
            cell_label = worksheet.cell(row=stat_row, column=1, value=label)
            cell_label.font = stat_font
            cell_label.fill = stat_fill

            # Fórmulas para las columnas numéricas
            for col in range(2, len(df.columns) + 1):
                col_letter = openpyxl.utils.get_column_letter(col)
                formula = (
                    f"={func}({col_letter}2:{col_letter}{last_data_row})"
                )
                cell_stat = worksheet.cell(
                    row=stat_row, column=col, value=formula
                )
                cell_stat.font = stat_font
                cell_stat.fill = stat_fill
                cell_stat.border = thin_border
                cell_stat.alignment = Alignment(horizontal="right")

        # Auto-ajustar ancho de columnas
        for col in worksheet.columns:
            max_len = max(len(str(cell.value or "")) for cell in col)
            col_letter = openpyxl.utils.get_column_letter(col[0].column)
            worksheet.column_dimensions[col_letter].width = max(max_len + 3, 12)

    print(f"\n[✔] Reporte guardado exitosamente en: {nombre_archivo}")

def main():
    rm = pyvisa.ResourceManager()

    # Cambia esta dirección según la configuración de tu equipo en NI MAX
    resource_name = "GPIB1::1::INSTR"

    # Lista para almacenar las lecturas
    registros = []

    try:
        inst = rm.open_resource(resource_name)
        inst.timeout = 5000  # 5 segundos
        inst.write_termination = "\n"
        inst.read_termination = "\n"

        # Verificar identificación del equipo
        idn = inst.query("*IDN?")
        print(f"Conectado a: {idn.strip()}")

        # Configuración del formato de salida en el analizador
        inst.write(":NUMeric:FORMAT ASCII")
        inst.write(":NUMeric:NUMBER 12")  # Indicar que se enviarán 12 ítems

        # Configuración de los 12 elementos de medición
        inst.write(":NUMeric:ITEM1 U,1")  # Voltaje RMS (V)
        inst.write(":NUMeric:ITEM2 UMN,1")  # Voltaje Medio (V)
        inst.write(":NUMeric:ITEM3 UDC,1")  # Voltaje DC (V)
        inst.write(":NUMeric:ITEM4 I,1")  # Corriente RMS (A)
        inst.write(":NUMeric:ITEM5 IMN,1")  # Corriente Media (A)
        inst.write(":NUMeric:ITEM6 IDC,1")  # Corriente DC (A)
        inst.write(":NUMeric:ITEM7 P,1")  # Potencia Activa (W)
        inst.write(":NUMeric:ITEM8 S,1")  # Potencia Aparente (VA)
        inst.write(":NUMeric:ITEM9 Q,1")  # Potencia Reactiva (VAR)
        inst.write(":NUMeric:ITEM10 LAMBda,1")  # Factor de Potencia (PF)
        inst.write(":NUMeric:ITEM11 PHI,1")  # Ángulo de Fase (deg)
        inst.write(":NUMeric:ITEM12 FU,1")  # Frecuencia de Voltaje (Hz)

        print(
            "\n--- Iniciando adquisición de datos (Presiona Ctrl+C para finalizar y guardar) ---"
        )
        print(
            f"{'Tiempo':<10} | {'V RMS (V)':<10} | {'I RMS (A)':<10} | {'P Activa (W)':<12} | {'F.P.':<8} | {'Frec (Hz)':<10}"
        )
        print("-" * 70)

        while True:
            t_actual = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            data_raw = inst.query(":NUMeric:VALue?")
            values = data_raw.strip().split(",")

            if len(values) >= 12:
                # Extracción y conversión de los 12 datos configurados
                u_rms = parse_float(values[0])
                u_mn = parse_float(values[1])
                u_dc = parse_float(values[2])
                i_rms = parse_float(values[3])
                i_mn = parse_float(values[4])
                i_dc = parse_float(values[5])
                p_activa = parse_float(values[6])
                s_aparente = parse_float(values[7])
                q_reactiva = parse_float(values[8])
                pf = parse_float(values[9])
                phi = parse_float(values[10])
                frec = parse_float(values[11])

                # Guardar registro en la lista
                registros.append(
                    {
                        "Tiempo": t_actual,
                        "Voltaje RMS (V)": u_rms,
                        "Voltaje Medio (V)": u_mn,
                        "Voltaje DC (V)": u_dc,
                        "Corriente RMS (A)": i_rms,
                        "Corriente Media (A)": i_mn,
                        "Corriente DC (A)": i_dc,
                        "Potencia Activa (W)": p_activa,
                        "Potencia Aparente (VA)": s_aparente,
                        "Potencia Reactiva (VAR)": q_reactiva,
                        "Factor de Potencia": pf,
                        "Ángulo de Fase (°)": phi,
                        "Frecuencia (Hz)": frec,
                    }
                )

                # Mostrar valores principales en consola
                v_str = f"{u_rms:10.4f}" if u_rms is not None else "      OVER"
                i_str = f"{i_rms:10.4f}" if i_rms is not None else "      OVER"
                p_str = (
                    f"{p_activa:12.4f}" if p_activa is not None else "        OVER"
                )
                pf_str = f"{pf:8.4f}" if pf is not None else "    OVER"
                f_str = f"{frec:10.4f}" if frec is not None else "      OVER"

                print(
                    f"{t_actual:<10} | {v_str} | {i_str} | {p_str} | {pf_str} | {f_str}"
                )

            time.sleep(0.5)

    except pyvisa.VisaIOError as e:
        print(f"\n[X] Error de comunicación GPIB: {e}")
    except KeyboardInterrupt:
        print("\n[!] Adquisición detenida por el usuario.")
    finally:
        # Cerrar conexión GPIB
        if "inst" in locals():
            inst.close()
        rm.close()

        # Generar reporte Excel si se capturaron datos
        if registros:
            nombre_reporte = (
                f"Reporte_WT3000_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
            )
            exportar_a_excel(registros, nombre_reporte)


if __name__ == "__main__":
    main()