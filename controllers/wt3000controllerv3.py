import time
import struct
import pyvisa

# 1. Configuración de conexión PyVISA
# Reemplaza la dirección según tu tipo de conexión (GPIB, Ethernet/Socket o USB)
# Ejemplo GPIB: 'GPIB0::14::INSTR'
# Ejemplo Ethernet Socket: 'TCPIP0::192.168.1.100::7000::SOCKET'
VISA_ADDRESS = 'GPIB0::1::INSTR'

rm = pyvisa.ResourceManager()

try:
    instrument = rm.open_resource(VISA_ADDRESS)
    
    # Ajustes del canal de comunicación VISA
    instrument.timeout = 5000  # 5 segundos
    instrument.write_termination = '\n'
    instrument.read_termination = '\n'
    
    print(f"Conectado a: {instrument.query('*IDN?').strip()}")

    # 2. Configuración para Máxima Velocidad
    # Establecer la tasa de actualización al mínimo posible (50 ms)
    instrument.write(":NUMeric:FORMAT REAL")         # Transferencia binaria de punto flotante
    instrument.write(":RATE 50MS")                  # Tasa de actualización a 50 ms
    
    # Configurar los ítems numéricos a leer en la pantalla/patrón activo
    # Configuramos elementos básicos para medir rápido en el patrón/matriz
    instrument.write(":NUMeric:ITEM1 U,1")          # Tensión RMS Canal 1
    instrument.write(":NUMeric:ITEM2 I,1")          # Corriente RMS Canal 1
    instrument.write(":NUMeric:ITEM3 P,1")          # Potencia Activa Canal 1
    instrument.write(":NUMeric:ITEM4 S,1")          # Potencia Aparente Canal 1
    instrument.write(":NUMeric:ITEM5 Q,1")          # Potencia Reactiva Canal 1
    instrument.write(":NUMeric:ITEM6 LAMBda,1")     # Factor de Potencia (PF) Canal 1
    
    # Indicar cuántos ítems se solicitarán en total
    instrument.write(":NUMeric:NUMber 6")

    print("\nIniciando adquisición continua de datos a alta velocidad...")
    print("Presiona Ctrl+C para detener.\n")

    # Variables de control
    last_status = None

    # 3. Bucle de adquisición continua de alta velocidad
    while True:
        # Preguntar si hay un nuevo ciclo de actualización completo (Update Flag)
        status = int(instrument.query(":STATus:CONDition?").strip())
        
        # Verificar el bit de Data Update (Bit 0 o cambios en la condición)
        # Solicitar el bloque de valores procesados
        raw_data = instrument.query_binary_values(":NUMeric:VALue?", datatype='f', is_big_endian=True)
        
        if raw_data:
            u1, i1, p1, s1, q1, pf1 = raw_data[:6]
            
            # Imprimir parámetros actualizados en consola
            print(f"[{time.strftime('%H:%M:%S.%3N')}] "
                  f"V: {u1:8.3f} V | "
                  f"I: {i1:8.4f} A | "
                  f"P: {p1:8.3f} W | "
                  f"S: {s1:8.3f} VA | "
                  f"Q: {q1:8.3f} var | "
                  f"PF: {pf1:6.4f}")

except KeyboardInterrupt:
    print("\nAdquisición detenida por el usuario.")
except Exception as e:
    print(f"\nError de comunicación: {e}")
finally:
    if 'instrument' in locals():
        instrument.close()
    rm.close()
    print("Conexión VISA cerrada.")