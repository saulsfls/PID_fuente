import pyvisa

rm = pyvisa.ResourceManager()
print("Recursos GPIB detectados:")
print(rm.list_resources())