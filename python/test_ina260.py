import pyvisa
rm = pyvisa.ResourceManager('@py')
print(rm.list_resources())
samd21 = rm.open_resource('USB0::51966::16384::123459::0::INSTR')
print(samd21.query("*IDN?"))
print(samd21.query("INA260:VOLTAGE?"))
print(samd21.query("INA260:CURRENT?"))
print(samd21.query("INA260:POWER?"))
samd21.write("INA260:RST")
