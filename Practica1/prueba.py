import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(2)

# Verificar conexión
ser.write(b"PING\n")
print(ser.readline().decode().strip())

# Valor medio (offset)
ser.write(b"SET_PWM:127\n")
print("DAC en 127 (1.65V)")

time.sleep(5)  # Mantener 5 segundos para que midas

ser.close()