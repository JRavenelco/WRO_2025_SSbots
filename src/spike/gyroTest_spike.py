import serial
import time
import sys

# --- Configuración ---
MOTOR_PORT_LETTER = 'A' # Letra del puerto
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
# --------------------
fd = serial.Serial(SERIAL_PORT, BAUD_RATE)
fd.write('from hub import motion_sensor\r'.encode())
m = fd.readline()
fd.write('motion_sensor.tilt_angles()\r'.encode())
m = fd.readline()
m = fd.readline()
print(m.decode())