import serial
import time
import sys

# --- Configuración ---
MOTOR_PORT_LETTER = 'B' # Letra del puerto
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
# --------------------
fd = serial.Serial(SERIAL_PORT, BAUD_RATE)
#fd.write('from hub import light_matrix\r'.encode())
fd.write('import motor\r'.encode())
fd.write('from hub import port\r'.encode())
#fd.write('light_matrix.write("Hi!")\r'.encode())
fd.write('motor.run(port.B, 1000)\r'.encode())
time.sleep(3)
fd.write('motor.stop(port.B)\r'.encode())