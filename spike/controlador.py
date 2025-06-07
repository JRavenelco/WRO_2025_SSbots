import serial
import time
import sys

# --- Configuración ---
MOTOR_PORT_LETTER = 'A'  # Letra del puerto
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
# --------------------

# Parámetros del controlador
Kp = 0.5  # Ganancia proporcional
target_angle = 0  # Ángulo objetivo para avanzar recto

fd = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Inicializar motores y giroscopio
fd.write('import motor\r'.encode())
fd.write('from hub import port\r'.encode())
fd.write('from hub import motion_sensor\r'.encode())
time.sleep(1)  # Espera a que los módulos se importen

def get_gyro_angle():
    fd.write('motion_sensor.get_gyroscope_data()\r'.encode())
    time.sleep(0.1)
    response = fd.readline().decode().strip()
    try:
        g_x, g_y, g_z = map(float, response.split(','))
        return g_z  # Asumiendo que el eje Z es el que controla la dirección
    except:
        return 0.0

try:
    while True:
        current_angle = get_gyro_angle()
        error = target_angle - current_angle
        correction = Kp * error

        left_speed = 1000 + correction
        right_speed = 1000 - correction

        # Asegurar que las velocidades estén dentro de los límites permitidos
        left_speed = max(min(left_speed, 1000), -1000)
        right_speed = max(min(right_speed, 1000), -1000)

        # Enviar comandos al motor
        fd.write(f'motor.run(port.A, {int(left_speed)}, {int(right_speed)})\r'.encode())
        time.sleep(0.1)  # Intervalo de actualización

except KeyboardInterrupt:
    # Detener los motores al finalizar
    fd.write('motor.stop(port.A)\r'.encode())
    fd.close()
    sys.exit()