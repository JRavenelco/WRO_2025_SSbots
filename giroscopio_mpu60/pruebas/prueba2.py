import time
import math
import smbus2
import numpy as np

# Configuración I2C
I2C_BUS = 1
MPU_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C

# Registros MPU9250
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
INT_PIN_CFG = 0x37
WHO_AM_I = 0x75

# Registros AK8963
MAG_CNTL = 0x0A
MAG_XOUT_L = 0x03
AK8963_WHO_AM_I = 0x00

# Inicializar bus I2C
bus = smbus2.SMBus(I2C_BUS)

# Parámetros de filtrado
ALPHA = 0.2  # Factor de suavizado para acelerómetro
GYRO_WEIGHT = 0.98  # Peso del giroscopio en filtro complementario
MAG_ALPHA = 0.1  # Factor de suavizado para magnetómetro
WINDOW_SIZE = 10  # Tamaño de ventana para promedio móvil

class KalmanFilter:
    """Filtro de Kalman simplificado para una dimensión"""
    def __init__(self, process_variance, measurement_variance, initial_value=0):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.error_estimate = 1.0
        
    def update(self, measurement):
        # Predicción
        error_prediction = self.error_estimate + self.process_variance
        
        # Actualización
        kalman_gain = error_prediction / (error_prediction + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1 - kalman_gain) * error_prediction
        
        return self.estimate

def mpu_init():
    """Inicializar el sensor MPU9250 con secuencia robusta"""
    # Reset completo
    bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0x80)
    time.sleep(0.1)
    
    # Despertar y configurar reloj
    bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    
    # Configurar giroscopio (±1000°/s)
    bus.write_byte_data(MPU_ADDRESS, GYRO_CONFIG, 0x10)
    
    # Configurar acelerómetro (±8g)
    bus.write_byte_data(MPU_ADDRESS, ACCEL_CONFIG, 0x10)
    
    # Configurar filtro paso bajo (DLPF)
    bus.write_byte_data(MPU_ADDRESS, CONFIG, 0x03)
    time.sleep(0.01)
    
    # Habilitar modo bypass para magnetómetro
    bus.write_byte_data(MPU_ADDRESS, INT_PIN_CFG, 0x02)
    time.sleep(0.1)
    
    # Configurar magnetómetro (modo continua, 100Hz, 16-bit)
    try:
        bus.write_byte_data(AK8963_ADDRESS, MAG_CNTL, 0x16)
        time.sleep(0.01)
    except:
        print("Advertencia: Problema con magnetómetro, continuando...")

def read_raw_data(addr):
    """Leer datos crudos de 2 bytes con manejo de errores"""
    try:
        high = bus.read_byte_data(MPU_ADDRESS, addr)
        low = bus.read_byte_data(MPU_ADDRESS, addr + 1)
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value
    except:
        return 0

def read_accel():
    """Leer acelerómetro en g's con suavizado básico"""
    x = read_raw_data(ACCEL_XOUT_H) * 8.0 / 32768.0
    y = read_raw_data(ACCEL_XOUT_H + 2) * 8.0 / 32768.0
    z = read_raw_data(ACCEL_XOUT_H + 4) * 8.0 / 32768.0
    return x, y, z

def read_gyro():
    """Leer giroscopio en grados/segundo"""
    x = read_raw_data(GYRO_XOUT_H) * 1000.0 / 32768.0
    y = read_raw_data(GYRO_XOUT_H + 2) * 1000.0 / 32768.0
    z = read_raw_data(GYRO_XOUT_H + 4) * 1000.0 / 32768.0
    return x, y, z

def read_mag():
    """Leer magnetómetro con suavizado"""
    try:
        data = bus.read_i2c_block_data(AK8963_ADDRESS, MAG_XOUT_L, 7)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536
        
        return x, y, z
    except:
        return 0, 0, 0

def calibrate_sensor(samples=400):
    """Calibración avanzada con eliminación de outliers"""
    print("Calibrando... Mantén el sensor plano y quieto")
    
    # Listas para almacenar muestras
    ax_samples, ay_samples, az_samples = [], [], []
    mx_samples, my_samples = [], []
    
    # Recopilar muestras
    for i in range(samples):
        ax, ay, az = read_accel()
        mx, my, mz = read_mag()
        
        ax_samples.append(ax)
        ay_samples.append(ay)
        az_samples.append(az)
        mx_samples.append(mx)
        my_samples.append(my)
        
        time.sleep(0.01)
        if i % 50 == 0:
            print(f"Progreso: {i*100/samples:.0f}%")
    
    # Eliminar outliers (valores fuera de 2 desviaciones estándar)
    def remove_outliers(data):
        mean = np.mean(data)
        std = np.std(data)
        return [x for x in data if abs(x - mean) < 2 * std]
    
    ax_clean = remove_outliers(ax_samples)
    ay_clean = remove_outliers(ay_samples)
    az_clean = remove_outliers(az_samples)
    mx_clean = remove_outliers(mx_samples)
    my_clean = remove_outliers(my_samples)
    
    # Calcular offsets con muestras limpias
    accel_offset = (np.mean(ax_clean), np.mean(ay_clean), np.mean(az_clean))
    mag_offset = (np.mean(mx_clean), np.mean(my_clean))
    
    print("\nCalibración completada!")
    print(f"Acel Offset: X={accel_offset[0]:.4f}, Y={accel_offset[1]:.4f}, Z={accel_offset[2]:.4f}")
    print(f"Mag Offset: X={mag_offset[0]:.1f}, Y={mag_offset[1]:.1f}")
    
    return accel_offset, mag_offset

def calculate_angles(ax, ay, az):
    """Calcular Roll y Pitch a partir del acelerómetro"""
    roll = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180/math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180/math.pi
    return roll, pitch

def calculate_yaw(mx, my):
    """Calcular Yaw a partir del magnetómetro"""
    yaw = math.atan2(my, mx) * 180/math.pi
    return yaw + 360 if yaw < 0 else yaw

# Variables globales para offsets del giroscopio
gyro_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}

def calibrate_gyro(samples=500):
    """Calibra el giroscopio calculando los offsets de los ejes X, Y, Z.
    Se recomienda mantener el sensor completamente inmóvil durante la calibración.
    """
    global gyro_offsets
    print(f"Iniciando calibración del giroscopio con {samples} muestras...")
    sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
    for i in range(samples):
        try:
            gx, gy, gz = read_gyro()
            sum_x += gx
            sum_y += gy
            sum_z += gz
            time.sleep(0.001) # Pequeña pausa para no saturar el bus I2C
        except Exception as e:
            print(f"Error durante la lectura de calibración del giroscopio: {e}")
            continue
        if i % 100 == 0:
            print(f"Progreso calibración giroscopio: {i*100/samples:.0f}%")

    gyro_offsets['x'] = sum_x / samples
    gyro_offsets['y'] = sum_y / samples
    gyro_offsets['z'] = sum_z / samples
    print(f"Calibración del giroscopio completada. Offsets: {gyro_offsets}")

# Inicialización
print("Iniciando sensor...")
mpu_init()
time.sleep(0.5)  # Tiempo de estabilización
print("Sensor inicializado correctamente")

# Calibración
accel_off, mag_off = calibrate_sensor(300)
calibrate_gyro(500) # Calibrar el giroscopio

# Inicializar variables para filtros
last_time = time.time()
roll_gyro, pitch_gyro = 0, 0

# Filtros Kalman para cada ángulo
# Inicializamos los filtros con un estado de 0 para que los ángulos comiencen en 0
roll_filter = KalmanFilter(0.01, 0.1, initial_value=0.0)
pitch_filter = KalmanFilter(0.01, 0.1, initial_value=0.0)
yaw_filter = KalmanFilter(0.05, 0.2, initial_value=0.0)

# Buffer para promedio móvil
angle_buffer = []

initial_yaw_offset = 0.0
calibrated_initial_yaw = False

try:
    print("\nPresiona Ctrl+C para detener")
    print(" Roll   Pitch   Yaw")
    print("--------------------")
    
    while True:
        # Calcular delta tiempo
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Leer sensores
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        mx, my, mz = read_mag()
        
        # Aplicar offsets
        ax -= accel_off[0]
        ay -= accel_off[1]
        az -= accel_off[2] - 1.0  # Compensar gravedad
        mx -= mag_off[0]
        my -= mag_off[1]
        
        # Aplicar offsets del giroscopio
        gx -= gyro_offsets['x']
        gy -= gyro_offsets['y']
        gz -= gyro_offsets['z']
        
        # Calcular ángulos crudos
        roll_accel, pitch_accel = calculate_angles(ax, ay, az)
        yaw_raw = calculate_yaw(my, mx)
        
        # Calibrar el offset inicial del Yaw solo una vez
        if not calibrated_initial_yaw:
            initial_yaw_offset = yaw_raw
            calibrated_initial_yaw = True
            print(f"Yaw inicial calibrado a: {initial_yaw_offset:.2f} grados.")
            # Forzar el estado inicial del filtro de Yaw a 0
            yaw_filter.state = 0.0

        # Aplicar el offset inicial al Yaw
        yaw_raw -= initial_yaw_offset
        # Normalizar el Yaw después de aplicar el offset para mantenerlo en un rango de 0-360 o -180 a 180
        yaw_raw = yaw_raw % 360
        if yaw_raw > 180: # Opcional: si prefieres el rango -180 a 180
            yaw_raw -= 360

        # Filtro complementario para Roll y Pitch
        roll_gyro += gx * dt
        pitch_gyro += gy * dt
        
        roll_comp = GYRO_WEIGHT * (roll_gyro) + (1 - GYRO_WEIGHT) * roll_accel
        pitch_comp = GYRO_WEIGHT * (pitch_gyro) + (1 - GYRO_WEIGHT) * pitch_accel
        
        # Actualizar estado para próxima iteración
        roll_gyro, pitch_gyro = roll_comp, pitch_comp
        
        # Aplicar filtro Kalman
        roll_filtered = roll_filter.update(roll_comp)
        pitch_filtered = pitch_filter.update(pitch_comp)
        yaw_filtered = yaw_filter.update(yaw_raw)
        
        # Normalizar Yaw final para mostrarlo en 0-360
        yaw_filtered = yaw_filtered % 360
        if yaw_filtered < 0: # Asegurarse de que sea positivo
            yaw_filtered += 360

        # Promedio móvil con ventana adaptable
        angle_buffer.append((roll_filtered, pitch_filtered, yaw_filtered))
        if len(angle_buffer) > WINDOW_SIZE:
            angle_buffer.pop(0)
        
        roll_avg = sum(r for r, p, y in angle_buffer) / len(angle_buffer)
        pitch_avg = sum(p for r, p, y in angle_buffer) / len(angle_buffer)
        yaw_avg = sum(y for r, p, y in angle_buffer) / len(angle_buffer)
        
        # Mostrar resultados
        print(f"\r{roll_avg:>6.1f} {pitch_avg:>6.1f} {yaw_avg:>6.1f}", end='', flush=True)
        time.sleep(0.05)  # Frecuencia de muestreo óptima

except KeyboardInterrupt:
    print("\nPrograma detenido")