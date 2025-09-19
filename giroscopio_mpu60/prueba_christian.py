import time
import math 
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *
from buildhat import Motor
import buildhat
from signal import pause

#motors
motor_traccion = Motor('A') # Motor para la tracción de empuje diferencial
motor_direccion = Motor('D') # Motor para la dirección tipo Ackerman
motor_traccion.plimit(1)
motor_direccion.plimit(1)  
error = 0

# Clase para el Filtro de Kalman
class KalmanFilter:
    def __init__(self, Q, R, P, x0):
        self.Q = Q  # Covarianza del ruido del proceso
        self.R = R  # Covarianza del ruido de la medición
        self.P = P  # Covarianza de la estimación de error
        self.x = x0 # Estado inicial (ángulo de Yaw)

    def predict(self, dt):
        # Predicción del estado y la covarianza
        # Para un ángulo de Yaw, el modelo es simple: x_k = x_{k-1}
        # P_k = P_{k-1} + Q
        self.P += self.Q * dt # Multiplicar Q por dt para escalar el ruido del proceso con el tiempo

    def update(self, measurement):
        # Actualización del estado usando la medición
        # Ganancia de Kalman: K = P / (P + R)
        K = self.P / (self.P + self.R)

        # Actualización del estado: x = x + K * (medicion - x)
        self.x = self.x + K * (measurement - self.x)

        # Actualización de la covarianza de error: P = (1 - K) * P
        self.P = (1 - K) * self.P

        return self.x

# Configuración del Giroscopio (IMU)
print("Inicializando IMU...")
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G
)
mpu.configure()

# Variables para el cálculo del ángulo de Yaw y calibración del giroscopio
last_time = time.time()
yaw_angle = 0.0
# Inicializar filtro de Kalman (ajusta Q y R según sea necesario)
kalman_filter = KalmanFilter(Q=0.01, R=0.5, P=1.0, x0=yaw_angle)
gyro_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0} # Almacenar los offsets de calibración

def calibrar_giroscopio(num_muestras=2000, tiempo_entre_muestras=0.001):
    """Calibración mejorada con detección de movimiento"""
    print(f"Calibrando con {num_muestras} muestras... (¡Mantener el sensor quieto!)")
    offsets = {'x': [], 'y': [], 'z': []}
    
    # Tomar muestras
    for _ in range(num_muestras):
        try:
            gx, gy, gz = mpu.readGyroscopeMaster()
            offsets['x'].append(gx)
            offsets['y'].append(gy)
            offsets['z'].append(gz)
            time.sleep(tiempo_entre_muestras)
        except Exception as e:
            print(f"Error en calibración: {e}")
            continue
    
    # Calcular media y desviación estándar
    for axis in offsets:
        media = sum(offsets[axis]) / len(offsets[axis])
        std = (sum((x - media) ** 2 for x in offsets[axis]) / len(offsets[axis])) ** 0.5
        print(f"Eje {axis}: Media={media:.4f} °/s, DesvEst={std:.4f} °/s")
        gyro_offsets[axis] = media
    
    print(f"Offsets finales: {gyro_offsets}")
    return gyro_offsets

# Variables globales
ultimo_giro = 0
tiempo_ultimo_giro = time.time()
umbral_giro = 1.0  # °/s para considerar que hay movimiento

def leer_giroscopio_con_compensacion():
    global yaw_angle, ultimo_giro, tiempo_ultimo_giro
    
    try:
        # Leer giroscopio
        gx, gy, gz = mpu.readGyroscopeMaster()
        gz_compensado = gz - gyro_offsets['z']
        
        # Detección de movimiento
        ahora = time.time()
        dt = ahora - tiempo_ultimo_giro
        tiempo_ultimo_giro = ahora
        
        # Si no hay movimiento significativo, reducir la deriva
        if abs(gz_compensado) < umbral_giro:
            factor_compensacion = 0.01  # Ajusta este valor según sea necesario
            yaw_angle *= (1 - factor_compensacion * dt)
        else:
            # Integrar cuando hay movimiento real
            yaw_angle += gz_compensado * dt
        
        # Aplicar filtro de Kalman
        kalman_filter.predict(dt)
        yaw_filtrado = kalman_filter.update(yaw_angle)
        
        return yaw_filtrado
        
    except Exception as e:
        print(f"Error en lectura: {e}")
        return None

def esta_quieto(umbral=0.1, num_muestras=10):
    """Usa el acelerómetro para detectar si el robot está quieto"""
    muestras = []
    for _ in range(num_muestras):
        try:
            ax, ay, az = mpu.readAccelerometerMaster()
            muestras.append((ax, ay, az))
            time.sleep(0.01)
        except:
            return False
    
    # Calcular varianza
    varianzas = [np.var([m[i] for m in muestras]) for i in range(3)]
    return all(v < umbral for v in varianzas)

def leer_giroscopio():
    """Lee el giroscopio y devuelve el ángulo de Yaw filtrado con Kalman.
    Aplica los offsets de calibración y usa un filtro de Kalman para reducir el ruido.
    """
    global yaw_angle, last_time, kalman_filter
    try:
        gyro_data = mpu.readGyroscopeMaster()
        # Aplicar offsets de calibración
        gyro_z = gyro_data[2] - gyro_offsets['z']

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Integrar la velocidad angular Z para obtener el cambio de ángulo de Yaw
        yaw_angle += gyro_z * dt

        # Filtrado de Kalman
        kalman_filter.predict(dt)
        yaw_angle_filtrado = kalman_filter.update(yaw_angle)

        return yaw_angle_filtrado
    except Exception as e:
        print(f"Error al leer giroscopio: {e}")
        return None\

def reiniciar_giroscopio():
    """
    Reinicia el giroscopio a su estado inicial.
    Esto incluye:
    - Reiniciar el ángulo Yaw a 0
    - Reiniciar el filtro de Kalman
    - Actualizar la última marca de tiempo
    """
    global yaw_angle, last_time, kalman_filter
    
    # 1. Reiniciar el ángulo Yaw
    yaw_angle = 0.0
    
    # 2. Actualizar la última marca de tiempo para evitar saltos en la integración
    last_time = time.time()
    
    # 3. Reiniciar el filtro de Kalman (opcional, dependiendo de tu implementación)
    # Si usas la misma instancia pero con valores iniciales
    kalman_filter.x = 0.0  # Reiniciar el estado
    kalman_filter.P = 1.0  # Reiniciar la covarianza del error
    
    print("Giroscopio reiniciado a 0 grados")
    return True
    

def pd(s1,s2,vel,kp,kd,ea,posComp):
    error=s1-s2
    a = (motor_direccion.get_position() + posComp)*(6.4)
    giro = 0
    if a > error:
        giro = error - a
    if a < error:
        giro = a - error
    giro = (kp*giro) + (kd*(giro-ea))
    if giro > 1:
        giro = 1
    if giro < -1:
        giro = -1
    motor_direccion.pwm(giro)     
    print(a,giro,s1,giro)
    return error



try:
    calibrar_giroscopio()
    reiniciar_giroscopio()
    print("Iniciando lectura continua del giroscopio. Presiona Ctrl+C para salir.")
    motor_direccion.set_speed_unit_rpm(True)
    motor_direccion.set_default_speed(100)
    tiempo_inicial = time.time()
    while time.time() - tiempo_inicial < 5:
        motor_direccion.run_to_position(0,100,True)
    #buildhat.BuildHAT.resethat()
    
    posComp = motor_direccion.get_position() - motor_direccion.get_aposition()
    print(posComp)
    #time.sleep(10)
    while True:
        angulo_yaw = leer_giroscopio()
        """if angulo_yaw is not None:
            print(f"Yaw: {angulo_yaw:.2f} grados")"""
        #time.sleep(0.05) # Pequeña pausa para controlar la frecuencia de lectura      
        error = pd(angulo_yaw,0,0.009,0.0005,0,error,posComp)
        while angulo_yaw > 360 or angulo_yaw < -360:
            angulo_yaw = 0  

except KeyboardInterrupt:
    print("Lectura de giroscopio detenida por el usuario.")
except Exception as e:
    print(f"Ocurrió un error inesperado: {e}")