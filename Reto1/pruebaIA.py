
#!/usr/bin/env python3
import time, queue, serial, subprocess, os, pty, threading, lgpio, board, adafruit_vl53l1x
from dataclasses import dataclass
from math import floor
from rplidar import RPLidar, RPLidarException
import numpy as np


# Data classes
@dataclass
class LidarData:
    front: int
    left: int
    deg_75: int
    deg_80: int
    deg_85: int
    deg_95: int
    right: int
    deg_285: int
    deg_280: int
    deg_275: int
    deg_265: int

@dataclass
class ToFSensorData:
    left: int
    right: int
    rear: int

@dataclass
class ToFSensor:
    device: adafruit_vl53l1x.VL53L1X
    name: str
    address: int
    gpio_pin: int

# Consistent GPIO pin numbers (use original: 16, 20, 21)
TOF_SENSORS = [
    ToFSensor(None, "left", address=0x26, gpio_pin=17),
    ToFSensor(None, "right", address=0x28, gpio_pin=27),
    ToFSensor(None, "rear", address=0x29, gpio_pin=22)
]

RUNNING = True
lidar_queue: "queue.Queue[LidarData]" = queue.Queue(maxsize=1)
tof_queue: "queue.Queue[ToFSensorData]" = queue.Queue(maxsize=1)

PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME)
i2c = board.I2C()
spike = None
RIGHT = 1
LEFT = -1

def screen_simplified():
    if not os.path.exists("/dev/ttyACM0"):
        return

    master_fd, slave_fd = pty.openpty() 
    
    subprocess.Popen( 
        ["screen", "/dev/ttyACM0", "115200"],
        stdin=slave_fd, 
        stdout=slave_fd,
        stderr=slave_fd,
    )

    os.close(slave_fd)
    time.sleep(1.0)
    os.write(master_fd, b'\x03') # Send Ctrl+C to screen
    time.sleep(0.2)
    os.write(master_fd, b'\x01') # Send Ctrl+A to screen
    time.sleep(0.2)
    os.write(master_fd, b'k') # Send 'k' to kill the screen session
    time.sleep(0.2)
    os.write(master_fd, b'y') # Send 'y' to confirm kill
    time.sleep(0.2)
    
    os.close(master_fd)

#se corre el screen para el spike

    
#se define la variable para el spike
spike = None
#variables globales rasp
der = 1
izq = -1

#Todas las funciones que se mandan por serial para el spike
def initialize_Libraries():
    spike.write("import motor\r".encode()) 
    spike.readline()
    spike.write("from hub import port\r".encode())
    spike.readline()#clear buffer
    spike.write("from hub import motion_sensor\r".encode())
    spike.readline()#clear buffer
    spike.write("import distance_sensor\r".encode())
    spike.readline()#clear buffer
    spike.write("import runloop\r".encode())
    spike.readline()#clear buffer
    #declare varianbles globales spike
    spike.write("der = -1\r".encode())
    spike.readline() #clear buffer
    spike.write("izq = 1\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    #declare functions for motors
    spike.write("def Hold():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.F, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.B, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def fc():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.F, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.B, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    # centrar el vehiculo normal
    spike.write("async def cv_especial():\r".encode())
    spike.readline() #clear buffer 
    spike.write("await motor.run_to_absolute_position(port.F, 0, 630,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("direction = motor.LONGEST_PATH, stop = motor.HOLD, acceleration = 1000, deceleration = 1000)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("runloop.run(cv_especial())\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()

    # centrar el vehiculo corto
    spike.write("def cvc():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.run_to_absolute_position(port.F, 0, 630,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 1000, deceleration = 1000)\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #pd 
    spike.write("def pd(s1,s2,vel,kp,kd,ea):\r".encode()) #ea es error anterior
    spike.readline() #clear buffer 
    spike.write("error=s1-s2\r".encode())
    spike.readline()#clear buffer
    spike.write("et= (kp*error) + (kd*(error-ea))\r".encode()) #et es error total
    spike.readline()#clear buffer
    spike.readline() #clear buffer
    spike.write("motor.run_to_absolute_position(port.F, int(et*6.4), 630, direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 10000)\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(vel))\r".encode())
    spike.readline() #clear buffer
    spike.write("return error\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #reset Gyro
    spike.write("def rg(grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motion_sensor.reset_yaw(grados)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #imprimir giroscopio
    spike.write("def pg():\r".encode())
    spike.readline() #clear buffer
    spike.write("return motion_sensor.tilt_angles()[0]\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #avanzar derecho
    spike.write("def ad(vel,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.5,10,error)\r".encode())
    spike.readline() #clear buffer 
    end_Function()

    spike.write("def vuelta(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_relative_position(port.F, 313*(direccion), 630)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(velocidad))\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())   
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def da(vel, referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(motion_sensor.tilt_angles()[0],referencia,vel,0.5,10,error)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def ag(vel,grados,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.reset_relative_position(port.B,0)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados) > abs(motor.relative_position(port.B)):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(motion_sensor.tilt_angles()[0],((10)*(referencia)),vel,0.5,10,error)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def spd(vel,distancia,sensor):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(distancia,sensor,vel,0.5,10,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.B)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def spi(vel,distancia,sensor):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(sensor,distancia,vel,0.5,10,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.B)\r".encode())
    spike.readline() #clear buffer
    end_Function()

#se hacen las funciones en la rasp que utilizan las funciones mandadas por serial
def centrar_vehiculo():
    spike.write("cv()\r".encode())
    spike.readline() #limpia el buffer 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"


def centrar_vehiculo_corto():
    spike.write("cvc()\r".encode())
    spike.readline() #limpia el buffer 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"

def Hold_motors():
    spike.write("Hold()\r".encode())
    spike.readline() #clear buffer

def Coast_motors():
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer


def end_Function():
    spike.write("\r".encode())
    spike.readline()#clear 
    spike.write("\r".encode())
    spike.readline()#clear buffer
    spike.write("\r".encode())
    spike.readline()#clear buffer
    
def reset_gyro(grados):
    spike.write(("rg("+str(grados)+")\r").encode())
    spike.readline() #clear buffer

def print_gyro():
    spike.write("pg()\r".encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    print("grioscopio: ",return_value)
    
def avanzar_distancia(vel,distancia,referencia):
    #avanzar cierta distancia sensro del fente, creo que no se ocupo 
    pass

def vuelta_grados(direccion,velocidad,grados):
    spike.write(("vuelta("+str(direccion)+","+str(velocidad)+","+str(grados)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la vuelta")
    Coast_motors()

def avanzar_detection_lidar_tof(vel, referencia):
    try:
        while not tof_queue.full():
            print("WAITING FOR TOF QUEUE\n")
        obj = tof_queue.get()
        while obj.left is not None and obj.right is not None:
            spike.write((f"da({vel},{referencia})\r").encode())
            spike.readline()
            if tof_queue.full():
                print("TOF queue received")
                obj = tof_queue.get()
            time.sleep(0.001)
            print(obj.left, obj.right, obj.rear)
        Coast_motors()
        if obj.left is None:
            print(obj.left)
            return LEFT
        elif obj.right is None:
            print(obj.right)
            return RIGHT
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

def avanzar_detection_lidar(vel, referencia):
    try:
        while not lidar_queue.full():
            print("WAITING FOR LIDAR QUEUE\n")
        obj = lidar_queue.get()
        while (obj.left < 1350 and obj.right < 1350) or obj.front > 1100:
            spike.write((f"da({vel},{referencia})\r").encode())
            spike.readline()
            if lidar_queue.full():
                obj = lidar_queue.get()
            time.sleep(0.001)
            print(obj.left, obj.right, obj.front)
        Coast_motors()
        if obj.left > 1350:
            print(obj.left)
            return LEFT
        elif obj.right > 1350:
            print(obj.right)
            return RIGHT
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()
        
def avanzar_detection_left_lidar(vel, referencia):
    try:
        while not lidar_queue.full():
            print("WAITING FOR LIDAR QUEUE LEFT\n")
        obj = lidar_queue.get()
        while not tof_queue.full():
            print("WAITING FOR TOF\n")
        obj_tof = tof_queue.get()
        while obj.left < 1350 or obj_tof.rear < 175:
            spike.write((f"da({vel},{referencia})\r").encode())
            spike.readline()
            if lidar_queue.full():
                obj = lidar_queue.get()
            if tof_queue.full():
                obj_tof = tof_queue.get()
            if obj_tof.rear is None:
                obj_tof.rear = 175
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()
    
def avanzar_detection_right_lidar(vel, referencia):
    try:
        while not lidar_queue.full():
            print("WAITING FOR LIDAR QUEUE RIGHT\n")
        obj = lidar_queue.get()
        while not tof_queue.full():
            print("WAITING FOR TOF\n")
        obj_tof = tof_queue.get()
        while obj.right < 1350 or obj_tof.rear < 175:
            spike.write((f"da({vel},{referencia})\r").encode())
            spike.readline()
            if lidar_queue.full():
                obj = lidar_queue.get()
            if tof_queue.full():
                obj_tof = tof_queue.get()
            if obj_tof.rear is None:
                obj_tof.rear = 175
            print(obj.right, obj_tof.rear)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

def vuelta_automatica(velocidad, grados):
    der = 1
    izq = -1
    while not lidar_queue.full():
        print("ESPERANDO EL QUEVE PARA LA VUELTA\n")
    obj = lidar_queue.get()
    print(obj.dist_right, obj.dist_left)
    if obj.dist_right > 1350:
        vuelta_grados(der,velocidad,grados)
    elif obj.dist_left > 1350:
        vuelta_grados(izq,velocidad,grados)
    Coast_motors()

def avanzar_recto_grados(velocidad,grados,referencia):
    spike.write(("ag("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("FIN DE LA FUNCION AVANZAR RECTO GRADOS")
    Coast_motors()

def seguir_pared_derecho(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor derecho
    pass

def seguir_pared_izquierdo(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor izquierdo
    pass
    
#funcion para el lidar que obtiene las medidas en angulos especificos
def RPlidar_worker():
    try:
        lidar.stop()
        time.sleep(0.2)
        lidar.clean_input()
        angles = [0]*360
        for scan in lidar.iter_scans(scan_type='express',max_buf_meas= 4000,min_len=0):
            for (_, angle, distance) in scan:
                angles[min([359,floor(angle)])] = distance
            lidar_data = LidarData(angles[0],angles[90],angles[75], angles[80],angles[85],angles[95],angles[270],angles[285],angles[280],angles[275],angles[265])
            if lidar_queue.full(): #check if the queue is full
                try:
                    #print("queue full lidar listo del worker")
                    lidar_queue.get()
                except lidar_queue.not_empty:
                    pass
            lidar_queue.put(lidar_data)
            #vision_queue.task_done()
    except RPLidarException as e:
        print("error in lidar",e)
        lidar.clean_input()

#convercion de grados a radianes
def grados_a_radianes(grados):
    grados_convertidos : int = (grados)/(180/np.pi)
    return grados_convertidos

#convercion de radianes a grados
def radianes_a_grados(radianes):
    radianes_convertidos : int = (radianes)*(180/np.pi)
    return radianes_convertidos

#funcion que calcula la correcion del giroscopio utilizando medidas del lidar y funciones trigonométricas
def correccion_izquierda():
    #todas las funciones trigonométricas usan radianes como entrada
    if lidar_queue.full():
        try:
            obj = lidar_queue.get_nowait()
            H = int(obj.dist_75*10) #Distancia a 75 grados
            H_80 = int(obj.dist_80*10)
            CA = int(obj.dist_left*10) #distancia izquierda del lidar
            L_85 = int(obj.dist_85*10)
            L_95 = int(obj.dist_95*10)
            print(CA,H, H_80, L_85, L_95)

            if (H == 0 or H > 5000) and H_80 == 0:
                lado_faltante = np.sqrt((L_85**2) + (L_95**2)- (2*L_85*L_95*np.cos(grados_a_radianes(10))))
                ley_de_senos = L_85*np.sin(grados_a_radianes(10))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = L_95/L_85
                if comparacion < 1 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 170-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-85)*-10)
                print("tercer triangulo")

            elif H == 0 or H > 5000:
                lado_faltante = np.sqrt((H_80**2) + (CA**2)- (2*H_80*CA*np.cos(grados_a_radianes(10))))
                ley_de_senos = H_80*np.sin(grados_a_radianes(10))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = radianes_a_grados(np.arcsin(CA/H_80))
                if comparacion < 80 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 180-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-90)*-10)
                print("segundo triangulo")

            else:
                #calculo con 90 y 80 grados
                lado_faltante = np.sqrt((H**2) + (CA**2)- (2*H*CA*np.cos(grados_a_radianes(15))))
                ley_de_senos = H*np.sin(grados_a_radianes(15))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = radianes_a_grados(np.arcsin(CA/H))
                if comparacion < 75 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 180-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-90)*-10)
                print("primer triangulo")      
            
            if angulo_correccion == None:
                angulo_correccion = 0
            elif angulo_correccion > 250:
                angulo_correccion = 250
            elif angulo_correccion < -250:
                angulo_correccion = -250
            print("angulo correccion: ",angulo_correccion)
            return angulo_correccion
        except queue.Empty:
            pass
    
    else:
        return None

def correccion_derecha(correcion_del_angulo):
    #todas las funciones trigonométricas usan radianes como entrada
    if lidar_queue.full():
        try:
            obj = lidar_queue.get_nowait()
            H = int(obj.dist_285*10) #Distancia a 75 grados
            H_280 = int(obj.dist_280*10)
            CA = int(obj.dist_right*10) #distancia izquierda del lidar
            L_265 = int(obj.dist_265*10)
            L_275 = int(obj.dist_275*10)
            print(CA,H, H_280, L_275, L_265)

            if (H == 0 or H > 5000) and H_280 == 0:
                lado_faltante = np.sqrt((L_265**2) + (L_275**2)- (2*L_265*L_275*np.cos(grados_a_radianes(10))))
                ley_de_senos = L_265*np.sin(grados_a_radianes(10))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = L_265/L_275
                if comparacion < 1 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 170-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-85)*10)
                print("tercer triangulo")

            elif H == 0 or H > 5000:
                lado_faltante = np.sqrt((H_280**2) + (CA**2)- (2*H_280*CA*np.cos(grados_a_radianes(10))))
                ley_de_senos = H_280*np.sin(grados_a_radianes(10))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = radianes_a_grados(np.arcsin(CA/H_280))
                if comparacion < 80 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 180-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-90)*10)
                print("segundo triangulo")

            else:
                #calculo con 90 y 80 grados
                lado_faltante = np.sqrt((H**2) + (CA**2)- (2*H*CA*np.cos(grados_a_radianes(15))))
                ley_de_senos = H*np.sin(grados_a_radianes(15))
                angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
                comparacion = radianes_a_grados(np.arcsin(CA/H))
                if comparacion < 75 or comparacion == np.nan: #si el robot esta inclinado hacia afuera
                    angulo_correccion = 180-angulo_correccion
                else:
                    angulo_correccion = angulo_correccion
                angulo_correccion = int((angulo_correccion-90)*10)
                print("primer triangulo")      
            
            if angulo_correccion == None:
                angulo_correccion = 0
            elif angulo_correccion > 250:
                angulo_correccion = 250
            elif angulo_correccion < -250:
                angulo_correccion = -250
            print("angulo correccion: ",angulo_correccion)
            return angulo_correccion
        except queue.Empty:
            pass
    
    else:
        return None

#funcion que habilita el acceso a los gpio
def setup_gpio():
    try:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, 4)
        lgpio.gpio_claim_output(h, 12)
        lgpio.gpio_claim_output(h, 17)
        lgpio.gpio_write(h, 12, 1)
        lgpio.gpio_write(h, 17, 1)
        for sensor in TOF_SENSORS:
            lgpio.gpio_claim_output(h, sensor.gpio_pin)
            lgpio.gpio_write(h, sensor.gpio_pin, 0)
            time.sleep(0.5)
        for sensor in TOF_SENSORS:
            lgpio.gpio_write(h, sensor.gpio_pin, 1)
            time.sleep(0.5)
            sensor.device = adafruit_vl53l1x.VL53L1X(i2c)
            print(sensor.device.model_info)
            sensor.device.distance_mode = 2
            sensor.device.timing_budget = 33
            sensor.device.roi_center = 199
            print(sensor.device.roi_center)
            sensor.device.roi_xy = (16,8)
            print(sensor.device.roi_xy)
            sensor.device.set_address(sensor.address)
        lgpio.gpio_write(h, 12, 0)
        print("GPIO initialized")
        return h, True
    except Exception as e:
        print(f"Error initializing sensors: {e}")
        return None, False

#funcion que limpia los gpio
def cleanup_gpio(h):
    try:
        for sensor in TOF_SENSORS:
            try:
                lgpio.gpio_free(h, sensor.gpio_pin)
            except Exception:
                pass
        lgpio.gpio_free(h, 4)
        lgpio.gpio_write(h, 12, 1)
        lgpio.gpio_write(h, 17, 1)
        lgpio.gpiochip_close(h)
        print("GPIO released")
    except Exception:
        pass

def sensor_worker():
    for sensor in TOF_SENSORS:
        sensor.device.start_ranging()
    obj = ToFSensorData(0, 0, 0)
    distances = [0, 0, 0]
    while True:
        try:
            for i, sensor in enumerate(TOF_SENSORS):
                if sensor.device.data_ready:
                    distances[i] = sensor.device.distance
                    sensor.device.clear_interrupt()
            obj.left = distances[0]
            obj.right = distances[1]
            obj.rear = distances[2]
            if tof_queue.full():
                try:
                    tof_queue.get()
                except Exception:
                    pass
            if obj.left or obj.right or obj.rear:
                tof_queue.put(obj)
                tof_queue.task_done()
            time.sleep(0.01)
        except Exception as e:
            print(f"Error reading sensor: {e}")

def main():
    global running, spike
    h, success = setup_gpio()
    try:
        spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        screen_simplified()
        initialize_Libraries()
        threading.Thread(target=sensor_worker).start()
        threading.Thread(target=RPlidar_worker).start()
        reset_gyro(0)
        time.sleep(0.1)
        print_gyro()
        print("presiona el boton\n")
        while(lgpio.gpio_read(h, 4) == 1):
            lgpio.gpio_write(h, 17, 1)
            time.sleep(0.05)
            lgpio.gpio_write(h, 17, 0)
            time.sleep(0.05)
        lgpio.gpio_write(h, 17, 0)
        reset_gyro(0)
        print("empezando ....\n")
        
        while running:
            try:
                #para el lidar
                """
                if vision_queue.full():
                    try:
                        a=0
                    except queue.Empty:
                        pass
                angulo_correccion = correccion_izquierda()
                if angulo_correccion != None:
                    print(angulo_correccion)
                #prubas con mas medidas de seguridad
                while not vision_queue.full():
                    print("esperando lidar\n")
                avanzar_detection_derecho_lidar(60,0)
                vuelta_grados(der,60,88)
                centrar_vehiculo()
                avanzar_recto_grados(60,600,-90)

                #programa principal
                """

                while not lidar_queue.full():
                    print("waiting for lidar\n")
                obj_lidar = lidar_queue.get()
                v = 0

                if obj_lidar.dist_left > 700 or obj_lidar.dist_right > 700:
                    
                    direccion = avanzar_detection_lidar(90,0)
                    time.sleep(1)
                    avanzar_recto_grados(90,300,0) # se va a aquitar con sensores

                    if direccion == izq:
                        vuelta_grados(izq,75,88)
                        centrar_vehiculo()
                        avanzar_recto_grados(90,1600,90)
                        angulo_correcion = correccion_izquierda()
                        reset_gyro(angulo_correcion)
                        time.sleep(0.1)
                        while v < 11:
                            avanzar_detection_left_lidar(90,0)
                            avanzar_recto_grados(90,300,90) # se va a aquitar con sensores
                            vuelta_grados(izq,75,88)
                            centrar_vehiculo()
                            avanzar_recto_grados(90,900,90)
                            reset_gyro(0)
                            angulo_correcion = correccion_izquierda()
                            reset_gyro(angulo_correcion)
                            print_gyro()
                            time.sleep(0.1)
                            v = v + 1
                        avanzar_recto_grados(90,700,0)
                    
                    if direccion == der:
                        vuelta_grados(der,90,88)
                        centrar_vehiculo()
                        avanzar_recto_grados(90,1600,-90)
                        angulo_correcion = correccion_derecha(15)
                        reset_gyro(angulo_correcion)
                        time.sleep(0.1)
                        while v < 11:
                            avanzar_detection_right_lidar(90,0)
                            vuelta_grados(der,75,88)
                            centrar_vehiculo()
                            avanzar_recto_grados(90,900,-90)
                            reset_gyro(0)
                            angulo_correcion = correccion_derecha(15)
                            reset_gyro(angulo_correcion)
                            time.sleep(0.1)
                            v = v + 1
                        avanzar_recto_grados(90,700,0)
                
                else:
                    
                    direccion = avanzar_detection_lidar(90,0)
                    avanzar_recto_grados(90,300,0) # se va a aquitar con sensores

                    if direccion == izq:
                        vuelta_grados(izq,75,88)
                        centrar_vehiculo()
                        avanzar_recto_grados(90,900,90)
                        angulo_correcion = correccion_izquierda()
                        reset_gyro(angulo_correcion)
                        time.sleep(0.1)
                        while v < 11:
                            avanzar_detection_left_lidar(90,0)
                            avanzar_recto_grados(90,300,90) # se va a aquitar con sensores
                            vuelta_grados(izq,75,88)
                            centrar_vehiculo()
                            avanzar_recto_grados(90,900,90)
                            reset_gyro(0)
                            angulo_correcion = correccion_izquierda()
                            reset_gyro(angulo_correcion)
                            time.sleep(0.1)
                            v = v + 1
                        avanzar_recto_grados(90,700,0)
                    
                    if direccion == der:
                        vuelta_grados(der,75,88)
                        centrar_vehiculo()
                        avanzar_recto_grados(90,900,-90)
                        angulo_correcion = correccion_derecha(15)
                        reset_gyro(angulo_correcion)
                        time.sleep(0.1)
                        while v < 11:
                            avanzar_detection_right_lidar(90,0)
                            vuelta_grados(der,75,88)
                            centrar_vehiculo()
                            avanzar_recto_grados(90,900,-90)
                            reset_gyro(0)
                            angulo_correcion = correccion_derecha(15)
                            reset_gyro(angulo_correcion)
                            time.sleep(0.1)
                            v = v + 1
                        avanzar_recto_grados(90,700,0)
                time.sleep(0.001)
                                
                running = False
    
            except KeyboardInterrupt:
                print("\nProgram interrupted! Cleaning up...")
                spike.write(chr(3).encode())
                spike.readline() #clear buffer
                spike.readline() #clear buffer
                spike.readline() #clear buffer
                Coast_motors()
                spike.close()
                print("\nStopping...")
                running = False
                cleanup_gpio(h)
                lidar.stop()
                break
            
    except Exception as e:
        print(f"Error: {e}")
        cleanup_gpio(h)
    finally:
        running = False
        lidar.stop()
        lidar.disconnect()
        cleanup_gpio(h)

if __name__ == "__main__":
    # Set up signal handler for clean exit
    
    main()
    print("Exiting...")
