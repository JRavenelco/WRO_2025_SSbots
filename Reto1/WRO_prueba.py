#!/usr/bin/env python3
#se importan todas las librerias necesarias

import time, queue, serial, subprocess, os, pty, threading, lgpio, board, busio, adafruit_vl53l1x
from dataclasses import dataclass
from math import floor
from rplidar import RPLidar, RPLidarException
import numpy as np

#se define la clase para el hilo del lidar
@dataclass
class RPlidar_data:          
    dist_left: int  
    dist_75: int 
    dist_315: int
    dist_right: int 
    area: float  

@dataclass
class Distance_data:
    distance_izq: int
    distance_derecho: int
    distance_trasero: int

#se define la clase para el hilo del sensor tof
@dataclass
class Sensor_tof_data:
    identifier : adafruit_vl53l1x.VL53L1X #adafruit_vl53l0x.VL53L0X
    name : str
    slave_address : int
    pin_gpio_enable :int   

#se guardan los sensores en una lista
SENSORS = [
    Sensor_tof_data(None,"izquierda", slave_address=0x26, pin_gpio_enable =17),
    Sensor_tof_data(None,"derecho", slave_address=0x28, pin_gpio_enable =27),
    Sensor_tof_data(None,"trasero", slave_address=0x29, pin_gpio_enable =22)
]


running = True
vision_queue: "queue.Queue[RPlidar_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del lidar
distance_queue: "queue.Queue[Distance_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del sensor tof

#se crea el lidar y se define su puerto
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME)

#se crea el sensor tof y se define su puerto
i2c = board.I2C() #i2c = busio.I2C(board.SCL, board.SDA)
 

#se define la funcion que inicia el screen
def screenSimplified():
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

    
#se define la variable para el spike
spike = None
#variables globales rasp
der = -1
izq = 1
relay = 10
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

    # centrar el vehiculo corto
    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.run_to_absolute_position(port.F, 0, 1000,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 10000, deceleration = 1000)\r".encode())
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
    spike.write("motor.run_to_absolute_position(port.F, int(et), 1100, direction = motor.SHORTEST_PATH, stop = motor.COAST, acceleration = 10000)\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (-100)*(vel))\r".encode())
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

    #avanzar derecho
    spike.write("def ad(vel,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.5,1,error)\r".encode())
    spike.readline() #clear buffer 
    end_Function()

    spike.write("def vuelta(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_absolute_position(port.F, 50*(direccion), 1100, stop = motor.HOLD, acceleration = 10000)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (-100)*(velocidad))\r".encode())
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
    spike.write("error = pd(referencia,motion_sensor.tilt_angles()[0],vel,0.5,10,error)\r".encode())
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
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.5,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()
    """
    spike.write("def spd(vel,distancia,sensor):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(distancia,sensor,vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.B)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    
    spike.write("def spi(vel,distancia,sensor):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(sensor,distancia,vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.B)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    """
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
    time.sleep(1)
    Coast_motors()


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
    #print("Fin de la vuelta")
    Coast_motors()

def avanzar_detection(vel,referencia):
    #avanzar hasta que detecte un vacio, son ambos sensores para la primera vuelta
    pass

def avanzar_detection_izquierdo(vel,referencia):

    try:
        while not distance_queue.full():
            a = 1
        obj_distance = distance_queue.get()
        while obj_distance.distance_izq > 0 and obj_distance.distance_izq < 1350:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if distance_queue.full():
                obj_distance = distance_queue.get()
            print(obj_distance.distance_izq)
        Coast_motors()
    except queue.Empty:
        pass

def avanzar_detection_izquierdo_lidar(vel,referencia):
    try:
        while not vision_queue.full():
            print("ESPERANDO EL QUEVE PARA DETECTAR EL VACIO IZQUIERDO\n")
        obj = vision_queue.get()
        while obj.dist_left < 1350:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                #print("obtuvo el queve")
                obj = vision_queue.get()
            print(obj.dist_left)
        Coast_motors()
        
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()
    
def avanzar_detection_derecho(vel,referencia):

    #avanzar hasta que detecte un vacio con el sensor derecho
    pass

def vuelta_automatica(velocidad, grados, uls):
    #da una vuelta automatica si hay un vacio, hacia cualquier lado
    pass

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
    print("Fin de la vuelta")
    Coast_motors()

def seguir_pared_derecho(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor derecho
    pass

def seguir_pared_izquierdo(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor izquierdo
    pass
    
#funcion para el lidar que obtiene las medidas en angulos especificos
def RPlidar_worker():
    #lidar.start_motor()
    #lidar.start() # star scaning measurements
    try:
        lidar.clean_input()
        angles = [0]*360 # array de 360 lugares para guardar las medidas del lidar
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                if distance is not None:
                    angles[min([359,floor(angle)])] = distance
            print(angles[0],angles[90],angles[270])
            lidar_data = RPlidar_data(angles[90],angles[75],angles[315],angles[270],time.time())
            if vision_queue.full(): #check if the queue is full
                try:
                    print("queue full")
                    vision_queue.get() # erase old values from the queue to have latest information 
                except vision_queue.not_empty:
                    pass
            vision_queue.put(lidar_data) #load new data on the queue
            vision_queue.task_done()
    except RPLidarException as e:
        print("error in lidar:", e)
        lidar.clean_input()
    except Exception as e:
        print("error in lidar-aritmetica:", e)
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
def correcion(correcion_del_angulo):
    if vision_queue.full():
        try:
            obj = vision_queue.get_nowait()
            H = int(obj.dist_75*10)
            CA = int(obj.dist_left*10)
            lado_faltante = np.sqrt((H**2) + (CA**2)- (2*H*CA*np.cos(grados_a_radianes(correcion_del_angulo))))
            ley_de_senos = H*np.sin(grados_a_radianes(correcion_del_angulo))
            angulo_correcion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
            if radianes_a_grados(np.arcsin(CA/H)) < 75:
                angulo_correcion = 180-angulo_correcion
            else:
                angulo_correcion = angulo_correcion
            angulo_correcion = angulo_correcion-90
            print(angulo_correcion)
            return angulo_correcion
        except queue.Empty:
            pass
        except KeyboardInterrupt:
            Coast_motors()

#funcion que habilita el acceso a los gpio
def setup_gpio():
    global relay
    try:
        h = lgpio.gpiochip_open(0) #se habilita el acceso a los gpio
        lgpio.gpio_claim_output(h, relay)
        lgpio.gpio_write(h, relay, 1)
        #for sensor in SENSORS:
         #   lgpio.gpio_claim_output(h, sensor.pin_gpio_enable)
          #  lgpio.gpio_write(h, sensor.pin_gpio_enable, 0)
           # time.sleep(0.5)
        print("Sensores inicializados\n")
        #for sensor in SENSORS:
         #   lgpio.gpio_write(h, sensor.pin_gpio_enable, 1)
          #  time.sleep(0.5)
          #  sensor.identifier = adafruit_vl53l1x.VL53L1X(i2c)
          #  print(sensor.identifier.model_info)
          #  sensor.identifier.distance_mode = 1
          #  sensor.identifier.timing_budget = 20
          #  sensor.identifier.roi_center = 199
          #  print(sensor.identifier.roi_center) 
          #  sensor.identifier.roi_xy = (16,8)
          #  print(sensor.identifier.roi_xy) 
          #  sensor.identifier.set_address(sensor.slave_address)
          #  print(f"Sensor {sensor.name} inicializado en la direccion {hex(sensor.slave_address)}\n")
        lgpio.gpio_write(h, relay, 0)
        lgpio.gpio_claim_input(h, 4)
        time.sleep(1)
        print("GPIO inicializados")
        return h, True

    except Exception as e:
        print(f"Error al inicializar sensores: {e}")
        return h, False

#funcion que limpia los gpio
def cleanup_gpio(h):
    global relay
    try:
        for sensor in SENSORS:
            try:
                lgpio.gpio_free(h, sensor.pin_gpio_enable)
            except:
                pass
        lgpio.gpio_free(h, 4)
        lgpio.gpio_free(h, relay)
        lgpio.gpiochip_close(h)
        print("GPIO libres")
    except:
        pass

def sensor_worker():
    for sensor in SENSORS:
        sensor.identifier.start_ranging()
    obj = Distance_data(0,0,0)
    distancias = [0,0,0]
    while 1:
        try:
            for i,sensor in enumerate(SENSORS):
                if sensor.identifier.data_ready:
                    distancias[i] = sensor.identifier.distance 
                    sensor.identifier.clear_interrupt()
            if distancias[0] != 0 or distancias[0] == None:
                obj.distance_izq = distancias[0]
            if distancias[1] != 0 or distancias[1] == None:
                obj.distance_derecho = distancias[1]
            if distancias[2] != 0 or distancias[2] == None:
                obj.distance_trasero = distancias[2]

            if distance_queue.full(): 
                try:
                    distance_queue.get() # erase old values from the queue to have latest information 
                except distance_queue.not_empty:
                    pass
            if obj.distance_izq != 0 or obj.distance_derecho != 0 or obj.distance_trasero != 0 or obj.distance_izq == None or obj.distance_derecho == None or obj.distance_trasero == None:
                distance_queue.put(obj)
                distance_queue.task_done() 
            time.sleep(0.01)
        except Exception as e:
            Coast_motors()
            print(f"Error al leer sensor: {e}")

def main():
    global running, spike
    h, success = setup_gpio()
    try:
        spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        screenSimplified()
        initialize_Libraries()
        print("presiona el boton\n")
        #while(lgpio.gpio_read(h, 4) == 1):
        #    a = 0
        print("empezando ....\n")
        lidar.stop()
        lidar.stop_motor() #detenemos el motor del lidar 
        lidar.disconnect() #desconectamos el lidar
        time.sleep(0.5)
        lidar.connect()
        #threading.Thread(target=sensor_worker).start()
        threading.Thread(target=RPlidar_worker).start()
        reset_gyro(0)
        time.sleep(2) #espera para que el giroscopio se reinicie
        obj_distance = Distance_data(None,None,None)
        while running:
            try:
                avanzar_detection_izquierdo_lidar(70,0)
                """if distance_queue.full():
                    obj_distance = distance_queue.get_nowait()
                    print(obj_distance.distance_izq, obj_distance.distance_derecho, obj_distance.distance_trasero)
                """
                """
                while not vision_queue.full():
                    print("no ando listo")
                    time.sleep(0.001)
                avanzar_detection_izquierdo_lidar(70,0)
                Coast_motors()
                """
                """
                centrar_vehiculo()
                time.sleep(1)
                avanzar_detection_izquierdo_lidar(70,0)
                time.sleep(1)
                vuelta_grados(izq,60,90)
                time.sleep(1)
                centrar_vehiculo()
                time.sleep(1)
                reset_gyro(0)
                avanzar_recto_grados(70,1000,0)
                time.sleep(5)
                angulo = correcion(15)
                print(angulo)
                reset_gyro(int(angulo*10))
                """
                #distancias = distance_queue.get_nowait()
                        
                #angulo_correcion = correcion(15)

                """if vision_queue.full():
                    try:
                        obj = vision_queue.get_nowait()
                        print(obj.dist_left, obj.dist_75, obj.dist_315, obj.dist_right)
                    except queue.Empty:
                        pass"""
                
                running = True
    
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
                lidar.disconnect()
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
