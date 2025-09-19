#!/usr/bin/env python3
#se importan todas las librerias necesarias

import time, queue, serial, subprocess, os, pty, threading, lgpio, board, busio, adafruit_vl53l1x
from dataclasses import dataclass
from math import floor
from rplidar import RPLidar, RPLidarException
import numpy as np

#se define la variable para el spike
spike = None
#variables globales rasp
der = -1
izq = 1
relay = 10
led = 9
button = 11
leftOrRight = 0

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
    Sensor_tof_data(None,"izquierda", slave_address=0x26, pin_gpio_enable =27),
    Sensor_tof_data(None,"derecho", slave_address=0x28, pin_gpio_enable =17),
    Sensor_tof_data(None,"trasero", slave_address=0x29, pin_gpio_enable =22)
]


running = True
vision_queue: "queue.Queue[RPlidar_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del lidar
distance_queue: "queue.Queue[Distance_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del sensor tof

#se crea el lidar y se define su puerto
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME,baudrate=1000000, timeout=1)

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
    spike.write("def hd():\r".encode())#HOLD MOTORS
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
    spike.write("motor.run_to_absolute_position(port.F, 140*(direccion), 1100, stop = motor.HOLD, acceleration = 10000)\r".encode())
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


def Hold_motors():
    spike.write("hd()\r".encode())
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
    #avanzar cierta distancia sensor del frente, creo que no se ocupo
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
    Hold_motors()

def avanzar_detection(vel,referencia):
    global leftOrRight
    try:
        while not vision_queue.full():
            print("ESPERANDO VACIO\n")
        obj = vision_queue.get()
        while obj.dist_left < 1350 and obj.dist_right < 1350:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                #print("obtuvo el queve")
                obj = vision_queue.get()
            #print(obj.dist_left)
            #print(obj.dist_right)
        if obj.dist_left >= 1350:
            leftOrRight = izq
            print("izquierda: ", obj.dist_left)
        elif obj.dist_right >= 1350:
            leftOrRight = der
            print("derecha: ", obj.dist_right)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()
        
def avanzar_detection_2(vel,referencia):
    try:
        while not vision_queue.full():
            print("ESPERANDO VACIO\n")
        obj = vision_queue.get()
        while obj.dist_left < 1350 and obj.dist_right < 1350:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                print("obtuvo el queve")
                obj = vision_queue.get()
            #print(obj.dist_left)
            #print(obj.dist_right)
        if obj.dist_left >= 1350:
            direction = izq
            print("izquierda: ", obj.dist_left)
        elif obj.dist_right >= 1350:
            direction = der
            print("derecha: ", obj.dist_right)
        Coast_motors()
        return direction
    except queue.Empty:
        pass
    except KeyboardInterrupt:
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
    print("Fin de la vuelta")
    Coast_motors()
    
def dos_puntos(velocidad, grados, referencia):
    global leftOrRight
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
    obj = vision_queue.get()
    y1 = obj.dist_left
    spike.write(("ag("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    Coast_motors()
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
    obj = vision_queue.get()
    if leftOrRight == izq:
        print("izquierda")
        y2 = obj.dist_left
    elif leftOrRight == der:
        print("derecha")
        y2 = obj.dist_right
    else:
        print("no hay leftOrRight")
    variacion = y2 - y1
    pendiente = int(radianes_a_grados(np.arctan(variacion / abs(175*grados/360)))*-10)
    print(pendiente)
    return pendiente

#funcion para el lidar que obtiene las medidas en angulos especificos
def RPlidar_worker():
    #lidar.start_motor()
    #lidar.start() # star scaning measurements
    time.sleep(2)  # Give motor time to spin up
    while not vision_queue.full():
        #print("ESPERANDO EL QUEVE PARA EL AVANZAR\n")
        pass
    #time.sleep(5)
    v = 0
    while v < 12:
        centrar_vehiculo()
        avanzar_detection(70,0)
        print("termine")
        print(leftOrRight)
        vuelta_grados(leftOrRight,60,88)
        print("avanzar recto")
        centrar_vehiculo()
        reset_gyro(0)
        avanzar_recto_grados(70,1000,0)
        time.sleep(2)
        #angulo = correccion(15)
        angulo = dos_puntos(70,1000,0)
        time.sleep(2)
        print(angulo)
        reset_gyro(angulo)
        time.sleep(0.001)
        v = v + 1
    

#convercion de grados a radianes
def grados_a_radianes(grados):
    grados_convertidos : int = (grados)/(180/np.pi)
    return grados_convertidos

#convercion de radianes a grados
def radianes_a_grados(radianes):
    radianes_convertidos : int = (radianes)*(180/np.pi)
    return radianes_convertidos

#funcion que calcula la correccion del giroscopio utilizando medidas del lidar y funciones trigonométricas
def correccion(correccion_del_angulo):
    if vision_queue.full():
        try:
            obj = vision_queue.get_nowait()
            H = int(obj.dist_75*10)
            CA = int(obj.dist_left*10)
            lado_faltante = np.sqrt((H**2) + (CA**2)- (2*H*CA*np.cos(grados_a_radianes(correccion_del_angulo))))
            ley_de_senos = H*np.sin(grados_a_radianes(correccion_del_angulo))
            angulo_correccion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
            if radianes_a_grados(np.arcsin(CA/H)) < 75:
                angulo_correccion = 180-angulo_correccion
            else:
                angulo_correccion = angulo_correccion
            angulo_correccion = int((angulo_correccion-90)*-10)
            print(angulo_correccion)
            return angulo_correccion
        except queue.Empty:
            pass
        except KeyboardInterrupt:
            Coast_motors()

#funcion que habilita el acceso a los gpio
def setup_gpio():
    global relay, led, button
    try:
        h = lgpio.gpiochip_open(0) #se habilita el acceso a los gpio
        lgpio.gpio_claim_input(h, button)
        lgpio.gpio_claim_output(h, relay)
        lgpio.gpio_claim_output(h, led)
        lgpio.gpio_write(h, relay, 1)
        time.sleep(1)
        lgpio.gpio_write(h, led, 1)
        """
        for sensor in SENSORS:
            lgpio.gpio_claim_output(h, sensor.pin_gpio_enable)
            lgpio.gpio_write(h, sensor.pin_gpio_enable, 0)
            time.sleep(0.5)
        for sensor in SENSORS:
            lgpio.gpio_write(h, sensor.pin_gpio_enable, 1)
            time.sleep(0.5)
            sensor.identifier = adafruit_vl53l1x.VL53L1X(i2c)
            print(sensor.identifier.model_info)
            sensor.identifier.distance_mode = 1
            sensor.identifier.timing_budget = 33
            sensor.identifier.roi_center = 199
            print(sensor.identifier.roi_center) 
            sensor.identifier.roi_xy = (16,8)
            print(sensor.identifier.roi_xy) 
            sensor.identifier.set_address(sensor.slave_address)
        """
        lgpio.gpio_write(h, relay, 0)
        print("GPIO inicializados")
        return h, True

    except Exception as e:
        print(f"Error al inicializar sensores: {e}")
        return h, False

#funcion que limpia los gpio
def cleanup_gpio(h):
    try:
        for sensor in SENSORS:
            try:
                lgpio.gpio_free(h, sensor.pin_gpio_enable)
            except:
                pass
        lgpio.gpio_write(h, relay, 1)
        lgpio.gpio_free(h, button)
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
        #threading.Thread(target=sensor_worker).start()
        
        reset_gyro(0)
        obj_distance = Distance_data(None,None,None)
        while running:
            try:
                
                """if distance_queue.full():
                    obj_distance = distance_queuecc.get_nowait()
                    print(obj_distance.distance_izq, obj_distance.distance_derecho, obj_distance.distance_trasero)
                """
                """
                while not vision_queue.full():
                    print("no ando listo")
                    time.sleep(0.001)
                avanzar_detection_izquierdo_lidar(70,0)
                Coast_motors()
                """
                info = lidar.get_info()
                for key, value in info.items():
                    print('{0:<13}: {1}'.format(key.capitalize(), str(value)))

                health = lidar.get_health()
                print('Health Status: {0[0]} - {0[1]}'.format(health))

                
                print('*' * 50)
                lidar.start_motor()
                time.sleep(3)  # Give motor time to spin up
                #lidar.start() # star scaning measurements
                
                try:
                    angles = [0]*360 # array de 360 lugares para guardar las medidas del lidar
                    iterator = lidar.iter_scans(max_buf_meas=32000,min_len=10)
                    threading.Thread(target=RPlidar_worker,daemon=True).start()
                    
                    for scan in iterator:
                        for (_, angle, distance) in scan:
                            if distance is not None:
                                angles[min([359,floor(angle)])] = distance
                        print(angles[0],angles[90],angles[270])
                        time.sleep(0.001)
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
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                    Coast_motors()
                except Exception as e:
                    print("error in lidar-aritmetica:", e)
                    lidar.clean_input()
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                    Coast_motors()
                except KeyboardInterrupt:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                    Coast_motors()
                    print("Stopping.")
                
                #distancias = distance_queue.get_nowait()
                        
                #angulo_correccion = correccion(15)

                """if vision_queue.full():
                    try:
                        obj = vision_queue.get_nowait()
                        print(obj.dist_left, obj.dist_75, obj.dist_315, obj.dist_right)
                    except queue.Empty:
                        pass"""
                
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
                lidar.disconnect()
                spike.close()
                running = False
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
