#!/usr/bin/env python3

import time
import lgpio
import threading
import queue
import signal
import sys
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple
"""
This module provides support for serial communication in Python.
"""
import serial
"""Added simplified script, originated from "interpreter_startup.py", starting here:"""
import subprocess
import os
import pty
# Configuration for each sensor
@dataclass
class UltrasonicSensor:
    name: str
    trigger_pin: int
    echo_pin: int
    last_reading: Optional[float] = None
    last_update: float = 0.0  # Timestamp of last successful reading

# Sensor configurations (trigger_pin, echo_pin)
SENSORS = [
    UltrasonicSensor("izquierda", trigger_pin=13, echo_pin=13),
    UltrasonicSensor("frente", trigger_pin=19, echo_pin=19),
    UltrasonicSensor("derecha", trigger_pin=6, echo_pin=6)
]

# Performance tuning parameters
MIN_MEASURE_INTERVAL = 0.04  # 30ms between measurements (about 33Hz per sensor)
SENSOR_TIMEOUT = 0.06      # 50ms timeout for echo pulse (reduced from 100ms)
SETTLE_TIME = 0.002         # 2ms settling time between trigger and echo
DISPLAY_UPDATE_RATE = 10    # Hz for display updates

# Speed of sound in cm/us
SPEED_OF_SOUND = 34300 / 1000000  # cm/s to cm/us

# Global flag for thread control
running = True

# Thread-safe queue for sensor updates
sensor_queue = queue.Queue()

def screenSimplified(): #Called main() in the original script, renamed to avoid conflict with the main function in this module.
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

if __name__ == "__main__":
    screenSimplified()
    
"""End of simplified script, originated from "interpreter_startup.py" """

spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#variables globales rasp
der = -1
izq = 1

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
    spike.write("def fr():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.E, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.A, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def fc():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.E, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.A, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    # centrar el vehiculo 
    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.run_to_absolute_position(port.E, 0, 550,\r".encode())
    spike.readline()#clear buffemotor.stop(port.E, stop = motor.COAST)r
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
    spike.write("motor.run_to_absolute_position(port.E, int(et), 300, direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 10000)\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.A, (100)*(vel))\r".encode())
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
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer 
    end_Function()

    #avanzar derecho indefinido
    spike.write("def ai(vel,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("while 1:\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer 
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    end_Function()

    spike.write("def vuelta(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_absolute_position(port.E, 49*(direccion), 550, direction = motor.SHORTEST_PATH)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.A, (100)*(velocidad))\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fr()\r".encode())   
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def da(vel, referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(referencia,motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def ag(vel,grados,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.reset_relative_position(port.A,0)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados) > abs(motor.relative_position(port.A)):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
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
    spike.write("error = pd(distancia,sensor,vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.A)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def spi(vel,distancia,sensor):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(sensor,distancia,vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("return motor.relative_position(port.A)\r".encode())
    spike.readline() #clear buffer
    end_Function()


def centrar_vehiculo():
    spike.write("cv()\r".encode())
    spike.readline() #clear buffer    return_value = spike.readline().decode()
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la vuelta")
    Free_spikeDirection()
    time.sleep(0.01)

def Free_spikeDirection():
    spike.write("fr()\r".encode())
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
    uls = [2000,2000,2000]
    i = 0
    while ((distancia*10) < uls[1]) or (uls[1] == -1):
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1
        spike.write(("ad("+str(vel)+","+str(referencia)+")\r").encode())
        spike.readline() #clear buffer
        print(uls[1])
    print("Fin del recorrido")

    Coast_motors()

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
    Free_spikeDirection()
    time.sleep(0.01)

def avanzar_detection(vel,referencia):
    uls = [1,1,1]
    i = 0
    while uls[0] > 0 and uls[2] > 0 and uls[0] < 2000 and uls[2] < 2000:
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1
        spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
        spike.readline() #clear buffer
    return uls
    #Coast_motors()

def avanzar_detection_gyro(vel,referencia):
    uls = [1,1,1]
    i = 0
    spike.write(("ai("+str(vel)+","+str(referencia)+")\r").encode()) #suprimir linea
    spike.readline() #clear buffer
    while uls[0] > 0 and uls[2] > 0 and uls[0] < 2000 and uls[2] < 2000:
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1
    spike.write(chr(3).encode())
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    Coast_motors()

def vuelta_automatica(velocidad, grados, uls):
    while 1:
        if uls[0] == -1 or uls[2] == -1:
            break
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1    
    print("vuelta = ",uls,"\n")
    if uls[0] == -1:
        vuelta_grados(izq,velocidad,grados)
    elif uls[2] == -1:
        vuelta_grados(der,velocidad,grados)
    Free_spikeDirection()

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
    uls = [1,1,1]
    i = 0
    for sensor in SENSORS:
        if sensor.last_reading is not None:
            uls[i] = int(sensor.last_reading)
        i = i + 1
    ditancia_Inicial = uls[2]
    spike.write("motor.reset_relative_position(port.A)\r".encode())
    spike.readline() #clear buffer
    gradosMotor = 0
    while abs(gradosMotor) < grados:
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1
        spike.write(("spd("+str(velocidad)+","+str(ditancia_Inicial)+","+str(uls[2])+")\r").encode())
        spike.readline() #clear buffer
        gradosMotor = int(spike.readline().decode())
    Coast_motors()


def seguir_pared_izquierdo(velocidad,grados):
    uls = [1,1,1]
    i = 0
    for sensor in SENSORS:
        if sensor.last_reading is not None:
            uls[i] = int(sensor.last_reading)
        i = i + 1
    ditancia_Inicial = uls[0]
    spike.write("motor.reset_relative_position(port.A)\r".encode())
    spike.readline() #clear buffer
    gradosMotor = 0
    while abs(gradosMotor) < grados:
        i = 0
        for sensor in SENSORS:
            if sensor.last_reading is not None:
                uls[i] = int(sensor.last_reading)
            i = i + 1
        spike.write(("spi("+str(velocidad)+","+str(ditancia_Inicial)+","+str(uls[0])+")\r").encode())
        spike.readline() #clear buffer
        gradosMotor = int(spike.readline().decode())
    Coast_motors()

def get_distance(h: int, sensor: UltrasonicSensor) -> Optional[float]:
    """Measures distance using an ultrasonic sensor with minimal latency."""
    try:
        # Send trigger pulse (minimal delay)
        lgpio.gpio_claim_output(h, sensor.trigger_pin)
        lgpio.gpio_write(h, sensor.trigger_pin, 0)
        time.sleep(0.00001)  # 1us (minimum)
        lgpio.gpio_write(h, sensor.trigger_pin, 1)
        time.sleep(0.000010)  # 10us (required by HC-SR04)
        lgpio.gpio_write(h, sensor.trigger_pin, 0)
        
        # Switch to input mode for echo (no delay)
        lgpio.gpio_claim_input(h, sensor.echo_pin)

        # Wait for the pulse to start (with timeout)
        timeout = time.time() + SENSOR_TIMEOUT
        while lgpio.gpio_read(h, sensor.echo_pin) == 0:
            if time.time() > timeout:
                return None

        pulse_start = time.time()
        
        # Wait for the pulse to end (with timeout)
        timeout = time.time() + SENSOR_TIMEOUT
        while lgpio.gpio_read(h, sensor.echo_pin) == 1:
            if time.time() > timeout:
                return None
        
        pulse_end = time.time()
        
        # Calculate distance in cm
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * SPEED_OF_SOUND * 10000000) / 2

        # Filter out invalid readings
        if 20 <= distance <= 2000:  # 4m max reliable range
            return distance
        else:
            return -1

    except Exception as e:
        print(f"Error reading {sensor.name} sensor: {e}")
        return None

def sensor_worker(h: int, sensor: UltrasonicSensor):
    """Worker function to continuously read a sensor with minimal latency"""
    last_measure_time = 0
    
    while running:
        current_time = time.time()
        
        # Only measure if enough time has passed since last measurement
        if current_time - last_measure_time >= MIN_MEASURE_INTERVAL:
            distance = get_distance(h, sensor)
            if distance is not None:
                sensor.last_reading = distance
                sensor.last_update = current_time
                sensor_queue.put((sensor.name, distance))
            last_measure_time = current_time
        else:
            # Small sleep to prevent 100% CPU usage
            time.sleep(0.001)  # 1ms

def setup_gpio():
    """Initialize GPIO and claim all necessary pins"""
    try:
        h = lgpio.gpiochip_open(0)
        
        # Setup all sensor pins
        for sensor in SENSORS:
            lgpio.gpio_claim_output(h, sensor.trigger_pin)
            lgpio.gpio_claim_input(h, sensor.echo_pin)
            lgpio.gpio_write(h, sensor.trigger_pin, 0)  # Ensure trigger is low
        
        # Let sensors settle
        print("Initializing sensors...")
        time.sleep(1)
        return h, True
        
    except Exception as e:
        print(f"GPIO initialization failed: {e}")
        return None, False

def cleanup_gpio(h):
    """Release all GPIO resources"""
    try:
        for sensor in SENSORS:
            try:
                lgpio.gpio_free(h, sensor.trigger_pin)
                lgpio.gpio_free(h, sensor.echo_pin)
            except:
                pass
        lgpio.gpiochip_close(h)
        print("GPIO resources cleaned up")
    except:
        pass

def main():
    global running
    
    print("Multi-Sensor Ultrasonic Distance Measurement")
    print("Sensors:")
    for sensor in SENSORS:
        print(f"  {sensor.name}: GPIO{sensor.trigger_pin}")
    print("Press Ctrl+C to exit\n")
    
    # Initialize GPIO
    h, success = setup_gpio()
    if not success:
        return
    
    try:
        # Start sensor worker threads with high priority
        threads = []
        for sensor in SENSORS:
            t = threading.Thread(
                target=sensor_worker,
                args=(h, sensor),  # No fixed interval, uses timing control
                daemon=True
            )
            t.start()
            threads.append(t)
        initialize_Libraries()
        reset_gyro(0)
        # Main display loop
        last_display = 0
        while running:
            try:
                uls = [1,1,1]
                uls = avanzar_detection(80,0)
                print("avanzar hasta detectar \n",uls)
                i = 0
                for sensor in SENSORS:
                    if sensor.last_reading is not None:
                        uls[i] = int(sensor.last_reading)
                    i = i + 1
                if uls[0] == -1 :
                    v = 0
                    while v < 12:
                        v = v + 1
                        uls = [1,1,1]
                        uls_ant = [1,1,1]      
                        uls = avanzar_detection(80,0)
                        print("avanzar hasta detectar \n",uls)       
                        vuelta_automatica(65,90,uls)
                        centrar_vehiculo()
                        avanzar_recto_grados(65,400,90)
                        time.sleep(0.001)
                        i = 0
                        for sensor in SENSORS:
                            if sensor.last_reading is not None:
                                uls_ant[i] = int(sensor.last_reading)
                            i = i + 1
                        print("avanzar recto \n",uls_ant)
                        avanzar_recto_grados(50,720,90)
                        i = 0
                        for sensor in SENSORS:
                            if sensor.last_reading is not None:
                                uls[i] = int(sensor.last_reading)
                            i = i + 1
                        print("avanzar recto \n",uls)
                        time.sleep(0.001)
                        angulo = uls_ant[0] - uls[0]
                        reset_gyro(int(((angulo)/((abs(720))* (4)))*10000)) #constate de porpocion a milimetros
                        print("Inclinacion = ",((angulo)/((abs(720))* (4)))*10000,int(((angulo)/((abs(720))* (4)))*10000))
                        Coast_motors()
                    running = False
                    print("movimiento acabado\n")

                if uls[2] == -1:
                    v = 0
                    while v < 12:
                        v = v + 1
                        uls = [1,1,1]
                        uls_ant = [1,1,1]      
                        uls = avanzar_detection(80,0)
                        print("avanzar hasta detectar \n",uls)       
                        vuelta_automatica(65,90,uls)
                        centrar_vehiculo()
                        avanzar_recto_grados(65,500,-90)
                        time.sleep(0.001)
                        i = 0
                        for sensor in SENSORS:
                            if sensor.last_reading is not None:
                                uls_ant[i] = int(sensor.last_reading)
                            i = i + 1
                        print("avanzar recto \n",uls_ant)
                        avanzar_recto_grados(50,720,-90)
                        i = 0
                        for sensor in SENSORS:
                            if sensor.last_reading is not None:
                                uls[i] = int(sensor.last_reading)
                            i = i + 1
                        print("avanzar recto \n",uls)
                        time.sleep(0.001)
                        angulo = uls_ant[2] - uls[2]
                        reset_gyro(int(((angulo)/((abs(720))* (2.8)))*-1000)) #constate de porpocion a milimetros
                        print("Inclinacion = ",((angulo)/((abs(720))* (2.8)))*-1000,int(((angulo)/((abs(720))* (2.8)))*-1000))
                        Coast_motors()
                    running = False
                    print("movimiento acabado\n")






                
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
                break
                
        # Wait for threads to finish
        for t in threads:
            t.join(timeout=1.0)
            
    except Exception as e:
        print(f"Error: {e}")
        cleanup_gpio(h)
    finally:
        running = False
        cleanup_gpio(h)

if __name__ == "__main__":
    # Set up signal handler for clean exit
    
    main()
    print("Exiting...")
