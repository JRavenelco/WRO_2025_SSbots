"""
This module provides support for serial communication in Python.
"""
import serial
import time
"""Added simplified script, originated from "interpreter_startup.py", starting here:"""
import subprocess
#import time (Already imported above)
import os
import pty

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
    spike.write("motor.stop(port.A, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.E, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer
    spike.write("distancias = [distance_sensor.distance(port.C), distance_sensor.distance(port.B), distance_sensor.distance(port.F)]\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def fc():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.A, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.E, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    # centrar el vehiculo 
    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.run_to_absolute_position(port.E, 0, 550,\r".encode())
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
    spike.write("motor.run_to_absolute_position(port.E, int(et), 550, direction = motor.SHORTEST_PATH)\r".encode())
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
    spike.write("def ad(vel,distancia,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("while 0 > distance_sensor.distance(port.C) or distance_sensor.distance(port.C) > distancia:\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(referencia,motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def vuelta(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_absolute_position(port.E, 49*(direccion), 550, direction = motor.SHORTEST_PATH)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.A, (100)*velocidad)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def da(vel):\r".encode())
    spike.readline() #clear buffer
    spike.write("ulz = distance_sensor.distance(port.B)\r".encode())
    spike.readline() #clear buffer
    spike.write("uld = distance_sensor.distance(port.F)\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("while ulz > 0 and uld > 0:\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(0,motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    spike.write("ulz = distance_sensor.distance(port.B)\r".encode())
    spike.readline() #clear buffer
    spike.write("uld = distance_sensor.distance(port.F)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def va(vel, grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("ulz = distance_sensor.distance(port.B)\r".encode())
    spike.readline() #clear buffer
    spike.write("uld = distance_sensor.distance(port.F)\r".encode())
    spike.readline() #clear buffer
    spike.write("der = -1\r".encode())
    spike.readline() #clear buffer
    spike.write("izq = 1\r".encode())
    spike.readline() #clear buffer
    spike.write("if ulz == -1:\r".encode())
    spike.readline() #clear buffer
    spike.write("vuelta(izq,vel,grados)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("if uld == -1:\r".encode())
    spike.readline() #clear buffer
    spike.write("vuelta(der,vel,grados)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

def center_vehicle():
    spike.write("cv()\r".encode())
    spike.readline() #clear buffer

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
    spike.write(("ad("+str(vel)+","+str(distancia)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
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
    Coast_motors()

def avanzar_detection(vel):
    spike.write(("da("+str(vel)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la deteccion")
    Coast_motors()

def vuelta_automatica(vel,grados):
    spike.write(("va("+str(vel)+","+str(grados)+")\r").encode())
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

try:# Main Program
    initialize_Libraries()
    i = 0
    while i < 8:
        i = i + 1
    #    reset_gyro()
    #    avanzar_distancia(90,780)
    #    vuelta_grados(der,60,90)
    #   Free_spikeDirection()
    #   reset_gyro()
    #time.sleep(0.050)
        reset_gyro(0)
        avanzar_detection(70)
        vuelta_automatica(60,91)
        reset_gyro(0)
        avanzar_distancia(70,780,-90)

    #Coast_motors() #brake all motors
    #spike.close() #close serial connection

except KeyboardInterrupt:
    print("\nProgram interrupted! Cleaning up...")
    spike.write(chr(3).encode())
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    Coast_motors()




