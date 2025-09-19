#!/usr/bin/env python3

import time, queue, serial, subprocess, os, pty, threading, lgpio, sys, cv2, argparse, json, blobconverter
from dataclasses import dataclass
from math import floor
from rplidar import RPLidar, RPLidarException
import numpy as np
from pathlib import Path
import depthai as dai

spike = None

right = -1
left = 1
relay = 10
led = 9
button = 11
redbox = 1
greenbox = 0 

@dataclass
class RPlidar_data:          
    dist_left: int  
    dist_75: int 
    dist_285: int
    dist_right: int
    dist_front: int
    area: float
    
running = True  # Initial delay to allow system to stabilize
vision_queue: "queue.Queue[RPlidar_data]" = queue.Queue(maxsize=1)
vision_camera_queue: "queue.Queue[PillarObservation]" = queue.Queue(maxsize=1)
# Configuracio para la vision de la camara
@dataclass
class PillarObservation:
    label: int           # "regresa el objeto rojo = 1, verde = 0, nada = -1"
    xmid: int   # En centímetros
    ymid: int # -1.0 izquierda, 0 centro, +1.0 derecha
    area: float          # Marca de tiempo

PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME,baudrate=1000000, timeout=2)
    
def spikeStartup(): #Start serial communication with spike
    global spike
    
    try:
        spike = serial.Serial("/dev/ttyACM0", 115200, timeout=2)
        time.sleep(1.0)
        print("Serial connection to SPIKE established.")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        spike = None
        
    time.sleep(0.1)
    spike.write(b'\x03') #syste Ctrl+C
    time.sleep(0.1)


    
def endFunction(): #Sends empty lines to spike to end a function
    spike.write("\r".encode())
    spike.readline() 
    spike.write("\r".encode())
    spike.readline()
    spike.write("\r".encode())
    spike.readline()

def spikeLibraries(): #Initializes libraries and functions in spike
    #import libraries
    spike.write("import motor\r".encode()) 
    spike.readline()
    spike.write("from hub import port\r".encode())
    spike.readline()
    spike.write("from hub import motion_sensor\r".encode())
    spike.readline()
    spike.write("import distance_sensor\r".encode())
    spike.readline()
    spike.write("import runloop\r".encode())
    spike.readline()
    #declare global variables for spike
    spike.write("right = -1\r".encode())
    spike.readline() 
    spike.write("left = 1\r".encode())
    spike.readline() 
    spike.write("error = 0\r".encode())
    spike.readline() 
    #declare functions for motors
    spike.write("def hd():\r".encode())#HOLD MOTORS
    spike.readline()  
    spike.write("motor.stop(port.F, stop = motor.HOLD)\r".encode())
    spike.readline()  
    spike.write("motor.stop(port.B, stop = motor.HOLD)\r".encode())
    spike.readline() 
    endFunction()

    spike.write("def fc():\r".encode()) #FREE COAST MOTORS
    spike.readline()  
    spike.write("motor.stop(port.F, stop = motor.COAST)\r".encode())
    spike.readline()  
    spike.write("motor.stop(port.B, stop = motor.COAST)\r".encode())
    spike.readline() 
    endFunction()

    # centrar el vehiculo normal
    spike.write("async def cv_especial():\r".encode())
    spike.readline() #clear buffer 
    spike.write("await motor.run_to_relative_position(port.F, 0, 550,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("stop = motor.COAST, acceleration = 1000, deceleration = 1000)\r".encode())
    spike.readline() #clear buffer
    endFunction()

    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("runloop.run(cv_especial())\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    endFunction()

    spike.write("def pd(s1,s2,vel,kp,kd,ea):\r".encode()) #PROPORTIONAL DERIVATIVE CONTROL #ea is the previous error
    spike.readline()  
    spike.write("error=s1-s2\r".encode())
    spike.readline()
    spike.write("et= (kp*error) + (kd*(error-ea))\r".encode()) #et is the total error
    spike.readline()
    spike.readline() 
    spike.write("motor.run_to_absolute_position(port.F, int(et), 1100, direction = motor.SHORTEST_PATH, stop = motor.COAST, acceleration = 10000)\r".encode())
    spike.readline() 
    spike.write("motor.set_duty_cycle(port.B, (-100)*(vel))\r".encode())
    spike.readline() 
    spike.write("return error\r".encode())
    spike.readline() 
    endFunction()

    spike.write("def rg(degrees):\r".encode()) #RESET GYRO
    spike.readline() 
    spike.write("motion_sensor.reset_yaw(degrees)\r".encode())
    spike.readline() 
    endFunction()

    spike.write("def ad(vel,reference):\r".encode()) #FORWARD WITH GYRO
    spike.readline() 
    spike.write("global error\r".encode())
    spike.readline() 
    spike.write("error = pd(((10)*(reference)),motion_sensor.tilt_angles()[0],vel,0.5,1,error)\r".encode())
    spike.readline()  
    endFunction()

    spike.write("def turn(side,speed,degrees):\r".encode()) #TURN
    spike.readline() 
    spike.write("motor.run_to_absolute_position(port.F, 140*(side), 1100, stop = motor.HOLD, acceleration = 10000)\r".encode())
    spike.readline() 
    spike.write("while side*(degrees*10) > side*(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() 
    spike.write("motor.set_duty_cycle(port.B, (-100)*(speed))\r".encode())
    spike.readline() 
    spike.write(chr(127).encode()) #SUPR
    spike.readline() 
    spike.write("fc()\r".encode())   
    spike.readline() 
    spike.write("return 255\r".encode())
    spike.readline() 
    endFunction()
    endFunction()

    spike.write("def da(vel, reference):\r".encode()) #DETECT AND GO FORWARD
    spike.readline() 
    spike.write("global error\r".encode())
    spike.readline() 
    spike.write("error = pd((reference)*10,motion_sensor.tilt_angles()[0],vel,0.5,10,error)\r".encode())
    spike.readline() 
    endFunction()

    spike.write("def ag(vel,degrees,reference):\r".encode())
    spike.readline() 
    spike.write("error = 0\r".encode())
    spike.readline() 
    spike.write("motor.reset_relative_position(port.B,0)\r".encode())
    spike.readline() 
    spike.write("while abs(degrees) > abs(motor.relative_position(port.B)):\r".encode())
    spike.readline() 
    spike.write("error = pd(((10)*(reference)),motion_sensor.tilt_angles()[0],vel,0.5,1,error)\r".encode())
    spike.readline() 
    spike.write(chr(127).encode()) #SUPR
    spike.readline() 
    spike.write("fc()\r".encode())
    spike.readline() 
    spike.write("return 255\r".encode())
    spike.readline() 
    endFunction()
    endFunction()
    
def resetSteering():
    spike.write("cv()\r".encode())
    spike.readline() 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"

def motorHold():
    spike.write("hd()\r".encode())
    spike.readline() 

def motorCoast():
    spike.write("fc()\r".encode())
    spike.readline() 
    
def gyroReset(degrees):
    spike.write(("rg("+str(degrees)+")\r").encode())
    spike.readline() 
    
def turn(side,speed,degrees):
    spike.write(("turn("+str(side)+","+str(speed)+","+str(degrees)+")\r").encode())
    spike.readline() 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    motorHold()

def exitParking(side,speed,degrees):
    spike.write(("motor.run_to_absolute_position(port.F, 140*("+str(side)+"), 1100, stop = motor.HOLD, acceleration = 10000)\r").encode())
    spike.readline()
    spike.readline()
    time.sleep(0.001)
    turn(side,speed,degrees)
    
def exitFirstFar(side):
    turn(side,70,85)
    time.sleep(2)
    resetSteering()
    time.sleep(2)
    lidarDistanceForward(70,(90*side),400)
    time.sleep(1)
    turn((side)*-1,70,25)
    resetSteering()
    time.sleep(1)    

def exitFirstNear(side):
    resetSteering()
    time.sleep(1)
    turn((side)*-1,70,0)
    time.sleep(2)
    resetSteering()
    time.sleep(1)

def findSide():
    try:
        obj = vision_queue.get()
        leftDistance = obj.dist_left
        rightDistance = obj.dist_right
        print("left: ", leftDistance, " right: ", rightDistance)
        if leftDistance > rightDistance and leftDistance > 100:
            side = left
            print("turn to the left\n")
        elif rightDistance > leftDistance:
            side = right
            print("turn to the right\n")
        motorCoast()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        print("\nGoodbye world 1")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        motorCoast()
    return side

def cubeDetection():
    global running
    while not vision_camera_queue.full():
        time.sleep(0.001)
        if running == False:
            break
        pass
    return vision_camera_queue.get()


def forwardVoid(vel,reference,leftOrRight):
    global running
    while not vision_queue.full():
        print("QUEUE NOT FULL YET\n")
        time.sleep(0.01)
        if running == False:
            break
        pass
    obj = vision_queue.get()
    if leftOrRight == left:
        while obj.dist_left < 1350:
            spike.write(("da("+str(vel)+","+str(reference)+")\r").encode())
            spike.readline() 
            if vision_queue.full():
                obj = vision_queue.get()
        motorCoast()
    elif leftOrRight == right:
        while obj.dist_right < 1350:
            spike.write(("da("+str(vel)+","+str(reference)+")\r").encode())
            spike.readline() 
            if vision_queue.full():
                obj = vision_queue.get()
        motorCoast()

def lidarDistanceForward(vel,reference,distance):
    global running
    while not vision_queue.full():
        print("QUEUE NOT FULL YET\n")
        time.sleep(0.01)
        if running == False:
            break
        pass
    obj = vision_queue.get()
    print("first dist front: ", obj.dist_front)
    while obj.dist_front > distance:
        spike.write(("da("+str(vel)+","+str(reference)+")\r").encode())
        spike.readline() 
        if vision_queue.full():
            obj = vision_queue.get()
    print("final dist front: ", obj.dist_front)
    motorCoast()

def goForward(speed,degrees,reference):
    spike.write(("ag("+str(speed)+","+str(degrees)+","+str(reference)+")\r").encode())
    spike.readline() 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    motorCoast()

def radiansToDegrees(radianes):
    degrees : int = (radianes)*(180/np.pi)
    return degrees

def dos_puntos(speed, degrees, reference,leftOrRight):
    global running
    while not vision_queue.full():
        print("QUEUE NOT FULL YET\n")
        time.sleep(0.01)
        if running == False:
            break
        pass
    obj = vision_queue.get()
    if leftOrRight == left:
        print("left")
        y1 = obj.dist_left
    elif leftOrRight == right:
        print("right")
        y1 = obj.dist_right
    else:
        print("no hay leftOrRight")
    spike.write(("ag("+str(speed)+","+str(degrees)+","+str(reference)+")\r").encode())
    spike.readline() 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    motorCoast()
    while not vision_queue.full():
        print("WAITING\n")
        time.sleep(0.01)
        if running == False:
            break
        pass
    obj = vision_queue.get()
    if leftOrRight == left:
        print("left")
        y2 = obj.dist_left
    elif leftOrRight == right:
        print("right")
        y2 = obj.dist_right
    else:
        print("no hay leftOrRight")
    variacion = y2 - y1
    pendiente = int(radiansToDegrees(np.arctan(variacion / abs(175*degrees/360)))*-10)
    print(pendiente)
    return pendiente


#=================================== GPIO functions ================================
def setup_gpio():
    global relay, led, button
    try:
        h = lgpio.gpiochip_open(0) #se habilita el acceso a los gpio
        lgpio.gpio_claim_input(h, button)
        lgpio.gpio_claim_output(h, relay)
        lgpio.gpio_claim_output(h, led)
        lgpio.gpio_write(h, relay, 1)
        lgpio.gpio_write(h, led, 0) #TURN OFF LED (ROBOT NOT READY)
        time.sleep(1)
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
    except KeyboardInterrupt:
        print("\nGoodbye world 2")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        motorCoast()

#funcion que limpia los gpio
def cleanup_gpio(h):
    try:
        lgpio.gpio_write(h, relay, 1)
        lgpio.gpio_free(h, button)
        lgpio.gpiochip_close(h)
        print("GPIO libres")
    except KeyboardInterrupt:
        print("\nGoodbye world 3")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        motorCoast()
    except Exception as e:
        print(f"Error al limpiar GPIO: {e}")

#=================================== camera functions ================================
# nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

def displayFrame(name, frame, detections):
    color = (255, 0, 0)
    for detection in detections:
        bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        cv2.putText(frame, labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
    # Show the frame
    cv2.imshow(name, frame)

#=================================== threads ================================

def camera_Worker():
    global running
    with dai.Device(pipeline) as device:
        
        best = PillarObservation(None,0,0,0)
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        frame = None
        detections = []
        startTime = time.monotonic()
        counter = 0
        color2 = (255, 255, 255)
        while running == True:
            try:
                area = [0,0,0,0,0,0,0,0,0,0]
                max_index = 0
                inRgb = qRgb.get()
                inDet = qDet.get()
                best = PillarObservation(None,0,0,0)
                if inRgb is not None:
                    frame = inRgb.getCvFrame()
                    cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                                (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

                if inDet is not None:
                    detections = inDet.detections
                    counter += 1         
                    i = 0      
                    for detection in detections:     
                        area[i] = ((detection.xmax - detection.xmin) * (detection.ymax - detection.ymin))*(100)
                        i += 1 
                    if i != 0:
                        max_index = np.argmax(area)
                        print("max index: ", max_index, " detections: ", detections[max_index].label,"\n")
                        xmid = (detections[max_index].xmin + detections[max_index].xmax)/2
                        ymid = (detections[max_index].ymin + detections[max_index].ymax)/2
                        best = PillarObservation(label=detections[max_index].label, xmid=xmid, ymid=ymid, area=area[max_index])
                if best is not None:
                    if vision_camera_queue.full():
                        try:
                            vision_camera_queue.get_nowait()
                        except queue.Empty:
                            pass
                    vision_camera_queue.put(best)

                if frame is not None:
                    displayFrame("rgb", frame, detections)

                if cv2.waitKey(1) == ord('q') or running == False:
                    cv2.destroyAllWindows()
                    break
                
            except KeyboardInterrupt:
                print("\nGoodbye world 4")
                print("Stopping camera thread.")
                running = False
                
        

def main_(h):
    global running
    #lidar.start_motor()
    #lidar.start() # star scaning measurements
    time.sleep(3)  # Give motor time to spin up
    while not vision_queue.full():
        #print("WAITING FOR QUEUE TO BE FILLED\n")
        time.sleep(0.01)
        if running == False:
            break
        pass
    v = 0
    
    #BUTTON PRESS TO START
    """lgpio.gpio_write(h, led, 1) #TURN ON LED (ROBOT READY)
    print("PRESS BUTTON TO START\n")
    while lgpio.gpio_read(h, button) == 1: #WAIT FOR BUTTON PRESS
        time.sleep(0.01)
        pass
    lgpio.gpio_write(h, led, 0) #TURN OFF LED (BUTTON PRESSED)
    """
    gyroReset(0)
    side = findSide()
    exitParking(side,70,65)
    time.sleep(0.2)
    cube = cubeDetection()
    time.sleep(2)
    print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
    if cube.label == greenbox:
        exitFirstFar(side)
    elif cube.label == redbox or cube.label == None:
        exitFirstNear(side)
    if side == left:
        while v < 3:
            if cube.label == greenbox:
                time.sleep(1)
                cube = cubeDetection()
                print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                if cube.label == greenbox or cube.label == None:
                    turn(side,70,(60*side))
                    resetSteering()
                    time.sleep(1)
                    cube = cubeDetection()
                    print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                    resetSteering()
                    time.sleep(1)
                    goForward(70,400,70)
                    resetSteering()
                    time.sleep(1)
                    if cube.label == greenbox or cube.label == None:
                        turn(side,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,side)
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                    elif cube.label == redbox:
                        (print("redbox detected on third\n"))
                        goForward(70,400,70)
                        resetSteering()
                        time.sleep(1)
                        turn((side)*-1,70,0)
                        resetSteering()
                        time.sleep(1)
                        lidarDistanceForward(70,0,350)
                        turn(side,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,(side*-1))
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                elif cube.label == redbox:
                    turn((side)*-1,70,0)
                    resetSteering()
                    time.sleep(1)
                    lidarDistanceForward(70,0,350)
                    turn(side,70,(100*side))
                    resetSteering()
                    time.sleep(1)
                    cube = cubeDetection()
                    print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                    time.sleep(1)
                    if cube.label == redbox or cube.label == None:
                        turn((side)*-1,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,(side*-1))
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                    elif cube.label == greenbox:
                        turn(side,70,(170*side))
                        resetSteering()
                        time.sleep(1)
                        lidarDistanceForward(70,170,350)
                        turn((side)*-1,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,side)
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                
            elif cube.label == redbox or cube.label == None:
                turn(side,70,(60*side))
                time.sleep(1)
                cube = cubeDetection()
                print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                if cube.label == greenbox or cube.label == None:
                    print("green or none\n")
                    turn(side,70,(85*side))
                    resetSteering()
                    time.sleep(1)
                    goForward(70,1000,85)
                    turn((side*-1),70,(60*side))
                    resetSteering()
                    time.sleep(1)
                    cube = cubeDetection()
                    print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                    resetSteering()
                    time.sleep(1)
                    goForward(70,400,70)
                    resetSteering()
                    time.sleep(1)
                    if cube.label == greenbox or cube.label == None:
                        turn(side,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,side)
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                    elif cube.label == redbox:
                        (print("redbox detected on third\n"))
                        goForward(70,400,70)
                        resetSteering()
                        time.sleep(1)
                        turn((side)*-1,70,0)
                        resetSteering()
                        time.sleep(1)
                        lidarDistanceForward(70,0,350)
                        turn(side,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,(side*-1))
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                elif cube.label == redbox:
                    (print("redbox detected on second\n"))
                    resetSteering()
                    time.sleep(1)
                    turn((side)*-1,70,0)
                    resetSteering()
                    time.sleep(1)
                    lidarDistanceForward(70,0,350)
                    resetSteering()
                    time.sleep(1)
                    turn(side,70,(85*side))
                    resetSteering()
                    time.sleep(1)
                    goForward(70,1000,90)
                    resetSteering()
                    time.sleep(1)
                    turn(side,70,(100*side))
                    resetSteering()
                    time.sleep(1)
                    cube = cubeDetection()
                    print("cube detected with label: ", cube.label, " area: ", cube.area, "\n")
                    time.sleep(1)
                    if cube.label == redbox or cube.label == None:
                        turn((side)*-1,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,(side*-1))
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
                    elif cube.label == greenbox:
                        turn(side,70,(170*side))
                        resetSteering()
                        time.sleep(1)
                        lidarDistanceForward(70,170,350)
                        turn((side)*-1,70,(85*side))
                        resetSteering()
                        time.sleep(1)
                        angulo = dos_puntos(70,1000,90,side)
                        gyroReset(angulo)
                        forwardVoid(70,0,side)
            resetSteering()
            time.sleep(1)
            gyroReset(0)
            if cube.label == greenbox:
                turn(side,70,25)
                resetSteering()
                time.sleep(1)
            else:
                pass
            v += 1
            print(v,"secciones\n")
            time.sleep(3)
        
        
        
    elif side == right:
        if cube.label == redbox:
            exitFirstFar(side)
        elif cube.label == greenbox or cube.label == None:
            exitFirstNear(side)
            
    resetSteering()
    time.sleep(10)
    running = False
    print(" ================================ RPLIDAR THREAD ENDING ================================= ")

def lidarWorker():
    global running, spike
    h, success = setup_gpio()
    try:
        lidar.connect()
        info = lidar.get_info()
        for key, value in info.items():
            print('{0:<13}: {1}'.format(key.capitalize(), str(value)))

        health = lidar.get_health()
        print('Health Status: {0[0]} - {0[1]}'.format(health))

        
        print('*' * 50)
        
        lidar.start_motor()
        time.sleep(4)  # Give motor time to spin up

    
        spikeStartup()
        spikeLibraries()
        print("empezando ....\n")
        
        gyroReset(0)

        #lidar.start() # star scaning measurements       
        try:
            angles = [0]*360 # array de 360 lugares para guardar las medidas del lidar
            iterator = lidar.iter_scans(max_buf_meas=32000,min_len=10)
            threading.Thread(target=main_,args=(h,)).start()
            threading.Thread(target=camera_Worker).start()
            time.sleep(2)
            for scan in iterator:
                for (_, angle, distance) in scan:
                    if distance is not None:
                        angles[min([359,floor(angle)])] = distance
                time.sleep(0.001)
                #print(angles[0],angles[90],angles[270])
                lidar_data = RPlidar_data(angles[90],angles[75],angles[285],angles[270],angles[0],time.time())
                if vision_queue.full(): #check if the queue is full
                    try:
                        #print("queue full")
                        vision_queue.get() # erase old values from the queue to have latest information 
                    except vision_queue.not_empty:
                        pass
                    except KeyboardInterrupt:
                        print("\nGoodbye world 5")
                        lidar.stop()
                        lidar.stop_motor()
                        lidar.disconnect()
                        motorCoast()
                vision_queue.put(lidar_data) #load new data on the queue
                vision_queue.task_done()
                if running == False:
                    break
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            

        except RPLidarException as e:
            print("error in lidar:", e)
            lidar.clean_input()
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            running = False
        except Exception as e:
            print("error in lidar-aritmetica:", e)
            lidar.clean_input()
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            running = False
        except KeyboardInterrupt:
            print("\nGoodbye world 6")
            print("\nProgram interrupted! Cleaning up...")
            print("\nStopping...")
            running = False
            cleanup_gpio(h)
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            running = False
        
        #distancias = distance_queue.get_nowait()
                
        #angulo_correccion = correccion(15)

        """if vision_queue.full():
            try:
                obj = vision_queue.get_nowait()
                print(obj.dist_left, obj.dist_75, obj.dist_315, obj.dist_right)
            except queue.Empty:
                pass"""
        
        running = False
            
            
    except Exception as e:
        print(f"Error: {e}")
        cleanup_gpio(h)
        running = False
    except KeyboardInterrupt:
        print("\nGoodbye world 7")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        motorCoast()
        running = False
    finally:
        running = False
        lidar.stop()
        lidar.disconnect()
        cleanup_gpio(h)
        running = False

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("-m", "--model", help="Provide model name or model path for inference",
                    default='yolov4_tiny_coco_416x416', type=str)
parser.add_argument("-c", "--config", help="Provide config path for inference",
                    default='json/yolov4-tiny.json', type=str)
args = parser.parse_args()

# parse config
configPath = Path(args.config)
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

# parse input shape
if "input_size" in nnConfig:
    W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# extract metadata
metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})

print(metadata)

# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})

# get model path
nnPath = args.model
if not Path(nnPath).exists():
    print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
    nnPath = str(blobconverter.from_zoo(args.model, shaves = 6, zoo_type = "depthai", use_cache=True))
# sync outputs
syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
xoutRgb = pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
nnOut.setStreamName("nn")

# Properties
camRgb.setPreviewSize(W, H)

camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(30)

# Network specific settings
detectionNetwork.setConfidenceThreshold(confidenceThreshold)
detectionNetwork.setNumClasses(classes)
detectionNetwork.setCoordinateSize(coordinates)
detectionNetwork.setAnchors(anchors)
detectionNetwork.setAnchorMasks(anchorMasks)
detectionNetwork.setIouThreshold(iouThreshold)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)

# Linking
camRgb.preview.link(detectionNetwork.input)
detectionNetwork.passthrough.link(xoutRgb.input)
detectionNetwork.out.link(nnOut.input)

lidarWorker()
print("Exiting...")