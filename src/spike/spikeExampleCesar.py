import serial

spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def initialize_Libraries():
    spike.write("import motor\r".encode()) 
    spike.readline()
    spike.write("from hub import port\r".encode())
    spike.readline()#clear buffer
    spike.write("from hub import motion_sensor\r".encode())
    spike.readline()#clear buffer
    #declare functions for motors
    spike.write("def fr():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.F, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.B, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    # centrar el vehiculo 
    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.run_to_absolute_position(port.F, 0, 550,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write(" direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 1000, deceleration = 1000)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #pd 
    spike.write("def pd(s1,s2,vel,kp,kd,ea):\r".encode()) #ea es error anterior
    spike.readline() #clear buffer 
    spike.write("error=s1-s2\r".encode())
    spike.readline()#clear buffer
    spike.write("et= (kp*error) + (kd*(error-ea))\r".encode()) #et es error total
    spike.readline()#clear buffer
    spike.write(" motor.run_to_absolute_position(port.F, int(et), 550, direction = motor.SHORTEST_PATH)\r".encode())
    spike.readline() #clear buffer
    spike.write(" motor.set_duty_cycle(port.B, (-100)*vel)\r".encode())
    spike.readline() #clear buffer
    spike.write(" return error\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #reset Gyro
    spike.write("def rg():\r".encode())
    spike.readline() #clear buffer
    spike.write("motion_sensor.reset_yaw(0)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #variables globales
    spike.write("def ad(vel,distancia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("while 0 > distance_sensor.distance(port.C) or distance_sensor.distance(port.C) > distancia:\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(0,motion_sensor.tilt_angles()[0],vel,0.5,1,error)\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

def center_vehicle():
    spike.write("cv()\r".encode())
    spike.readline() #clear buffer

def Free_spikeDirection():
    spike.write("fr()\r".encode())
    spike.readline() #clear buffer

def end_Function():
    spike.write("\r".encode())
    spike.readline()#clear 
    spike.write("\r".encode())
    spike.readline()#clear buffer
    spike.write("\r".encode())
    spike.readline()#clear buffer
    
def reset_gyro():
    spike.write("rg()\r".encode())
    spike.readline() #clear buffer
    
def avanzar_distancia(vel,distancia):
    spike.write(("ad("+str(vel)+","+str(distancia)+")\r").encode())
    spike.readline() #clear buffer

# Main Program
initialize_Libraries()
reset_gyro()
avanzar_distancia(50,750)
Free_spikeDirection()

spike.close() #close serial connection