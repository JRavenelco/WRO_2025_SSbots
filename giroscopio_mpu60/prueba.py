# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Display inclination data five times per second

# See this page to learn the math and physics principals behind this example:
# https://learn.adafruit.com/how-tall-is-it/gravity-and-acceleration

import time
import math

from mpu9250_jmdev.registers import * 
from mpu9250_jmdev.mpu_9250 import MPU9250 

# Crear una instancia MPU9250 
mpu = MPU9250( 
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # En caso de que el MPU9250 esté conectado a otro dispositivo I2C 
    address_mpu_slave=None, 
    bus=1, 
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ) 

# Configurar el MPU9250 
mpu.configure() 
time.sleep(1)

def complementary_filter(gyro_data, accel_data, prev_angle, alpha=0.98):
    """
    Applies a complementary filter to combine gyroscope and accelerometer data.

    Args:
        gyro_data: Gyroscope data (angular rate).
        accel_data: Accelerometer data (acceleration).
        prev_angle: Previous angle estimate.
        alpha: Weighting factor (0 < alpha < 1).

    Returns:
        The updated angle estimate.
    """
    # Gyroscope integration
    angle_from_gyro = prev_angle + gyro_data * dt

    # Accelerometer angle calculation
    accel_angle = math.atan2(accel_data[1], accel_data[0]) 
    
    # Complementary filter
    angle =  accel_angle
    return angle

# Example usage
dt = 0.01  # Sample time (seconds)
angle = 0.0
while True:
    gyro_data = mpu.readGyroscopeMaster()[0]  # Replace with your function
    accel_data = mpu.readAccelerometerMaster() # Replace with your function
    angle = complementary_filter(gyro_data, accel_data, angle, alpha=0.98)
    print(f"Angle: {angle:.2f} degrees")
    time.sleep(dt)
# Given a point (x, y) return the angle of that point relative to x axis.
# Returns: angle in degrees


def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


# Given an accelerometer sensor object return the inclination angles of X/Z and Y/Z
# Returns: tuple containing the two angles in degrees


def get_inclination(_sensor):
    x, y, z = _sensor.readAccelerometerMaster()
    return vector_2_degrees(x, z), vector_2_degrees(y, z)


while True:
    angle_xz, angle_yz = get_inclination(mpu)
    print(f"XZ angle = {angle_xz:6.2f}deg   YZ angle = {angle_yz:6.2f}deg")
    