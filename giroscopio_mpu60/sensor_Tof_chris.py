import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C and VL53L0X sensor
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c,0x27)
vl53.set_address(0x29)

try:
    while True:
        distance = vl53.range  # Read distance in millimeters
        if distance > 8000:
            distance = -1
            print(f"Distance: {distance} mm")
        else:
            print(f"Distance: {distance} mm")
        time.sleep(0.025)
except KeyboardInterrupt:
    print("Exiting...")
