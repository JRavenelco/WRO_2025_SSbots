import threading
import queue
import signal
import sys
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple
import serial
import subprocess
import os
import pty
import time
import socket
import subprocess
from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
import lgpio
import time
import lgpio

# GPIO pin definitions
led    = 17  # GPIO pin for the LED
button = 27  # GPIO pin for the button

#Motor port definitions
driveMotor = port.A
steeringMotor = port.E
# Configuración de GPIO
def setup_gpio():
    """Initialize GPIO and claim all necessary pins"""
    try:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(h, button)  # Input pin for button
        lgpio.gpio_claim_output(h, led)    # Output pin for LED
        lgpio.gpio_write(h, led, 1)        # LED encendido al inicio
        return h, True
    except Exception as e:
        print(f"GPIO initialization failed: {e}")
        return None, False

def cleanup_gpio(h):
    """Release all GPIO resources"""
    try:
        lgpio.gpio_free(h, button) # Free button pin
        lgpio.gpio_free(h, led)    # Free LED pin
        lgpio.gpiochip_close(h)
    except:
        pass

# Espera para asegurar que el sistema esté listo
time.sleep(5)

# Inicializa el display
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=32)

font = ImageFont.load_default()  # SIEMPRE usa la fuente por defecto

# Setup GPIO pins
h, success = setup_gpio()
if not success:
    print("Failed to initialize GPIO")
    exit(1)

try:
    last_update = 0
    ip_text = ""
    while True:
        # Si el botón es presionado, salir del loop inmediatamente
        if lgpio.gpio_read(h, button) == 0:
            print("Botón presionado, saliendo...")
            break

        now = time.monotonic()
        if now - last_update >= 5 or ip_text == "":
            # Obtener IP y SSID
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                ip_address = s.getsockname()[0]
                s.close()
                try:
                    ssid = subprocess.check_output(['iwgetid', '-r'], text=True).strip()
                except (subprocess.CalledProcessError, FileNotFoundError):
                    ssid = "N/A"
                ip_text = f"SSID: {ssid}\nIP: {ip_address}"
            except OSError:
                ip_text = "OFFLINE"

            # Crear imagen y dibujar texto
            image = Image.new("1", (device.width, device.height))
            draw = ImageDraw.Draw(image)
            bbox = draw.multiline_textbbox((0, 0), ip_text, font=font, spacing=1)
            x = (device.width - (bbox[2] - bbox[0])) // 2
            y = (device.height - (bbox[3] - bbox[1])) // 2
            draw.multiline_text((x, y), ip_text, font=font, fill=255, spacing=1, align="center")
            device.display(image)
            last_update = now

        time.sleep(0.05)  # Pequeña espera para no consumir CPU

finally:
    # Apaga el LED y limpia recursos
    lgpio.gpio_write(h, led, 0)
    cleanup_gpio(h)
    device.clear()