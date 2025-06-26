import time
import socket
import subprocess
from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

# Espera para asegurar que el sistema esté listo
time.sleep(5)

# Inicializa el display
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, width=128, height=32)

font = ImageFont.load_default()

try:
    while True:
        # Obtener IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
            # Obtener SSID
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

        time.sleep(5)
finally:
    # Limpia el display al salir
    device.clear()