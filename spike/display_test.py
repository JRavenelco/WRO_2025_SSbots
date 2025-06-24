import time
# Espera 3 segundos para que el sistema se inicialice completamente
time.sleep(3)

import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import subprocess

# Configuración del bus I2C y del display
i2c = busio.I2C(board.SCL, board.SDA)
disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Limpia la pantalla al iniciar
disp.fill(0)
disp.show()

# Crea una imagen para dibujar
width = disp.width
height = disp.height
image = Image.new("1", (width, height))
draw = ImageDraw.Draw(image)

# Carga la fuente por defecto
font = ImageFont.load_default()

# Intenta obtener la dirección IP y el SSID
try:
    # Obtener IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_address = s.getsockname()[0]
    s.close()

    # Obtener SSID de la red WiFi
    try:
        ssid = subprocess.check_output(['iwgetid', '-r'], text=True).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Si iwgetid falla, puede ser una conexión Ethernet o no estar disponible
        ssid = "N/A"

    ip_text = f"Status: ONLINE\nSSID: {ssid}\nIP: {ip_address}"

except OSError:
    ip_text = "Status: OFFLINE\nSSID: NONE\nIP: NONE"

# Limpia la imagen (fondo negro)
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Calcula la posición para centrar el texto usando multiline_textbbox
# El parámetro 'spacing' controla el interlineado en píxeles.
bbox = draw.multiline_textbbox((0, 0), ip_text, font=font, spacing=1)
x = (width - (bbox[2] - bbox[0])) // 2
y = (height - (bbox[3] - bbox[1])) // 2

# Dibuja el texto en la imagen usando multiline_text
# Añadimos 'spacing=1' y 'align="center"' para un mejor formato.
draw.multiline_text((x, y), ip_text, font=font, fill=255, spacing=1, align="center")

# Muestra la imagen en el display
disp.image(image)
disp.show()