import cv2
import depthai as dai
from pathlib import Path
import sys
import tty
import termios

# --- Configuración ---
BASE_PATH = Path("/home/maker/spike_ssbots/WRO_2025_SSbots-sebas/oak/dataset")
FOLDER_OPTIONS = {
    "1": {"name": "greenbox", "prefix": "g"},
    "2": {"name": "redbox", "prefix": "r"},
    "3": {"name": "combined", "prefix": "c"},
}

# --- Selección de Carpeta ---
def select_folder():
    """Pide al usuario que seleccione una carpeta y devuelve la configuración."""
    print("Seleccione la carpeta para guardar las imágenes:")
    for key, value in FOLDER_OPTIONS.items():
        print(f"  {key}: {value['name']}")

    while True:
        choice = input("Ingrese el número de su elección: ")
        if choice in FOLDER_OPTIONS:
            return FOLDER_OPTIONS[choice]
        else:
            print("Opción no válida. Por favor, intente de nuevo.")

# --- Obtener Siguiente Índice ---
def get_next_index(folder_path, prefix):
    """
    Calcula el siguiente índice para el nombre de archivo basado en los archivos existentes.
    Si la carpeta no existe, la crea.
    """
    if not folder_path.exists():
        print(f"La carpeta {folder_path} no existe. Creándola...")
        folder_path.mkdir(parents=True, exist_ok=True)
        return 0
    
    existing_files = list(folder_path.glob(f'{prefix}*.jpg'))
    return len(existing_files)

# --- Script Principal ---
if __name__ == "__main__":
    config = select_folder()
    target_folder_name = config["name"]
    file_prefix = config["prefix"]
    target_path = BASE_PATH / target_folder_name

    img_counter = get_next_index(target_path, file_prefix)
    print(f"Guardando imágenes en: '{target_path}'")
    print(f"Prefijo de archivo: '{file_prefix}'. Próxima imagen será: {file_prefix}{img_counter}.jpg")

    # Crear pipeline de DepthAI para captura bajo demanda
    pipeline = dai.Pipeline()

    # Nodo de la cámara a color
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    # Establecer la resolución máxima del sensor (13MP para OAK-D Lite)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_13_MP)
    # La salida 'still' produce un frame de alta resolución cuando se le ordena
    # Ajustar el tamaño del 'still' a la resolución máxima
    cam_rgb.setStillSize(4208, 3120)

    # Nodo de entrada para el control de la cámara
    control_in = pipeline.create(dai.node.XLinkIn)
    control_in.setStreamName("control")
    control_in.out.link(cam_rgb.inputControl)

    # Nodo de salida para las imágenes capturadas
    xout_still = pipeline.create(dai.node.XLinkOut)
    xout_still.setStreamName("still")
    cam_rgb.still.link(xout_still.input)

    # Guardar la configuración original de la terminal
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Poner la terminal en modo "raw" para leer teclas individuales
        tty.setcbreak(sys.stdin.fileno())

        # Iniciar el dispositivo
        with dai.Device(pipeline) as device:
            # Obtener colas de entrada y salida
            control_queue = device.getInputQueue("control")
            still_queue = device.getOutputQueue("still", maxSize=1, blocking=True)
            
            print("\nDispositivo OAK-D listo.")
            print("Presione 'p' para tomar una foto.")
            print("Presione 'q' para salir.")

            while True:
                # Esperar a que se presione una tecla
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    print("\nSaliendo...")
                    break
                elif key.lower() == 'p':
                    print(f"\rCapturando...", end="", flush=True)
                    
                    # Enviar señal para capturar una imagen
                    ctrl = dai.CameraControl()
                    ctrl.setCaptureStill(True)
                    control_queue.send(ctrl)
                    
                    # Esperar a que la imagen llegue del dispositivo (bloqueante)
                    in_still = still_queue.get()
                    frame = in_still.getCvFrame()
                    
                    # Guardar la imagen
                    img_name = f"{file_prefix}{img_counter}.jpg"
                    img_path = target_path / img_name
                    cv2.imwrite(str(img_path), frame)
                    
                    # \r vuelve al inicio de la línea para sobreescribir el mensaje
                    print(f"\r¡Foto guardada! -> {img_path}          ", flush=True)
                    img_counter += 1

    finally:
        # Restaurar la configuración de la terminal al salir
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nScript finalizado.")