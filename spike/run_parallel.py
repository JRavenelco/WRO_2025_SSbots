import multiprocessing
import time
import sys
import os
import signal # Para manejar señales de terminación

# Añadir las rutas de los directorios que contienen los scripts al path de Python
# Ajusta estas rutas si tus scripts están en otro lugar
motor_control_dir = "/home/pi/WRO_2025/spike"
yolo_dir = "/home/pi/depthai-python/examples/Yolo" 

# Verificar si las rutas existen antes de añadirlas
if os.path.isdir(motor_control_dir):
    sys.path.insert(0, motor_control_dir) 
else:
    print(f"Advertencia: Directorio no encontrado {motor_control_dir}")

if os.path.isdir(yolo_dir):
    # Añadir al final es generalmente más seguro para no sobreescribir módulos estándar
    sys.path.append(yolo_dir) 
else:
    print(f"Advertencia: Directorio no encontrado {yolo_dir}")

# Importar los módulos refactorizados
try:
    import motorControlBinden
    import tiny_yolo
except ImportError as e:
    print(f"Error al importar módulos: {e}")
    print("Asegúrate de que las rutas en sys.path sean correctas y los archivos __init__.py existan si son paquetes.")
    sys.exit(1)

# Variables globales para los procesos
motor_process = None
yolo_process = None

def motor_process_target_wrapper():
    """Función wrapper para capturar excepciones en el proceso motor."""
    print(f"[Motor Process {os.getpid()}] Iniciando...")
    try:
        motorControlBinden.main_loop()
    except Exception as e:
        print(f"[Motor Process {os.getpid()}] Error: {e}")
    finally:
        print(f"[Motor Process {os.getpid()}] Finalizado.")

def yolo_process_target_wrapper():
    """Función wrapper para capturar excepciones en el proceso YOLO."""
    print(f"[YOLO Process {os.getpid()}] Iniciando...")
    try:
        # Puedes pasar argumentos a main_loop si es necesario, ej:
        # tiny_yolo.main_loop(nn_model_path='/path/to/your/model.blob')
        tiny_yolo.main_loop() 
    except Exception as e:
        print(f"[YOLO Process {os.getpid()}] Error: {e}")
    finally:
        print(f"[YOLO Process {os.getpid()}] Finalizado.")

def shutdown_handler(signum, frame):
    """Manejador para señales de terminación (SIGINT, SIGTERM)."""
    print(f"\nRecibida señal {signal.Signals(signum).name}. Terminando procesos...")
    
    global motor_process, yolo_process
    
    # Terminar procesos hijos de forma segura
    if yolo_process and yolo_process.is_alive():
        print("Terminando proceso YOLO...")
        yolo_process.terminate() # Envía SIGTERM
    
    if motor_process and motor_process.is_alive():
        print("Terminando proceso de control de motor...")
        motor_process.terminate() # Envía SIGTERM

    # Esperar a que los procesos terminen
    if yolo_process:
        yolo_process.join(timeout=5)
        if yolo_process.is_alive():
            print("Proceso YOLO no terminó, forzando...")
            yolo_process.kill() # Envía SIGKILL si SIGTERM falló
            yolo_process.join()

    if motor_process:
        motor_process.join(timeout=5)
        if motor_process.is_alive():
            print("Proceso motor no terminó, forzando...")
            motor_process.kill() # Envía SIGKILL si SIGTERM falló
            motor_process.join()

    print("Todos los procesos han sido terminados.")
    sys.exit(0) # Salir del script principal

if __name__ == '__main__':
    print("Iniciando script principal...")

    # Registrar manejadores de señales para terminación limpia
    signal.signal(signal.SIGINT, shutdown_handler)  # Manejar Ctrl+C
    signal.signal(signal.SIGTERM, shutdown_handler) # Manejar kill/systemd stop

    try:
        # Crear procesos
        print("Creando procesos...")
        motor_process = multiprocessing.Process(target=motor_process_target_wrapper, name="MotorControl")
        yolo_process = multiprocessing.Process(target=yolo_process_target_wrapper, name="YoloDetection")

        # Iniciar procesos
        print("Iniciando proceso de control de motor...")
        motor_process.start()
        # Dar un pequeño respiro antes de iniciar el siguiente, especialmente si usan hardware
        time.sleep(3) 
        
        print("Iniciando proceso YOLO...")
        yolo_process.start()

        print("Procesos iniciados. Esperando (o presiona Ctrl+C para salir)...")
        
        # Mantener el proceso principal vivo y esperando
        # join() esperará indefinidamente hasta que el proceso termine por sí mismo o por señal
        motor_process.join() 
        yolo_process.join()

    except Exception as e:
        print(f"Error en el script principal: {e}")
        # Intentar limpiar si hubo un error inesperado en el script principal
        shutdown_handler(signal.SIGTERM, None) 
    finally:
        print("Script principal finalizado.")
