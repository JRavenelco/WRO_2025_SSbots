import serial
import time
import sys
from evdev import InputDevice, categorize, ecodes

# --- Configuración Serial ---
MOTOR_PORT_B = 'B' # Puerto para Arriba/Abajo
MOTOR_PORT_D = 'D' # Puerto para Izquierda/Derecha
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
MOTOR_SPEED = 750 # Velocidad del motor (ajusta según necesites)
# ---------------------------

# --- Configuración Control ---
CONTROLLER_PATH = '/dev/input/event0' 
# Códigos de evento para el D-Pad (basado en evtest)
DPAD_X_CODE = ecodes.ABS_X # Suele ser 0
DPAD_Y_CODE = ecodes.ABS_Y # Suele ser 1
BTN_TOUCH_CODE = 330       # Código para BTN_TOUCH

# --- Umbrales ajustados según la salida de evtest ---
# Valores observados:
# Izquierda: X≈155
# Derecha:   X≈757
# Abajo:     Y≈983
# Arriba:    Y≈1485
# Centro X (estimado/visto): ≈380-600?
# Centro Y (estimado/visto): ≈1100-1300?

X_CENTER_APPROX = 450      # Valor X aproximado cuando está centrado
Y_CENTER_APPROX = 1200     # Valor Y aproximado cuando está centrado

X_THRESHOLD_LOW = 250      # Valor X por debajo del cual se considera Izquierda
X_THRESHOLD_HIGH = 600     # Valor X por encima del cual se considera Derecha
Y_THRESHOLD_LOW = 1100     # Valor Y por debajo del cual se considera Abajo 
Y_THRESHOLD_HIGH = 1350    # Valor Y por encima del cual se considera Arriba
# ----------------------------------------------------

def send_command(fd, command):
    """Envía un comando al Brick y espera un poco."""
    #print(f"Enviando: {command.strip()}") # Descomenta para depurar comandos
    try:
        fd.write(command.encode())
        time.sleep(0.05) # Pequeña pausa para no saturar el puerto serial
    except serial.SerialException as e:
        print(f"Error al enviar comando: {e}")
    except Exception as e:
        print(f"Error inesperado en send_command: {e}")

def main_loop(): # <--- Función principal
    motor_b_running = False 
    motor_d_running = False 
    dpad_touched = False
    fd = None
    controller = None

    try:
        # Conectar al control
        controller = InputDevice(CONTROLLER_PATH)
        print(f"Control conectado: {controller.name}")

        # Conectar al Spike Brick
        fd = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # Añadir timeout
        print("Conectado al Spike Brick.")
        
        # Inicializar motores en el Brick
        send_command(fd, 'import motor\r')
        send_command(fd, 'from hub import port\r')
        print("Motor y puerto importados en el Brick.")

        # Bucle principal para leer eventos del control
        print("Esperando eventos del control...")
        current_x = X_CENTER_APPROX 
        current_y = Y_CENTER_APPROX 

        # Variables para solo avance
        should_run_motor_b = False
        should_run_motor_d = False

        for event in controller.read_loop():
            
            # Valores para esta iteración
            new_should_run_motor_b = False
            new_should_run_motor_d = False

            # --- Lógica de lectura de eventos ---
            if event.type == ecodes.EV_KEY and event.code == BTN_TOUCH_CODE:
                dpad_touched = (event.value == 1)
                if not dpad_touched:
                    current_x = X_CENTER_APPROX
                    current_y = Y_CENTER_APPROX
                    print("D-Pad soltado - Detener motores")
            elif event.type == ecodes.EV_ABS:
                if dpad_touched: 
                    if event.code == DPAD_X_CODE:
                        current_x = event.value
                        # print(f"X: {current_x}")  # Descomenta para depurar
                    elif event.code == DPAD_Y_CODE:
                        current_y = event.value
                        # print(f"Y: {current_y}")  # Descomenta para depurar

            # --- Lógica de determinación de estado (solo avance) ---
            if dpad_touched:
                # Motor B - Solo activar con Arriba (Y alto)
                if current_y > Y_THRESHOLD_HIGH:
                    new_should_run_motor_b = True
                    print("Botón Arriba: Activar Motor B")
                
                # Motor D - Solo activar con Derecha (X alto)
                if current_x > X_THRESHOLD_HIGH:
                    new_should_run_motor_d = True
                    print("Botón Derecha: Activar Motor D")
            
            # --- Lógica de actualización de motores (solo avance) ---
            if new_should_run_motor_b != should_run_motor_b:
                should_run_motor_b = new_should_run_motor_b
                if should_run_motor_b:
                    print("Motor B: Avanzando")
                    send_command(fd, f'motor.run(port.{MOTOR_PORT_B}, {MOTOR_SPEED})\r')
                else:
                    print("Motor B: Detenido")
                    send_command(fd, f'motor.stop(port.{MOTOR_PORT_B})\r')

            if new_should_run_motor_d != should_run_motor_d:
                should_run_motor_d = new_should_run_motor_d
                if should_run_motor_d:
                    print("Motor D: Avanzando") 
                    send_command(fd, f'motor.run(port.{MOTOR_PORT_D}, {MOTOR_SPEED})\r')
                else:
                    print("Motor D: Detenido")
                    send_command(fd, f'motor.stop(port.{MOTOR_PORT_D})\r')

    except FileNotFoundError:
        print(f"Error: No se encontró el control en {CONTROLLER_PATH}.")
    except serial.SerialException as e:
        print(f"Error: No se pudo conectar al Spike Brick en {SERIAL_PORT}. {e}")
    except PermissionError:
        print(f"Error: Permiso denegado para acceder a {CONTROLLER_PATH}. Ejecuta con sudo.")
    except KeyboardInterrupt:
        print("\nDeteniendo control de motor (KeyboardInterrupt).")
    except Exception as e:
        print(f"Ocurrió un error inesperado en control motor: {e}")
    finally:
        print("Limpiando recursos del control de motor...")
        if fd and fd.is_open:
            print("Deteniendo motores (control)...")
            try:
                # Intentar detener motores de forma segura
                send_command(fd, f'motor.stop(port.{MOTOR_PORT_B})\r')
                time.sleep(0.05)
                send_command(fd, f'motor.stop(port.{MOTOR_PORT_D})\r')
            except Exception as e_stop:
                print(f"Error al intentar detener motores (control): {e_stop}")
            finally:
                fd.close()
                print("Puerto serial cerrado.")
        # El objeto controller de evdev no suele requerir cierre explícito
        print("Limpieza de control de motor finalizada.")

# Permite ejecutar el script directamente también
if __name__ == '__main__':
    main_loop()
