import serial
import time  # Make sure this is present
import sys
import tty
import termios
import pexpect # <<< ADD THIS LINE

def prepare_serial_port_with_keys(port="/dev/ttyACM0", baudrate=115200):
    """
    Attempts to clear a screen session on the specified port by sending Ctrl+A, K, Y.
    """
    screen_command = f"screen \"{port}\" {baudrate}"
    print(f"Attempting to prepare serial port: {screen_command}")
    try:
        child = pexpect.spawn(screen_command, timeout=5)
        # Wait a moment for screen to initialize or attach
        time.sleep(0.5)
        # Check if screen actually started or if it exited immediately
        if not child.isalive():
            print(f"Screen session for {port} did not start or exited quickly. Assuming port is clear or screen failed.")
            return
        print("Sending Ctrl+A (screen command prefix)")
        child.sendcontrol('a') # Sends Ctrl+A
        time.sleep(0.2) 
        print("Sending 'k' (kill window command)")
        child.send('k')
        time.sleep(0.2)
        # Expect the "Kill window" prompt.
        # Using a regex for flexibility with the prompt's exact wording/casing.
        index = child.expect([r"[Kk]ill.*\(y/n\)", pexpect.TIMEOUT, pexpect.EOF], timeout=2)
        if index == 0: # Prompt found
            print("Sending 'y' (confirm kill)")
            child.sendline('y') # Using sendline for 'y' just in case it expects a newline
            time.sleep(0.5) 
            print(f"Screen session on {port} should be terminated if it was active.")
        elif index == 1:
            print("Timeout waiting for kill prompt. Screen session might not have been active or prompt differs.")
        elif index == 2:
            print("Screen session ended (EOF) before kill prompt. It might not have been active.")
        if child.isalive():
            child.close(force=True)
            print("Closed pexpect child process.")
    except pexpect.exceptions.ExceptionPexpect as e:
        print(f"Pexpect error while trying to manage screen session: {e}")
    except Exception as e:
        print(f"An unexpected error occurred while preparing serial port with screen: {e}")
    finally:
        print(f"Finished attempt to prepare serial port {port}.")
    time.sleep(1) # Crucial delay to allow the port to free up

class SpikeMotorControl:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1):
        """Inicializa la conexión con el Spike Prime"""
        try:
            self.spike = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Esperar a que se establezca la conexión
            self.initialize_motors()
            print("Conexión establecida con Spike Prime")
        except Exception as e:
            print(f"Error al conectar con Spike Prime: {e}")
            sys.exit(1)

    def send_command(self, command):
        """Envía un comando al Spike Prime y limpia el buffer"""
        try:
            self.spike.write(f"{command}\r".encode())
            time.sleep(0.1)  # Pequeña pausa para evitar sobrecarga
            return self.spike.readline().decode().strip()
        except Exception as e:
            print(f"Error al enviar comando: {e}")
            return ""

    def initialize_motors(self):
        """Inicializa los motores y funciones en el Spike Prime"""
        # Detener cualquier movimiento previo
        self.send_command("import motor")
        self.send_command("from hub import port")
        
        # Función para detener motores
        self.send_command("def stop_motors():")
        self.send_command("    motor.stop(port.B, stop=motor.COAST)")
        self.send_command("    motor.stop(port.F, stop=motor.COAST)")
        
        # Función para mover hacia adelante/atrás
        self.send_command("def move(speed, duration=0.5):")
        self.send_command("    motor.run(port.B, speed)")
        self.send_command("    time.sleep(duration)")
        self.send_command("    stop_motors()")
        
        # Función para girar (Ackerman)
        self.send_command("def turn(angle, speed=30, duration=0.5):")
        self.send_command("    motor.run_to_relative_position(port.F, angle, speed)")
        self.send_command("    time.sleep(duration)")
        
        # Importar time en el Spike Prime
        self.send_command("import time")
        print("Motores inicializados")

    def move_forward(self, speed=50, duration=0.5):
        """Mueve el robot hacia adelante"""
        print(f"Avanzando a velocidad {speed}% por {duration} segundos")
        self.send_command(f"move({speed}, {duration})")

    def move_backward(self, speed=50, duration=0.5):
        """Mueve el robot hacia atrás"""
        print(f"Retrocediendo a velocidad {speed}% por {duration} segundos")
        self.send_command(f"move({-speed}, {duration})")

    def turn_left(self, angle=30, speed=30, duration=0.5):
        """Gira las ruedas a la izquierda"""
        print(f"Girando a la izquierda {angle} grados")
        self.send_command(f"turn({-abs(angle)}, {speed}, {duration})")

    def turn_right(self, angle=30, speed=30, duration=0.5):
        """Gira las ruedas a la derecha"""
        print(f"Girando a la derecha {angle} grados")
        self.send_command(f"turn({abs(angle)}, {speed}, {duration})")

    def stop(self):
        """Detiene todos los motores"""
        print("Deteniendo motores")
        self.send_command("stop_motors()")

    def close(self):
        """Cierra la conexión serial"""
        self.stop()
        self.spike.close()
        print("Conexión cerrada")

def get_key():
    """Obtiene una tecla presionada sin necesidad de presionar Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    """Función principal para control manual"""
    prepare_serial_port_with_keys()  # <<< ADD THIS LINE AT THE VERY BEGINNING   
    try:
        # Inicializar controlador de motores
        controller = SpikeMotorControl()         
        print("""
        Controles:
        [Flecha Arriba] - Avanzar
        [Flecha Abajo]  - Retroceder
        [Flecha Izq]    - Girar izquierda
        [Flecha Der]    - Girar derecha
        [Espacio]       - Detener
        [q]             - Salir
        """)   
        while True:
            key = get_key()
            
            if key == '\x1b':  # Tecla de escape para flechas
                key += get_key() + get_key()
                if key == '\x1b[A':  # Flecha arriba
                    controller.move_forward()
                elif key == '\x1b[B':  # Flecha abajo
                    controller.move_backward()
                elif key == '\x1b[D':  # Flecha izquierda
                    controller.turn_left()
                elif key == '\x1b[C':  # Flecha derecha
                    controller.turn_right()
            elif key == ' ':  # Espacio
                controller.stop()
            elif key.lower() == 'q':  # Salir
                print("Saliendo...")
                break
                
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main()
