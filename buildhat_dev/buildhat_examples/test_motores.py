import time
from buildhat import Motor

def test_motor_port(port_id):
    """Tests a motor on a given port."""
    try:
        print(f"\n--- Probando motor en el Puerto {port_id} ---")
        motor = Motor(port_id)
        
        print("Girando hacia adelante por 2 segundos...")
        motor.start(100)
        time.sleep(2)
        
        print("Girando en reversa por 2 segundos...")
        motor.start(-100)
        time.sleep(2)
        
        motor.stop()
        print(f"Prueba para el Puerto {port_id} completada.")
        return True
        
    except Exception as e:
        print(f"Error probando el Puerto {port_id}: {e}")
        print("Por favor, asegúrate de que un motor esté conectado correctamente a este puerto.")
        return False

if __name__ == "__main__":
    print("Script de Prueba de Motores para Build HAT")
    print("Conecta los motores a los puertos A, B, C, o D.")
    
    while True:
        port = input("Ingresa el puerto a probar (A, B, C, o D), o 'q' para salir: ").upper()
        
        if port in ['A', 'B', 'C', 'D']:
            test_motor_port(port)
        elif port == 'Q':
            print("Saliendo del script de prueba.")
            break
        else:
            print("Entrada inválida. Por favor, ingresa A, B, C, D, o q.")
