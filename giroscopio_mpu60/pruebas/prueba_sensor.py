#!/usr/bin/env python3
# Script de depuración para sensor VL53L0X

import time
import statistics
from collections import deque
import board
import busio
import adafruit_vl53l0x

def main():
    print("=== Iniciando depuración del sensor VL53L0X ===")
    
    try:
        # 1. Inicializar I2C con reintentos
        print("1. Inicializando bus I2C...")
        i2c = None
        for _ in range(3):  # Reintentar 3 veces
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                break
            except Exception as e:
                print(f"   Error al inicializar I2C: {e}")
                time.sleep(1)
        
        if i2c is None:
            raise RuntimeError("No se pudo inicializar el bus I2C")
        
        # 2. Verificar conexión I2C
        print("2. Buscando dispositivos I2C...")
        try:
            i2c.try_lock()
            devices = i2c.scan()
            i2c.unlock()
            print(f"   Dispositivos I2C encontrados: {[hex(x) for x in devices]}")
            
            if not devices:
                raise RuntimeError("No se detectaron dispositivos I2C")
                
            if 0x29 not in devices:
                raise RuntimeError("No se encontró el sensor VL53L0X (0x29) en el bus I2C")
                
        except Exception as e:
            i2c.unlock()
            raise RuntimeError(f"Error al escanear dispositivos I2C: {e}")
        
        # 3. Inicializar sensor
        print("3. Inicializando sensor VL53L0X...")
        try:
            sensor = adafruit_vl53l0x.VL53L0X(i2c)
            print("   ✓ Sensor inicializado correctamente")
        except Exception as e:
            raise RuntimeError(f"Error al inicializar el sensor: {e}")
        
        # 4. Configurar sensor para mayor alcance
        print("4. Configurando sensor para mayor alcance...")
        try:
            # Aumentar el presupuesto de tiempo para mayor precisión en distancias largas
            sensor.measurement_timing_budget = 500000  # 500ms para mayor alcance
            
            # Configurar para alta precisión en distancias largas
            # Nota: La biblioteca de Adafruit no expone directamente estas configuraciones,
            # pero podemos ajustar el timing budget que ayuda con el rango
            
            # Iniciar mediciones continuas
            sensor.start_continuous()
            print("   Configuración para largo alcance completada")
            print("   Nota: El rango máximo teórico es ~2m, pero puede variar")
            print("   según las condiciones de iluminación y superficie")
            
        except Exception as e:
            print(f"   Advertencia: No se pudo configurar para largo alcance: {e}")
            print("   Usando configuración por defecto...")
            sensor.start_continuous()
        
        # 5. Bucle de lectura filtrado
        print("\n=== Iniciando lecturas (Ctrl+C para detener) ===")
        
        # Parámetros de rango
        MIN_DISTANCE = 50     # mm
        MAX_DISTANCE = 2000   # mm
        
        # Historial para filtrado (mediana)
        history = deque(maxlen=5)
        
        while True:
            try:
                # Leer distancia cruda
                raw_distance = sensor.range

                # Agregar a historial si está en rango
                if MIN_DISTANCE <= raw_distance <= MAX_DISTANCE:
                    history.append(raw_distance)
                
                # Mostrar resultado
                if len(history) >= 3:
                    filtered = int(statistics.median(history))
                    print(f"Distancia: {filtered:4d} mm   ", end='\r')
                else:
                    # Historial insuficiente o valor fuera de rango
                    print("Distancia:    -1 mm   ", end='\r')
                
                time.sleep(0.1)
            except Exception as e:
                print(f"\nError en lectura: {e}")
                print("Distancia:    -1 mm   ", end='\r')
                time.sleep(0.2)
                
    except Exception as e:
        print(f"\n=== ERROR CRÍTICO ===\n{str(e)}\n")
        print("Posibles soluciones:")
        print("1. Verifica que el sensor esté correctamente conectado")
        print("2. Asegúrate de que I2C está habilitado (sudo raspi-config)")
        print("3. Prueba ejecutando con: sudo python3 prueba_sensor.py")
        print("4. Verifica que no hay conflictos con otros dispositivos I2C")
        return 1
    
    finally:
        # Asegurarse de liberar el bloqueo I2C
        if 'i2c' in locals():
            try:
                if hasattr(i2c, "locked") and i2c.locked:
                    i2c.unlock()
                if hasattr(i2c, "deinit"):
                    i2c.deinit()  # Liberar recursos I2C
            except Exception as e:
                print(f"Advertencia al limpiar I2C: {e}")
        print("\n=== Depuración finalizada ===")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrueba interrumpida por el usuario")
    finally:
        print("Sistema finalizado correctamente")