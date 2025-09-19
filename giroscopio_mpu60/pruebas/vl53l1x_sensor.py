#!/usr/bin/env python3
# Script optimizado para sensor VL53L1X/V2

import time
import board
import busio
import adafruit_vl53l1x
from collections import deque
import statistics

class VL53L1X_Sensor:
    def __init__(self, i2c):
        """Inicializa el sensor VL53L1X con configuración mejorada."""
        self.sensor = adafruit_vl53l1x.VL53L1X(i2c)
        
        # Configuración del sensor
        self.sensor.distance_mode = 2  # 2 = Largo alcance (hasta 4m)
        self.sensor.timing_budget = 200  # ms para mejor precisión
        self.sensor.start_ranging()
        
        # Filtrado
        self.history = deque(maxlen=5)  # Historial para mediana
        self.min_distance = 50    # mm
        self.max_distance = 2000  # mm
        
        # Calentamiento del sensor
        self._warm_up()
    
    def _warm_up(self):
        """Realiza lecturas iniciales para estabilizar el sensor."""
        print("Calentando el sensor...")
        for _ in range(10):
            try:
                self.sensor.distance
                time.sleep(0.1)
            except:
                pass
    
    def get_filtered_distance(self):
        """Obtiene una distancia filtrada y validada."""
        try:
            # Leer distancia
            distance = self.sensor.distance
            
            # Validar rango
            if distance is None or distance < self.min_distance or distance > self.max_distance:
                return -1
                
            # Agregar al historial
            self.history.append(distance)
            
            # Si no hay suficientes lecturas, devolver -1
            if len(self.history) < 3:
                return -1
                
            # Calcular mediana para filtrar ruido
            return int(statistics.median(self.history))
            
        except Exception as e:
            print(f"Error en lectura: {e}")
            return -1

def main():
    print("=== Iniciando sensor VL53L1X/V2 ===")
    
    try:
        # Inicializar I2C
        print("1. Inicializando bus I2C...")
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Inicializar sensor
        print("2. Inicializando sensor...")
        sensor = VL53L1X_Sensor(i2c)
        print("   ✓ Sensor inicializado correctamente")
        print(f"   Modo de distancia: {'Largo alcance (hasta 4m)' if sensor.sensor.distance_mode == 2 else 'Corto alcance'}")
        print(f"   Rango válido: {sensor.min_distance}-{sensor.max_distance} mm")
        
        # Bucle principal
        print("\n=== Iniciando lecturas (Ctrl+C para detener) ===")
        while True:
            distance = sensor.get_filtered_distance()
            if distance > 0:
                print(f"Distancia: {distance:4d} mm   ", end='\r')
            else:
                print("Distancia:    -1 mm   ", end='\r')
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nLectura detenida por el usuario")
    except Exception as e:
        print(f"\nError crítico: {e}")
    finally:
        if 'sensor' in locals():
            sensor.sensor.stop_ranging()
        print("Sensor detenido correctamente")

if __name__ == "__main__":
    main()
