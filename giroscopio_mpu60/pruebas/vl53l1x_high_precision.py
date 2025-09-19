#!/usr/bin/env python3
# Script de alta precisión para VL53L1X/V2 con filtro de Kalman

import time
import board
import busio
import adafruit_vl53l1x
import numpy as np
from collections import deque
from scipy.signal import savgol_filter

class KalmanFilter:
    """Implementación de un filtro de Kalman simple para mediciones de distancia."""
    def __init__(self, process_variance=1e-5, measurement_variance=0.1**2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_distance = 0.0
        self.estimation_error = 1.0
        
    def update(self, measurement):
        # Predicción
        prediction = self.estimated_distance
        prediction_error = self.estimation_error + self.process_variance
        
        # Actualización
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimated_distance = prediction + kalman_gain * (measurement - prediction)
        self.estimation_error = (1 - kalman_gain) * prediction_error
        
        return self.estimated_distance

class VL53L1X_HighPrecision:
    def __init__(self, i2c):
        """Inicializa el sensor con configuración de alta precisión."""
        self.sensor = adafruit_vl53l1x.VL53L1X(i2c)
        
        # Configuración del sensor para máxima precisión
        self.sensor.distance_mode = 1  # 1 = Corto alcance (mayor precisión)
        self.sensor.timing_budget = 200  # ms (mayor tiempo = mayor precisión)
        self.sensor.start_ranging()
        
        # Filtros
        self.kalman = KalmanFilter()
        self.median_history = deque(maxlen=5)  # Para filtro de mediana
        self.savgol_window = 7  # Tamaño de ventana para Savitzky-Golay
        self.savgol_order = 2   # Orden del polinomio
        self.raw_history = deque(maxlen=20)  # Historial para filtro Savitzky-Golay
        
        # Rango válido
        self.min_distance = 50    # mm
        self.max_distance = 2000  # mm
        
        # Calentamiento
        self._warm_up()
    
    def _warm_up(self):
        """Realiza lecturas iniciales para estabilizar el sensor."""
        print("Calentando el sensor (esto puede tomar unos segundos)...")
        warmup_readings = []
        
        for _ in range(20):
            try:
                reading = self.sensor.distance
                if reading and 0 < reading < 4000:  # Rango razonable
                    warmup_readings.append(reading)
                    time.sleep(0.1)
            except:
                pass
        
        # Inicializar filtro con la mediana de las lecturas de calentamiento
        if warmup_readings:
            initial_estimate = np.median(warmup_readings)
            self.kalman.estimated_distance = initial_estimate
            print(f"Sensor calentado. Lectura inicial: {initial_estimate:.1f} mm")
    
    def _is_valid_reading(self, distance):
        """Valida si una lectura está dentro de los parámetros aceptables."""
        if distance is None:
            return False
        return self.min_distance <= distance <= self.max_distance
    
    def _apply_filters(self, raw_distance):
        """Aplica todos los filtros a la lectura en bruto."""
        if not self._is_valid_reading(raw_distance):
            return -1
        
        # 1. Filtro de mediana (rápido, elimina valores atípicos)
        self.median_history.append(raw_distance)
        if len(self.median_history) < 3:  # Mínimo para mediana
            return -1
        median_filtered = np.median(self.median_history)
        
        # 2. Filtro de Kalman (estima el estado real)
        kalman_filtered = self.kalman.update(median_filtered)
        
        # 3. Filtro Savitzky-Golay (suavizado sin retraso)
        self.raw_history.append(kalman_filtered)
        if len(self.raw_history) >= self.savgol_window:
            try:
                window_size = min(len(self.raw_history), self.savgol_window)
                if window_size > self.savgol_order:
                    y = savgol_filter(
                        list(self.raw_history)[-window_size:],
                        window_size,
                        self.savgol_order
                    )
                    return max(0, int(y[-1]))  # No distancias negativas
            except:
                pass
        
        return max(0, int(kalman_filtered))  # Fallback a Kalman si no hay suficiente historial
    
    def get_high_precision_distance(self):
        """Obtiene una distancia con la máxima precisión posible."""
        try:
            raw_distance = self.sensor.distance
            if not self._is_valid_reading(raw_distance):
                return -1
                
            return self._apply_filters(raw_distance)
            
        except Exception as e:
            print(f"\nError en lectura: {e}")
            return -1

def main():
    print("=== VL53L1X/V2 - Modo Alta Precisión ===")
    print("Este modo prioriza la precisión sobre la velocidad de respuesta.")
    print("Puede haber un ligero retraso en las lecturas debido al procesamiento.")
    
    try:
        # Inicializar I2C
        print("\n1. Inicializando bus I2C...")
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Inicializar sensor
        print("2. Inicializando sensor VL53L1X/V2...")
        sensor = VL53L1X_HighPrecision(i2c)
        print("   ✓ Sensor inicializado correctamente")
        print(f"   Modo: {'Corto alcance' if sensor.sensor.distance_mode == 1 else 'Largo alcance'}")
        print(f"   Rango válido: {sensor.min_distance}-{sensor.max_distance} mm")
        print("\n=== Iniciando lecturas (Ctrl+C para detener) ===")
        
        # Bucle principal
        while True:
            distance = sensor.get_high_precision_distance()
            if distance > 0:
                print(f"Distancia: {distance:4d} mm   ", end='\r')
            else:
                print("Distancia:    -1 mm   ", end='\r')
            time.sleep(0.05)  # Pequeña pausa para no saturar el bus I2C
            
    except KeyboardInterrupt:
        print("\n\nLectura detenida por el usuario")
    except Exception as e:
        print(f"\n\nError crítico: {e}")
    finally:
        if 'sensor' in locals():
            sensor.sensor.stop_ranging()
        print("Sensor detenido correctamente")

if __name__ == "__main__":
    main()
