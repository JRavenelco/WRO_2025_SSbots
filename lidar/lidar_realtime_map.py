#!/usr/bin/env python3
# filepath: /home/pi/lidar/lidar_realtime_map_qt.py
import sys
import time
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg
from rplidar import RPLidar

# Configuration
PORT_NAME = '/dev/ttyUSB0'
DMAX = 4000  # Maximum distance to display (mm)

class LidarThread(QThread):
    """Thread for handling LIDAR data acquisition"""
    data_signal = pyqtSignal(list, list)
    
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True
        
    def run(self):
        # Aumentar timeout a 5 segundos y quitar connect() explícito
        lidar = RPLidar(self.port, baudrate=115200, timeout=5) 
        try:
            # print("Connecting to LIDAR...") # connect() no es necesario aquí
            print("Starting motor...")
            lidar.start_motor()
            time.sleep(2)  # Espera a que el motor alcance velocidad
            
            try:
                # Intentar obtener info/health ANTES del bucle principal
                info = lidar.get_info()
                print(f"LIDAR Info: {info}")
                health = lidar.get_health()
                print(f"LIDAR Health: {health}")
                if health[0] != 'Good':
                     # Si la salud no es buena, no tiene sentido continuar
                     raise Exception(f"LIDAR health status is not Good: {health}")
            except Exception as e:
                 print(f"Could not get LIDAR info/health before scan: {e}")
                 # Si falla aquí, es probable que el escaneo también falle, así que salimos
                 raise e # Relanzar la excepción para que se maneje en el finally

            print("Starting scan...")
            for scan in lidar.iter_scans(): 
                if not self.running:
                    break
                    
                x = []
                y = []
                
                # Convertir ángulo y distancia a coordenadas cartesianas
                for _, angle, distance in scan: 
                    if distance > 0:
                        rad = np.deg2rad(angle)
                        x.append(distance * np.cos(rad))
                        y.append(distance * np.sin(rad))
                
                # Emitir los datos (solo si hay puntos)
                if x:
                    self.data_signal.emit(x, y)
                
        except Exception as e:
            # Imprimir el error específico que ocurrió
            print(f"Error in LIDAR thread during operation: {e}") 
        finally:
            print("Stopping LIDAR...")
            # Asegurarse de que el objeto lidar existe antes de intentar usarlo
            if 'lidar' in locals() and lidar.motor_running:
                 lidar.stop()
                 lidar.stop_motor()
                 lidar.disconnect()
            print("LIDAR stopped and disconnected")
    
    def stop(self):
        self.running = False
        self.wait()

class LidarVisualization(QMainWindow):
    """Main window for LIDAR visualization"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LIDAR Scan Visualization")
        self.resize(800, 800)
        
        # Central widget setup
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # PyQtGraph setup
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setXRange(-DMAX, DMAX)
        self.plot_widget.setYRange(-DMAX, DMAX)
        self.plot_widget.showGrid(True, True)
        self.plot_widget.setLabel('left', "Y (mm)")
        self.plot_widget.setLabel('bottom', "X (mm)")
        self.plot_widget.addLegend()
        
        layout.addWidget(self.plot_widget)
        
        # Create scatter plot item
        self.scatter = pg.ScatterPlotItem(
            size=5, 
            pen=pg.mkPen(None), 
            brush=pg.mkBrush(30, 144, 255, 150),
            name="LIDAR Points"
        )
        self.plot_widget.addItem(self.scatter)
        
        # Draw a small red dot at origin (0,0) - the LIDAR position
        origin = pg.ScatterPlotItem(
            size=10,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(255, 0, 0, 200),
            name="LIDAR"
        )
        origin.addPoints([0], [0])
        self.plot_widget.addItem(origin)
        
        # Create and start the LIDAR thread
        self.lidar_thread = LidarThread(PORT_NAME)
        self.lidar_thread.data_signal.connect(self.update_plot)
        self.lidar_thread.start()
    
    def update_plot(self, x, y):
        """Update the scatter plot with new data points"""
        if x and y:
            # Convertir listas a arrays NumPy
            x_np = np.array(x)
            y_np = np.array(y)
            # Pasar los arrays directamente a setData
            self.scatter.setData(x=x_np, y=y_np) 
            self.setWindowTitle(f"LIDAR Scan Visualization - {len(x)} points")
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.lidar_thread.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = LidarVisualization()
    window.show()
    sys.exit(app.exec_())