import depthai as dai
import cv2
import os
import time

class OakCamera:
    def __init__(self, save_folder):
        self.save_folder = save_folder
        os.makedirs(self.save_folder, exist_ok=True)
        self.pipeline = dai.Pipeline()
        self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.cam_rgb.setFps(30) # FPS para la vista previa

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(xout_rgb.input)

    def start_manual_capture(self):
        with dai.Device(self.pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            print("\n--- Captura Manual ---")
            print("En la ventana de la cámara:")
            print("  - Presiona 's' para guardar una imagen.")
            print("  - Presiona 'q' para salir.")
            print("----------------------")

            frame = None
            while True:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    # Muestra el frame en una ventana
                    cv2.imshow("Captura Manual", frame)

                # Espera por una tecla (1ms de retraso)
                key = cv2.waitKey(1) & 0xFF

                # Si se presiona 's', guarda el frame actual
                if key == ord('s'):
                    if frame is not None:
                        filename = os.path.join(self.save_folder, f"image_{time.time_ns()}.jpg")
                        cv2.imwrite(filename, frame)
                        print(f"Imagen guardada: {filename}")
                    else:
                        print("Aún no hay un frame disponible. Espera un momento.")

                # Si se presiona 'q', rompe el bucle
                elif key == ord('q'):
                    break
        
        # Cierra todas las ventanas de OpenCV
        cv2.destroyAllWindows()
        print("\nPrograma finalizado.")