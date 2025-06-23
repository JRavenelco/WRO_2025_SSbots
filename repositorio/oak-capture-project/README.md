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

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(xout_rgb.input)

    def start_capture(self, rate, duration):
        self.cam_rgb.setFps(rate)

        with dai.Device(self.pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=30, blocking=False)
            frame_count = 0
            start_time = time.time()
            print(f"Iniciando captura por {duration} segundos a {rate} FPS...")

            while time.time() - start_time < duration:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    filename = os.path.join(self.save_folder, f"image_{time.time_ns()}.jpg")
                    cv2.imwrite(filename, frame)
                    frame_count += 1
            
            end_time = time.time()
            elapsed_time = end_time - start_time
            actual_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            print(f"\nCaptura finalizada.")
            print(f"Se guardaron {frame_count} imágenes en {elapsed_time:.2f} segundos.")
            print(f"FPS promedio real: {actual_fps:.2f}")