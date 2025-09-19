#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import traceback

if __name__ == '__main__':
    # --- CONFIGURACION ---
    # Reemplaza estos valores con los tuyos:
    MODEL_ID = "wro-round-2-mefmf"
    MODEL_VERSION = "3"
    
    # ¡¡¡IMPORTANTE!!! Usa tu CLAVE API PRIVADA aqui
    PRIVATE_API_KEY = "1QaioZ6lSVket4h0AjJP"
    
    CONFIDENCE_THRESHOLD = 0.5
    OVERLAP_THRESHOLD = 0.5
    HAS_DEPTH = True 
    # --- FIN DE LA CONFIGURACION ---

    if PRIVATE_API_KEY == "TU_CLAVE_API_PRIVADA_AQUI":
        print("ERROR: Por favor, reemplaza la clave API por la tuya. El programa se detendra.")
        exit()

    print(f"Inicializando RoboflowOak con:")
    print(f"  Modelo ID: {MODEL_ID}")
    print(f"  Version del Modelo: {MODEL_VERSION}")
    print(f"  Usando profundidad: {HAS_DEPTH}")
    print(f"  Umbral de Confianza: {CONFIDENCE_THRESHOLD}")
    print(f"  Umbral de Superposicion: {OVERLAP_THRESHOLD}")

    rf = None
    try:
        rf = RoboflowOak(
            model=MODEL_ID,
            confidence=CONFIDENCE_THRESHOLD,
            overlap=OVERLAP_THRESHOLD,
            version=MODEL_VERSION,
            api_key=PRIVATE_API_KEY,
            rgb=True,
            depth=HAS_DEPTH,
            device=None,
            blocking=True
        )
        print("RoboflowOak inicializado correctamente. Iniciando bucle de deteccion...")

        gui_enabled = True
        try:
            cv2.imshow("Prueba de GUI", np.zeros((1, 1), dtype=np.uint8))
            cv2.destroyAllWindows()
            print("Soporte para GUI de OpenCV detectado. Las ventanas de visualizacion se mostraran.")
        except cv2.error:
            gui_enabled = False
            print("ERROR: Soporte de GUI de OpenCV no detectado. Las ventanas de visualizacion no se mostraran.")
            print("Para habilitar la GUI, asegurate de tener las dependencias de GTK instaladas.")

        while True:
            t_start = time.time()
            
            # Si no necesitas el mapa de profundidad, ignoralo con `_`
            result, frame_with_detections, _, depth_map = rf.detect()
            
            predictions = result.get("predictions", [])
            
            if gui_enabled:
                # Calcular y mostrar FPS en el frame
                t_end = time.time()
                fps = 1 / (t_end - t_start) if (t_end - t_start) > 0 else 0
                cv2.putText(frame_with_detections, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Opcional: Agregar la distancia a las detecciones si el modo GUI esta activado
                if HAS_DEPTH and depth_map is not None:
                    if depth_map.ndim == 2:
                        for pred in predictions:
                            # CORREGIDO: Acceso a atributos con notacion de punto
                            x, y = int(pred.x), int(pred.y)
                            h_depth, w_depth = depth_map.shape
                            x_clamped = max(0, min(x, w_depth - 1))
                            y_clamped = max(0, min(y, h_depth - 1))
                            median_depth_mm = depth_map[y_clamped, x_clamped]
                            if median_depth_mm > 0:
                                distance_cm = f"{median_depth_mm / 10.0:.1f}cm"
                                cv2.putText(frame_with_detections, f"{pred.class_name}: {distance_cm}",
                                            (x, y - 15), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Mostrar el frame con las detecciones
                cv2.imshow("Roboflow OAK Detections", frame_with_detections)

                # Salir con la tecla 'q'
                if cv2.waitKey(1) == ord('q'):
                    print("Saliendo del bucle de deteccion...")
                    break
            else:
                # Si no hay GUI, imprimir las detecciones en la consola
                print("\033[H\033[J", end="") # Limpiar la consola
                if predictions:
                    print(f"Detecciones encontradas en la consola:")
                    for pred in predictions:
                        # CORREGIDO: Acceso a atributos con notacion de punto
                        class_name = pred.class_name
                        confidence = pred.confidence
                        x, y = pred.x, pred.y
                        
                        distance_cm = "N/A"
                        if HAS_DEPTH and depth_map is not None:
                            h_depth, w_depth = depth_map.shape
                            x_clamped = max(0, min(int(x), w_depth - 1))
                            y_clamped = max(0, min(int(y), h_depth - 1))
                            median_depth_mm = depth_map[y_clamped, x_clamped]
                            if median_depth_mm > 0:
                                distance_cm = f"{median_depth_mm / 10.0:.1f}cm"
                        
                        print(f"  - Clase: {class_name}, Confianza: {confidence:.2f}, Posicion (X, Y): ({x}, {y}), Distancia: {distance_cm}")
                else:
                    print("No se encontraron detecciones.")
                
                # Pausa para evitar un uso excesivo de la CPU en modo headless
                time.sleep(0.5)

    except Exception as e:
        print(f"Ocurrio un error inesperado durante la ejecucion: {e}")
        traceback.print_exc()

    finally:
        print("Cerrando recursos y terminando.")
        # CORREGIDO: Eliminacion de la llamada a rf.close()
        if gui_enabled:
            cv2.destroyAllWindows()