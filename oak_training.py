import argparse
import os
import cv2
import depthai as dai
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense


def capture_data(num_images=100, save_folder='dataset'):
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
    # Crear pipeline de DepthAI
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName('rgb')
    cam_rgb.preview.link(xout_rgb.input)

    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name='rgb', maxSize=4, blocking=False)
        count = 0
        print('Capturando imágenes. Presiona "q" para terminar.')
        while count < num_images:
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                filename = os.path.join(save_folder, f'image_{count:04d}.jpg')
                cv2.imwrite(filename, frame)
                print(f'Guardada: {filename}')
                count += 1
                cv2.imshow('Capturando', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        cv2.destroyAllWindows()


def load_dataset(dataset_folder='dataset', img_size=(64, 64)):
    images = []
    labels = []
    # Para este ejemplo, asumimos que todas las imágenes pertenecen a la misma clase (etiqueta 0)
    for filename in os.listdir(dataset_folder):
        if filename.endswith('.jpg'):
            img_path = os.path.join(dataset_folder, filename)
            img = cv2.imread(img_path)
            if img is not None:
                img = cv2.resize(img, img_size)
                images.append(img)
                labels.append(0)  # Etiqueta dummy
    images = np.array(images, dtype='float32') / 255.0
    labels = np.array(labels)
    return images, labels


def extract_dataset(zip_path='/home/jetson/WRO_2025/docs/WRO Round 2.v3-red_green_xpark_320.yolov8.zip', extract_to='extracted_dataset'):
    import zipfile
    if not os.path.exists(extract_to):
        os.makedirs(extract_to)
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to)
    print('Dataset extraído a', extract_to)
    return extract_to


def build_model(input_shape):
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=input_shape),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(64, activation='relu'),
        Dense(1, activation='sigmoid')
    ])
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
    return model


def train_model(dataset_folder=None, epochs=5):
    img_size = (64, 64)
    if dataset_folder is None:
        dataset_folder = 'dataset'
        if not os.path.exists(dataset_folder):
            dataset_folder = extract_dataset()
    images, labels = load_dataset(dataset_folder, img_size=img_size)
    if len(images) == 0:
        print('No se encontraron imágenes en el dataset. Captura datos primero o revisa el archivo de dataset.')
        return
    input_shape = (img_size[0], img_size[1], 3)
    model = build_model(input_shape)
    print('Entrenando el modelo...')
    model.fit(images, labels, epochs=epochs, batch_size=8)
    model.save('oak_model.h5')
    print('Modelo entrenado y guardado como oak_model.h5')


def main():
    parser = argparse.ArgumentParser(description='Algoritmo de entrenamiento para la cámara OAK')
    parser.add_argument('--capture', action='store_true', help='Capturar datos de entrenamiento')
    parser.add_argument('--train', action='store_true', help='Entrenar el modelo usando datos capturados')
    parser.add_argument('--num_images', type=int, default=100, help='Número de imágenes a capturar')
    parser.add_argument('--epochs', type=int, default=5, help='Número de épocas de entrenamiento')
    args = parser.parse_args()
    
    if args.capture:
        capture_data(num_images=args.num_images)
    elif args.train:
        train_model(epochs=args.epochs)
    else:
        print('Usa --capture para capturar datos o --train para entrenar el modelo.')


if __name__ == '__main__':
    main()
