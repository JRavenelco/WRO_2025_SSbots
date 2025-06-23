import argparse
from camera import OakCamera

def main():
    parser = argparse.ArgumentParser(description="OAK Camera Manual Image Capture")
    parser.add_argument('--save_folder', type=str, required=True, help="Carpeta para guardar las imágenes.")
    args = parser.parse_args()

    print(f"Las imágenes se guardarán en: {args.save_folder}")
    
    camera = OakCamera(args.save_folder)
    camera.start_manual_capture()

if __name__ == "__main__":
    main()