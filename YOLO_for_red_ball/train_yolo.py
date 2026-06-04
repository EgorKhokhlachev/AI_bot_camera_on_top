"""Скрипт обучения YOLOv8 для красного шара."""
import argparse
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, help='data.yaml')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--output', default='ball_detector.pt')
    args = parser.parse_args()

    model = YOLO('yolov8n.pt')
    model.train(data=args.data, epochs=args.epochs, batch=args.batch,
                imgsz=640, name='red_ball_train')
    import shutil
    shutil.copy(f'runs/detect/red_ball_train/weights/best.pt', args.output)
    print(f'Model saved to {args.output}')

if __name__ == '__main__':
    main()