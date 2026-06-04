from ultralytics import YOLO

class BallDetector:
    """Детектор красного шара с использованием YOLO."""

    def __init__(self, model_path='/home/egor/camera_on_top_ws/src/maze_solver/config/ball_detector.pt'):
        self.yolo = YOLO(model_path)

    def detect_red_ball(self, image):
        """
        Возвращает мировые координаты (x, y) центра красного шара или None.
        image – numpy BGR-изображение.
        """
        results = self.yolo(image, verbose=False)
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                class_name = r.names[cls_id]
                if class_name.lower() == '0':
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    return cx, cy  # пиксельные координаты
        return None