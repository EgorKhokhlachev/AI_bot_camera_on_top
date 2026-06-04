import cv2
from ultralytics import YOLO

# Загружаем модель
model = YOLO('ball_detector.pt')

# Указываем путь к тестовому изображению
image_path = 'test_image.jpg'

# Выполняем предсказание
results = model(image_path)

# Визуализируем результат
annotated = results[0].plot()

# Показываем в окне
cv2.imshow('Red Ball Detection', annotated)
cv2.waitKey(0)  # нажмите любую клавишу для закрытия
cv2.destroyAllWindows()

# Сохраняем результат
cv2.imwrite('result.jpg', annotated)