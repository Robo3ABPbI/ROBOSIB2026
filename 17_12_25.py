import cv2
import numpy as np

import cv2
import numpy as np

# Диапазоны цветов в HSV
yellow_LOWER_COLOR = np.array([16, 35, 189])
yellow_UPPER_COLOR = np.array([85, 255, 255])

purple_LOWER_COLOR = np.array([63, 40, 54])
purple_UPPER_COLOR = np.array([159, 121, 168])

green_LOWER_COLOR = np.array([28, 76, 109])
green_UPPER_COLOR = np.array([99, 255, 255])

red_LOWER_COLOR = np.array([0, 100, 100])
red_UPPER_COLOR = np.array([10, 255, 255])

blue_LOWER_COLOR = np.array([90, 50, 50])
blue_UPPER_COLOR = np.array([130, 255, 255])

# Захват видео с камеры
vid = cv2.VideoCapture(0)

# Список для хранения информации об объектах
objects_info = []

while True:
    ret, frame = vid.read()
    if not ret:
        break
    
    # Преобразование в HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Маски для каждого цвета
    mask_yellow = cv2.inRange(hsv, yellow_LOWER_COLOR, yellow_UPPER_COLOR)
    mask_purple = cv2.inRange(hsv, purple_LOWER_COLOR, purple_UPPER_COLOR)
    mask_green = cv2.inRange(hsv, green_LOWER_COLOR, green_UPPER_COLOR)
    mask_red = cv2.inRange(hsv, red_LOWER_COLOR, red_UPPER_COLOR)
    mask_blue = cv2.inRange(hsv, blue_LOWER_COLOR, blue_UPPER_COLOR)

    # Обработка масок
    masks = [mask_blue, mask_red, mask_green]
    colors = [(255, 0, 0), (0, 0, 255), (0, 255, 0)]

    for mask, color in zip(masks, colors):
        # Обнаружение контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Аппроксимируем контур
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Обводим контур любой формы
            x, y, w, h = cv2.boundingRect(contour)
            if color == (0, 255, 0):  # Зелёные объекты
                area = cv2.contourArea(contour)
                if area > 180:  # Большой объект
                    cv2.drawContours(frame, [approx], 0, (0, 128, 0), 2)
                    center_x, center_y = x + w // 2, y + h // 2
                    cv2.circle(frame, (center_x, center_y), 5, (0, 128, 0), -1)
                    objects_info.append([(0, 128, 0), (center_x, center_y)])
                elif area > 80:  # Маленький объект
                    cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                    center_x, center_y = x + w // 2, y + h // 2
                    cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    objects_info.append([(0, 255, 0), (center_x, center_y)])
            else:  # Красные и синие объекты
                cv2.drawContours(frame, [approx], 0, color, 2)
                center_x, center_y = x + w // 2, y + h // 2
                cv2.circle(frame, (center_x, center_y), 5, color, -1)
                objects_info.append([color, (center_x, center_y)])

    # Отображение кадров
    cv2.imshow("robozari 2026 v0.3", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print('...')

cv2.destroyAllWindows()
