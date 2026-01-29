import cv2
import numpy as np

# Загрузите изображение
img = cv2.imread('img.png')  # Замените на путь к вашему изображению
if img is None:
    raise FileNotFoundError('Изображение не найдено.')

# Размеры для ресайза
d, v = 940, 480  # Можете изменить под свои нужды

# Ресайз изображения
img_resized = cv2.resize(img, (d, v))

# Конвертация в HSV
hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

# Цветовые диапазоны для масок
# Красный
lower_red1 = np.array([0, 100, 50])
upper_red1 = np.array([15, 255, 255])
lower_red2 = np.array([160, 100, 50])
upper_red2 = np.array([180, 255, 255])

# Зеленый
lower_green1 = np.array([40, 70, 70])
upper_green1 = np.array([80, 255, 255])
lower_green2 = np.array([30, 70, 40])
upper_green2 = np.array([85, 255, 255])

# Создание масок
mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)

mask_green1 = cv2.inRange(hsv, lower_green1, upper_green1)
mask_green2 = cv2.inRange(hsv, lower_green2, upper_green2)
mask_green = cv2.bitwise_or(mask_green1, mask_green2)

# Морфологические операции для очистки масок
kernel = np.ones((3, 3), np.uint8)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

# Поиск контуров
contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Флаг обнаружения
triangle_found = False

# Обработка красных фигур
for contour in contours_red:
    area = cv2.contourArea(contour)
    if 300 < area < 10000:
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 3 and cv2.isContourConvex(approx):
            triangle_found = True
            # Обводка и подпись
            cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
            for point in approx:
                cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 255), -1)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.putText(img_resized, "Red Triangle", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

# Обработка зеленых фигур
for contour in contours_green:
    area = cv2.contourArea(contour)
    if 300 < area < 10000:
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 3 and cv2.isContourConvex(approx):
            triangle_found = True
            # Обводка и подпись
            cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
            for point in approx:
                cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 255), -1)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.putText(img_resized, "Green Triangle", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / h if h > 0 else 0
            if 0.5 < aspect_ratio < 2.0:
                triangle_found = True
                cv2.drawContours(img_resized, [approx], -1, (0, 255, 255), 2)
                cv2.putText(img_resized, "Green Rectangle", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# Обработка контуров с 3-4 вершинами (возможно, квадраты/прямоугольники)
for contour in contours_red:
    area = cv2.contourArea(contour)
    if 500 < area < 15000:
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / h if h > 0 else 0
            if 0.5 < aspect_ratio < 2.0:
                triangle_found = True
                cv2.drawContours(img_resized, [approx], -1, (0, 255, 255), 2)
                cv2.putText(img_resized, "Red Rectangle", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

# Вывод статуса
status_texts = []
if triangle_found:
    status_texts.append("Detected: Triangle or Rectangle")
for i, text in enumerate(status_texts):
    cv2.putText(img_resized, text, (50, 50 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2)

# Отображение результатов
cv2.imshow("Red Mask", cv2.resize(mask_red, (d // 2, v // 2)))
cv2.imshow("Processed Image", img_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("ended")
