import cv2
import numpy as np

import time

# ARDUINO = serial.Serial('COM11', 9600) # COM3 нужно заменить на тот порт, куда поключена плата
e = '0'
time.sleep(2)

print("connected")

vid = cv2.VideoCapture(0)

print("started")

while True:

    if cv2.waitKey(10) == 27:
        break
    
    ret, img = vid.read()

    v = int(img.shape[0] * 1.0)
    d = int(img.shape[1] * 1.0)

    img_resized = cv2.resize(img, (d, v))
  
    # Конвертация изображения в HSV
    hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    
    # Определение диапазона красного цвета в HSV (расширенные диапазоны)
    lower_red1 = np.array([0, 100, 50])    # Более широкий диапазон
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 100, 50])   # Более широкий диапазон
    upper_red2 = np.array([180, 255, 255])

    # Определение диапазона зелёного цвета в HSV (расширенные диапазоны)
    lower_green1 = np.array([40, 70, 70])    # Более точный нижний предел
    upper_green1 = np.array([80, 255, 255])  # Более точный верхний предел
    lower_green2 = np.array([30, 70, 40])    # Более точный нижний предел
    upper_green2 = np.array([85, 255, 255])  # Более точный верхний предел

    # Определение диапазона синего цвета в HSV
    lower_blue1 = np.array([100, 70, 70])    # Нижний предел для темно-синего
    upper_blue1 = np.array([130, 255, 255])  # Верхний предел для синего

    lower_blue2 = np.array([90, 70, 40])     # Расширенный нижний предел
    upper_blue2 = np.array([140, 255, 255])  # Расширенный верхний предел

    # Создание масок для красного цвета
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Создание масок для зеленого цвета
    mask_green1 = cv2.inRange(hsv, lower_green1, upper_green1)
    mask_green2 = cv2.inRange(hsv, lower_green2, upper_green2)
    mask_green = cv2.bitwise_or(mask_green1, mask_green2)

    # Создание масок для синего цвета
    mask_blue1 = cv2.inRange(hsv, lower_blue1, upper_blue1)
    mask_blue2 = cv2.inRange(hsv, lower_blue2, upper_blue2)
    mask_blue = cv2.bitwise_or(mask_blue1, mask_blue2)

    # Морфологические операции для улучшения масок
    kernel = np.ones((3,3), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # Поиск контуров
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    triangle_found = False
    circle_found = False

    # Обработка КРАСНЫХ объектов (треугольники и круги)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if 300 < area < 15000:
            # Сначала проверяем на треугольник
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Проверка на треугольник
            if len(approx) == 3 and cv2.isContourConvex(approx):
                triangle_found = True
                cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
                
                # Рисуем вершины треугольника
                for point in approx:
                    cv2.circle(img_resized, tuple(point[0]), 5, (0, 0, 255), -1)
                
                # Находим bounding rect для текста
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(img_resized, "Red Triangle", 
                           (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Затем проверяем на круг (только если не треугольник)
            elif len(approx) >= 6:  # У круга должно быть много вершин
                # Вычисляем параметры для определения круга
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Находим ограничивающую окружность
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    # Вычисляем соотношение площадей
                    circle_area = np.pi * radius * radius
                    area_ratio = area / circle_area if circle_area > 0 else 0
                    
                    # Более строгие критерии для круга
                    if (0.7 < circularity < 1.3 and 
                        0.6 < area_ratio < 0.9 and  # Более строгий диапазон
                        radius > 10):  # Минимальный размер
                        
                        circle_found = True
                        cv2.circle(img_resized, center, radius, (0, 0, 255), 3)
                        cv2.circle(img_resized, center, 3, (0, 0, 255), -1)
                        cv2.putText(img_resized, "Red Circle", 
                                   (int(x - radius), int(y - radius - 10)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Обработка ЗЕЛЁНЫХ объектов (треугольники)
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if 300 < area < 10000:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 3 and cv2.isContourConvex(approx):
                triangle_found = True
                cv2.drawContours(img_resized, [approx], -1, (0, 255, 0), 3)
                
                for point in approx:
                    cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 0), 1)
                
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(img_resized, "Green Triangle", 
                           (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            
            # Затем проверяем на круг (только если не треугольник)
            elif len(approx) >= 6:  # У круга должно быть много вершин
                # Вычисляем параметры для определения круга
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Находим ограничивающую окружность
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    # Вычисляем соотношение площадей
                    circle_area = np.pi * radius * radius
                    area_ratio = area / circle_area if circle_area > 0 else 0
                    
                    # Более строгие критерии для круга
                    if (0.7 < circularity < 1.3 and 
                        0.6 < area_ratio < 0.9 and  # Более строгий диапазон
                        radius > 10):  # Минимальный размер
                        
                        circle_found = True
                        cv2.circle(img_resized, center, radius, (0, 255, 0), 3)
                        cv2.circle(img_resized, center, 3, (0, 255, 0), -1)
                        cv2.putText(img_resized, "Green Circle", 
                                   (int(x - radius), int(y - radius - 10)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
    # Обработка СИНИХ объектов (треугольники)
    for contour in contours_blue:
        area = cv2.contourArea(contour)
        if 300 < area < 10000:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 3 and cv2.isContourConvex(approx):
                triangle_found = True
                cv2.drawContours(img_resized, [approx], -1, (255, 0, 0), 3)
                
                for point in approx:
                    cv2.circle(img_resized, tuple(point[0]), 5, (255, 0, 0), -1)
                
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(img_resized, "Blue Triangle", 
                           (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # Затем проверяем на круг (только если не треугольник)
            elif len(approx) >= 6:  # У круга должно быть много вершин
                # Вычисляем параметры для определения круга
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Находим ограничивающую окружность
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    # Вычисляем соотношение площадей
                    circle_area = np.pi * radius * radius
                    area_ratio = area / circle_area if circle_area > 0 else 0
                    
                    # Более строгие критерии для круга
                    if (0.7 < circularity < 1.3 and 
                        0.6 < area_ratio < 0.9 and  # Более строгий диапазон
                        radius > 10):  # Минимальный размер
                        
                        circle_found = True
                        cv2.circle(img_resized, center, radius, (255, 0, 0), 3)
                        cv2.circle(img_resized, center, 3, (255, 0, 0), -1)
                        cv2.putText(img_resized, "Blue Circle", 
                                   (int(x - radius), int(y - radius - 10)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Обработка прямоугольников (отдельный цикл для лучшего разделения)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if 300 < area < 15000:
            epsilon = 0.02 * cv2.arcLength(contour, True)  # Более точная аппроксимация
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Проверка на прямоугольник (4 вершины)
            if len(approx) == 4:
                # Проверяем форму bounding rect
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / h if h > 0 else 0
                
                # Если объект примерно прямоугольной формы
                if 0.5 < aspect_ratio < 2.0:
                    cv2.drawContours(img_resized, [approx], -1, (0, 100, 255), 2)
                    cv2.putText(img_resized, "Red Rectangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)

    # Аналогично для зеленых и синих прямоугольников
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if 300 < area < 15000:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / h if h > 0 else 0
                
                if 0.5 < aspect_ratio < 2.0:
                    cv2.drawContours(img_resized, [approx], -1, (100, 255, 100), 2)
                    cv2.putText(img_resized, "Green Rectangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)

    for contour in contours_blue:
        area = cv2.contourArea(contour)
        if 300 < area < 15000:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / h if h > 0 else 0
                
                if 0.5 < aspect_ratio < 2.0:
                    cv2.drawContours(img_resized, [approx], -1, (255, 100, 100), 2)
                    cv2.putText(img_resized, "Blue Rectangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)

    # Вывод текста в зависимости от присутствия объектов
    status_text = []
    
    if triangle_found:
        status_text.append("triangle detected")
    
    if circle_found:
        status_text.append("red circle detected")

    # Отображение статуса на изображении
    for i, text in enumerate(status_text):
        cv2.putText(img_resized, text, (50, 50 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # Дополнительно: показываем маску красного для отладки
    mask_display = cv2.resize(mask_red, (d//2, v//2))
    cv2.imshow("Red Mask", mask_display)
    
    cv2.imshow("robozari 2026 v0.1", img_resized)
    
cv2.destroyAllWindows()

print("ended")
