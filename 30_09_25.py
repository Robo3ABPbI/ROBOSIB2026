import cv2
import numpy as np
import serial
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

    v = int(img.shape[0] * 0.7)
    d = int(img.shape[1] * 0.7)

    img_resized = cv2.resize(img, (d, v))
  
    # Конвертация изображения в HSV
    hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    """
    # Определение диапазона оранжевого цвета в HSV
    lower_orange = np.array([7, 95, 134])  # Нижний предел
    upper_orange = np.array([31, 255, 255])  # Верхний предел
    """
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

    # Создание маски для оранжевого цвета
#    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    # Создание масок для красного цвета
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Создание масок для красного цвета
    mask_green1 = cv2.inRange(hsv, lower_green1, upper_green1)
    mask_green2 = cv2.inRange(hsv, lower_green2, upper_green2)
    mask_green = cv2.bitwise_or(mask_green1, mask_green2)

    # Морфологические операции для улучшения маски красного
    kernel = np.ones((3,3), np.uint8)  # Уменьшил ядро
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    # Поиск контуров в маске оранжевого
#    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Поиск контуров в маске красного
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Поиск контуров в маске зелёного
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#    helmet_found = False
    triangle_found = False
    """
    # Обработка оранжевых объектов (каски)
    for contour in contours_orange:
        area = cv2.contourArea(contour)
        if area > 400:
            helmet_found = True
            cv2.drawContours(img_resized, [contour], -1, (255, 0, 0), 3)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.putText(img_resized, "Helmet", 
                       (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            break
    """
    # Обработка красных объектов (треугольники)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        # Уменьшил минимальную площадь и добавил максимальную
        if 300 < area < 10000:  # Более широкий диапазон площади
            # Аппроксимация контура
            epsilon = 0.04 * cv2.arcLength(contour, True)  # Увеличил epsilon
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Если контур имеет 3 вершины - это треугольник
            if len(approx) == 3:
                # Упрощенные проверки (убрал строгие фильтры)
                perimeter = cv2.arcLength(contour, True)
                
                # Проверка на выпуклость
                if cv2.isContourConvex(approx):
                    triangle_found = True
                    cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
                    
                    # Рисуем вершины треугольника
                    for point in approx:
                        cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 255), -1)
                    
                    # Находим bounding rect для текста
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.putText(img_resized, "Red Triangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Обработка красных объектов (треугольники)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        # Уменьшил минимальную площадь и добавил максимальную
        if 300 < area < 10000:  # Более широкий диапазон площади
            # Аппроксимация контура
            epsilon = 0.04 * cv2.arcLength(contour, True)  # Увеличил epsilon
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Если контур имеет 3 вершины - это треугольник
            if len(approx) == 3:
                # Упрощенные проверки (убрал строгие фильтры)
                perimeter = cv2.arcLength(contour, True)
                
                # Проверка на выпуклость
                if cv2.isContourConvex(approx):
                    triangle_found = True
                    cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
                    
                    # Рисуем вершины треугольника
                    for point in approx:
                        cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 255), -1)
                    
                    # Находим bounding rect для текста
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.putText(img_resized, "Red Triangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Обработка зелёных объектов (треугольники)
    for contour in contours_green:
        area = cv2.contourArea(contour)
        # Уменьшил минимальную площадь и добавил максимальную
        if 300 < area < 10000:  # Более широкий диапазон площади
            # Аппроксимация контура
            epsilon = 0.04 * cv2.arcLength(contour, True)  # Увеличил epsilon
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Если контур имеет 3 вершины - это треугольник
            if len(approx) == 3:
                # Упрощенные проверки (убрал строгие фильтры)
                perimeter = cv2.arcLength(contour, True)
                
                # Проверка на выпуклость
                if cv2.isContourConvex(approx):
                    triangle_found = True
                    cv2.drawContours(img_resized, [approx], -1, (0, 0, 255), 3)
                    
                    # Рисуем вершины треугольника
                    for point in approx:
                        cv2.circle(img_resized, tuple(point[0]), 5, (0, 255, 255), -1)
                    
                    # Находим bounding rect для текста
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.putText(img_resized, "Green Triangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Также проверяем контуры с 4 вершинами (могут быть квадратными знаками с треугольным содержимым)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if 500 < area < 15000:
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Если контур имеет 3-5 вершин (возможны небольшие погрешности)
            if 3 <= len(approx) == 4:
                # Проверяем форму bounding rect
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / h if h > 0 else 0
                
                # Если объект примерно треугольной формы
                if 0.5 < aspect_ratio < 2.0:
                    triangle_found = True
                    cv2.drawContours(img_resized, [approx], -1, (0, 255, 255), 2)
                    cv2.putText(img_resized, "Red Rectangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Вывод текста в зависимости от присутствия объектов
    status_text = []
    """
    if helmet_found:
        status_text.append("see cask")
        e = '1'
    else:
        status_text.append("no cask!")
        e = '0'
    """    
    if triangle_found:
        status_text.append("red triangle detected")

    # Отображение статуса на изображении
    for i, text in enumerate(status_text):
        cv2.putText(img_resized, text, (50, 50 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # Дополнительно: показываем маску красного для отладки
    mask_display = cv2.resize(mask_red, (d//2, v//2))
    cv2.imshow("Red Mask", mask_display)
    
    cv2.imshow("robozari 2026 v0.1", img_resized)
    
#    i = e.encode()
#    ARDUINO.write(i)
    
cv2.destroyAllWindows()

print("ended")
