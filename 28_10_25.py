import cv2
import numpy as np
#import serial
import time
import math

# ARDUINO = serial.Serial('COM11', 9600) # COM3 нужно заменить на тот порт, куда поключена плата
e = '0'
time.sleep(2)

print("connected")

vid = cv2.VideoCapture(0)

print("started")

# Переменные для хранения координат центров фигур
triangle_centers = []  # Список центров треугольников
circle_centers = []    # Список центров кругов
rectangle_centers = [] # Список центров прямоугольников
robot_center = None    # Центр робота (жёлтый квадрат)

# CONFIG (FORM, COLOR)
TARGET_FIGURES = [
    ["triangle", "green"],
    ["circle", "blue"],
    ["circle", "red"],
    ["rectangle", "green"]
]

def calculate_angle_and_distance(robot_pos, target_pos):
    """Вычисляет угол поворота и расстояние до цели относительно робота"""
    if robot_pos is None or target_pos is None:
        return None, None
    
    rx, ry = robot_pos
    tx, ty = target_pos
    
    # Вычисляем вектор от робота к цели
    dx = tx - rx
    dy = ty - ry
    
    # Вычисляем расстояние (евклидово)
    distance = math.sqrt(dx**2 + dy**2)
    
    # Вычисляем угол в радианах (относительно положительной оси X)
    angle_rad = math.atan2(dy, dx)
    
    # Преобразуем в градусы (0-360, где 0 - направление вправо)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg, distance

def draw_robot_reference(frame, robot_pos):
    """Рисует систему координат робота"""
    if robot_pos is None:
        return
    
    rx, ry = robot_pos
    # Рисуем крест в центре робота
    cv2.line(frame, (rx-10, ry), (rx+10, ry), (0, 255, 255), 2)
    cv2.line(frame, (rx, ry-10), (rx, ry+10), (0, 255, 255), 2)
    
    # Рисуем направление "вперёд" (ось X)
    cv2.arrowedLine(frame, (rx, ry), (rx+30, ry), (0, 255, 255), 2)

def draw_target_line(frame, robot_pos, target_pos, color):
    """Рисует линию от робота к цели"""
    if robot_pos is None or target_pos is None:
        return
    
    rx, ry = robot_pos
    tx, ty = target_pos
    
    # Рисуем линию от робота к цели
    cv2.line(frame, (rx, ry), (tx, ty), color, 2)
    
    # Рисуем маленький круг в точке цели
    cv2.circle(frame, (tx, ty), 5, color, -1)

while True:

    if cv2.waitKey(10) == 27:
        break
    
    ret, img = vid.read()

    v = int(img.shape[0] * 1.0)
    d = int(img.shape[1] * 1.0)

    img_resized = cv2.resize(img, (d, v))
  
    # Очищаем списки координат для нового кадра
    triangle_centers.clear()
    circle_centers.clear()
    rectangle_centers.clear()
    robot_center = None
    
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

    lower_yellow1 = np.array([20, 70, 70])    # Нижний предел желтого
    upper_yellow1 = np.array([40, 255, 255])  # Верхний предел желтого
    lower_yellow2 = np.array([15, 50, 50])     # Расширенный нижний предел
    upper_yellow2 = np.array([45, 255, 255])   # Расширенный верхний предел

    # Создание масок для жёлтого цвета
    mask_yellow1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
    mask_yellow2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
    mask_yellow = cv2.bitwise_or(mask_yellow1, mask_yellow2)
    
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

    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    
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
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                
                # Вычисляем центр треугольника
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    triangle_centers.append((cx, cy, "red"))
                
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
                        circle_centers.append((center[0], center[1], "red"))
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
                
                # Вычисляем центр треугольника
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    triangle_centers.append((cx, cy, "green"))
                
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
                        circle_centers.append((center[0], center[1], "green"))
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
                
                # Вычисляем центр треугольника
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    triangle_centers.append((cx, cy, "blue"))
                
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
                        circle_centers.append((center[0], center[1], "blue"))
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
                    # Вычисляем центр прямоугольника
                    cx = x + w // 2
                    cy = y + h // 2
                    rectangle_centers.append((cx, cy, "red"))
                    
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
                    # Вычисляем центр прямоугольника
                    cx = x + w // 2
                    cy = y + h // 2
                    rectangle_centers.append((cx, cy, "green"))
                    
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
                    # Вычисляем центр прямоугольника
                    cx = x + w // 2
                    cy = y + h // 2
                    rectangle_centers.append((cx, cy, "blue"))
                    
                    cv2.drawContours(img_resized, [approx], -1, (255, 100, 100), 2)
                    cv2.putText(img_resized, "Blue Rectangle", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)

    # Обработка РОБОТА (жёлтый квадрат)
    for contour in contours_yellow:
        area = cv2.contourArea(contour)
        if 300 < area < 1500000:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / h if h > 0 else 0
                
                if 0.5 < aspect_ratio < 2.0:
                    # Вычисляем центр робота
                    cx = x + w // 2
                    cy = y + h // 2
                    robot_center = (cx, cy)
                    
                    cv2.drawContours(img_resized, [approx], -1, (100, 200, 255), 2)
                    cv2.putText(img_resized, "Robot", 
                               (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)

    # Вывод текста в зависимости от присутствия объектов
    status_text = []
    
    if triangle_found:
        status_text.append("triangle detected")
    
    if circle_found:
        status_text.append("red circle detected")

    # Отображение статуса на изображении
    for i, text in enumerate(status_text):
        cv2.putText(img_resized, text, (50, 50 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Функция для проверки, нужно ли показывать фигуру
    def should_show_figure(figure_type, color):
        for target_figure, target_color in TARGET_FIGURES:
            if target_figure == figure_type and target_color == color:
                return True
        return False
    
    # Рисуем систему координат робота
    draw_robot_reference(img_resized, robot_center)
    
    # Отображение координат только целевых фигур и вычисление углов/расстояний
    coord_y = 100
    target_data = []  # Для хранения данных о целевых фигурах
    
    for center in triangle_centers:
        x, y, color = center
        if should_show_figure("triangle", color):
            angle, distance = calculate_angle_and_distance(robot_center, (x, y))
            if angle is not None and distance is not None:
                target_data.append(("triangle", color, x, y, angle, distance))
                cv2.putText(img_resized, f"Tri {color}: ({x}, {y})", 
                           (50, coord_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                coord_y += 20
                # Рисуем линию к цели
                draw_target_line(img_resized, robot_center, (x, y), (0, 255, 0))
    
    for center in circle_centers:
        x, y, color = center
        if should_show_figure("circle", color):
            angle, distance = calculate_angle_and_distance(robot_center, (x, y))
            if angle is not None and distance is not None:
                target_data.append(("circle", color, x, y, angle, distance))
                cv2.putText(img_resized, f"Cir {color}: ({x}, {y})", 
                           (50, coord_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                coord_y += 20
                # Рисуем линию к цели
                draw_target_line(img_resized, robot_center, (x, y), (255, 0, 0))
    
    for center in rectangle_centers:
        x, y, color = center
        if should_show_figure("rectangle", color):
            angle, distance = calculate_angle_and_distance(robot_center, (x, y))
            if angle is not None and distance is not None:
                target_data.append(("rectangle", color, x, y, angle, distance))
                cv2.putText(img_resized, f"Rect {color}: ({x}, {y})", 
                           (50, coord_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                coord_y += 20
                # Рисуем линию к цели
                draw_target_line(img_resized, robot_center, (x, y), (0, 0, 255))
    
    # Отображение информации об угле и расстоянии
    info_y = coord_y + 30
    if robot_center:
        cv2.putText(img_resized, f"Robot: ({robot_center[0]}, {robot_center[1]})", 
                   (50, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        info_y += 30
    
    for figure_type, color, x, y, angle, distance in target_data:
        cv2.putText(img_resized, f"{color} {figure_type}: {angle:.1f}°, {distance:.1f}px", 
                   (50, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25

    # Вывод координат в консоль только для целевых фигур
    target_figures_found = False
    console_output = []
    
    for figure_type, color, x, y, angle, distance in target_data:
        console_output.append(f"{figure_type} {color}: центр в ({x}, {y}), угол: {angle:.1f}°, расстояние: {distance:.1f}px")
        target_figures_found = True
    
    if target_figures_found:
        print("\n" + "="*50)
        print("ЦЕЛЕВЫЕ фигуры и их параметры относительно робота:")
        for output in console_output:
            print(output)
        print("="*50)

    # Дополнительно: показываем маску красного для отладки
    mask_display = cv2.resize(mask_red, (d//2, v//2))
    cv2.imshow("mask", mask_display)
    
    cv2.imshow("robozari 2026 v0.2", img_resized)
    
cv2.destroyAllWindows()

print("ended")
