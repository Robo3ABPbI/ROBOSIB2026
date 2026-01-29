"""
Серерная часть, её НЕ ТРОГАТЬ

"""
import socket
import pickle
import time

hostAddress = '98:3B:8F:C7:1A:2C'
channel = 4
backlog = 1
mySocket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
mySocket.bind((hostAddress, channel))
print("проверка работы клиента")
mySocket.listen()
print("соединение...")
client, address = mySocket.accept()

"""
ОСНОВНОЙ КОД

"""
print("coolbot connected")

import cv2
import numpy as np
import itertools
from math import sqrt, inf, atan2, degrees, pi, cos, sin

# Диапазоны цветов в HSV
yellow_LOWER_COLOR = np.array([15, 103, 138])
yellow_UPPER_COLOR = np.array([100, 255, 255])

purple_LOWER_COLOR = np.array([110, 60, 60])
purple_UPPER_COLOR = np.array([255, 255, 255])

green_LOWER_COLOR = np.array([45, 45, 60])
green_UPPER_COLOR = np.array([115, 255, 255])

red_LOWER_COLOR = np.array([0, 95, 95])
red_UPPER_COLOR = np.array([10, 255, 255])

blue_LOWER_COLOR = np.array([90, 50, 50])
blue_UPPER_COLOR = np.array([111, 255, 255])

# Захват видео с камеры
vid = cv2.VideoCapture(0)
# Список для хранения информации об объектах
objects_info = []

# разница углов (её допустимый максимум)
delta_angles_ROBOT_FIGURE = 3.0

# границы поиска по х
Xmin = 70
Xmax = 540

# границы поиска по y
Ymin = 0
Ymax = 425

# Пороги площадей для больших объектов
MIN_BIG_AREA_YELLOW = 250  # Минимальная площадь для больших жёлтых пятен
MIN_BIG_AREA_PURPLE = 250  # Минимальная площадь для больших фиолетовых пятен
MAX_SMALL_AREA = 50  # Максимальная площадь для маленьких объектов

# Целевые углы в зависимости от цвета объектов
TARGET_CORNERS = {
    "Red": (100, 50),           # Левый верхний угол
    "Blue": (100, 470),         # Левый нижний угол
    "Dark Green": (560, 50),    # Правый верхний угол
    "Green": (560, 470),        # Правый нижний угол
    "Yellow": (320, 240),       # Центр
    "Purple": (320, 240)        # Центр
}

# Параметры для избегания препятствий
OBSTACLE_AVOIDANCE_RADIUS = 25  # Радиус безопасности вокруг объектов
SINGLE_PATH_MODE = True  # Режим одного стабильного маршрута

# Глобальные переменные для углов
target_angle = None
current_angle = None
angle_difference = None
robot_aligned = False

# глобальный флаг для работы робота
move_left = False

# Для управления частотой отправки команд
last_send_time = 0
SEND_INTERVAL = 0.05  # 50 мс между отправками команд

# Для стабильности направления поворота
last_turn_direction = None
TURN_DIRECTION_HYSTERESIS = 5.0

# Параметры маршрута
ITEM_CAPTURE_DISTANCE = 5  # Расстояние для захвата предмета (пиксели)
FINAL_POINT_DISTANCE = 15   # Расстояние до конечной точки
captured_items = []         # Список захваченных предметов
current_target_index = 1    # Текущий индекс целевой точки в маршруте
optimal_path = None         # Текущий маршрут
route_completed = False     # Флаг завершения маршрута

def calculate_distance(point1, point2):
    """Вычисляет Евклидово расстояние между двумя точками"""
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def calculate_angle(point1, point2):
    """Вычисляет угол между горизонталью и линией point1->point2 в градусах (0-360)"""
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    angle_rad = atan2(dy, dx)
    angle_deg = degrees(angle_rad)
    
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg

def angle_difference_deg(angle1, angle2):
    """Вычисляет минимальную разницу между двумя углами в градусах и определяет направление поворота"""
    global move_left, last_turn_direction
    
    diff = (angle2 - angle1) % 360
    
    if diff > 180:
        angle_diff = 360 - diff
        turn_left = False
    else:
        angle_diff = diff
        turn_left = True
    
    if last_turn_direction is not None and angle_diff < TURN_DIRECTION_HYSTERESIS:
        if last_turn_direction == "left":
            turn_left = True
        else:
            turn_left = False
    else:
        if angle_diff >= TURN_DIRECTION_HYSTERESIS:
            last_turn_direction = "left" if turn_left else "right"
    
    move_left = turn_left
    return angle_diff

def line_intersects_circle(p1, p2, center, radius):
    """Проверяет, пересекает ли линия p1-p2 круг с центром center и радиусом radius"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    fx = center[0] - p1[0]
    fy = center[1] - p1[1]
    
    segment_length_squared = dx*dx + dy*dy
    
    if segment_length_squared > 0:
        t = max(0, min(1, (fx*dx + fy*dy) / segment_length_squared))
    else:
        t = 0
    
    closest_x = p1[0] + t * dx
    closest_y = p1[1] + t * dy
    
    distance = sqrt((center[0] - closest_x)**2 + (center[1] - closest_y)**2)
    
    return distance <= radius

def find_best_avoidance_side(start, goal, obstacle, obstacle_radius, other_obstacles):
    """Находит лучшую сторону для обхода препятствия"""
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    
    length = sqrt(dx*dx + dy*dy)
    if length > 0:
        dx /= length
        dy /= length
    
    perp1 = (-dy, dx)   # Левая сторона
    perp2 = (dy, -dx)   # Правая сторона
    
    left_point = (
        int(obstacle[0] + perp1[0] * obstacle_radius * 1.5),
        int(obstacle[1] + perp1[1] * obstacle_radius * 1.5)
    )
    
    right_point = (
        int(obstacle[0] + perp2[0] * obstacle_radius * 1.5),
        int(obstacle[1] + perp2[1] * obstacle_radius * 1.5)
    )
    
    cross_product = (goal[0] - start[0]) * (obstacle[1] - start[1]) - \
                    (goal[1] - start[1]) * (obstacle[0] - start[0])
    
    if cross_product > 0:
        return "right", right_point
    else:
        return "left", left_point

def find_path_with_obstacle_avoidance(start, goal, obstacles, obstacle_radius):
    """Находит путь от start до goal с обходом препятствий"""
    if not obstacles:
        return [start, goal]
    
    direct_path_clear = True
    blocking_obstacles = []
    
    for obstacle in obstacles:
        if line_intersects_circle(start, goal, obstacle, obstacle_radius):
            direct_path_clear = False
            blocking_obstacles.append(obstacle)
    
    if direct_path_clear:
        return [start, goal]
    
    blocking_obstacles.sort(key=lambda obs: calculate_distance(start, obs))
    
    path_points = [start]
    current_pos = start
    
    for i, obstacle in enumerate(blocking_obstacles):
        if not line_intersects_circle(current_pos, goal, obstacle, obstacle_radius):
            continue
        
        other_obstacles = [o for o in blocking_obstacles if o != obstacle]
        side, avoid_point = find_best_avoidance_side(current_pos, goal, obstacle, obstacle_radius, other_obstacles)
        
        point_valid = True
        for obs in obstacles:
            if calculate_distance(avoid_point, obs) < obstacle_radius * 0.8:
                point_valid = False
                break
        
        if point_valid:
            approach_distance = obstacle_radius * 0.7
            dx = obstacle[0] - current_pos[0]
            dy = obstacle[1] - current_pos[1]
            length = sqrt(dx*dx + dy*dy)
            
            if length > 0:
                dx /= length
                dy /= length
                
                approach_point = (
                    int(obstacle[0] - dx * approach_distance),
                    int(obstacle[1] - dy * approach_distance)
                )
                
                approach_clear = True
                for obs in obstacles:
                    if line_intersects_circle(current_pos, approach_point, obs, obstacle_radius):
                        approach_clear = False
                        break
                
                if approach_clear:
                    path_points.append(approach_point)
                    current_pos = approach_point
            
            path_points.append(avoid_point)
            current_pos = avoid_point
    
    path_points.append(goal)
    return path_points

def find_optimal_path_with_avoidance(start_point, points, end_point, all_obstacles, obstacle_radius):
    """Находит оптимальный путь с обходом препятствий"""
    if not points:
        return [start_point, end_point]
    
    points_sorted = sorted(points, key=lambda p: (p[0], p[1]))
    
    full_path = [start_point]
    unvisited = points_sorted.copy()
    current = start_point
    
    while unvisited:
        nearest = None
        min_dist = float('inf')
        
        for point in unvisited:
            dist = calculate_distance(current, point)
            if dist < min_dist:
                min_dist = dist
                nearest = point
        
        if nearest:
            segment_path = find_path_with_obstacle_avoidance(current, nearest, all_obstacles, obstacle_radius)
            
            if len(segment_path) > 1:
                full_path.extend(segment_path[1:])
            
            current = nearest
            unvisited.remove(nearest)
    
    final_segment = find_path_with_obstacle_avoidance(current, end_point, all_obstacles, obstacle_radius)
    if len(final_segment) > 1:
        full_path.extend(final_segment[1:])
    
    return full_path

def draw_obstacles(frame, obstacles, radius, color=(128, 128, 128)):
    """Рисует препятствия на кадре"""
    for obstacle in obstacles:
        cv2.circle(frame, obstacle, radius, color, 2)
        cv2.circle(frame, obstacle, 2, color, -1)

def check_item_captured_simple(robot_front_center):
    """Упрощённая проверка захвата предмета"""
    global current_target_index, captured_items, optimal_path
    
    if robot_front_center is None or optimal_path is None or len(optimal_path) < 3:
        return False
    
    # Проверяем, есть ли текущая целевая точка
    if current_target_index < len(optimal_path) - 1:  # Не проверяем конечную точку
        current_target = optimal_path[current_target_index]
        
        # Вычисляем расстояние до текущей цели
        distance = calculate_distance(robot_front_center, current_target)
        
        # Проверяем захват
        if current_target not in captured_items and distance < ITEM_CAPTURE_DISTANCE:
            captured_items.append(current_target)

            dataToSend = 5  # Вперёд
            bytesToSend = pickle.dumps(dataToSend)
            client.send(bytesToSend)
                
            
            print(f"✓ Предмет {current_target_index} захвачен! Расстояние: {distance:.1f}px")
            
            # Переходим к следующей точке
            current_target_index += 1
            
            # Если следующая точка есть, выводим информацию
            if current_target_index < len(optimal_path):
                next_target = optimal_path[current_target_index]
                print(f"→ Переход к точке {current_target_index} ({next_target})")
            
            return True
    
    return False

def check_final_point_reached(robot_front_center):
    """Проверяет, достиг ли робот конечной точки"""
    global current_target_index, optimal_path, route_completed
    
    if robot_front_center is None or optimal_path is None or len(optimal_path) < 2:
        return False
    
    # Получаем конечную точку маршрута
    final_point = optimal_path[-1]
    
    # Проверяем расстояние до конечной точки
    distance = calculate_distance(robot_front_center, final_point)
    
    if distance < FINAL_POINT_DISTANCE and not route_completed:
        route_completed = True
        #print(f"✓ Конечная точка достигнута! Расстояние: {distance:.1f}px")
        #print("✓ Маршрут завершён!")
        return True
    
    return False

def get_next_target_point():
    """Получает следующую целевую точку в маршруте"""
    global current_target_index, optimal_path, captured_items
    
    if optimal_path is None or len(optimal_path) < 2:
        return None
    
    # Если мы дошли до конца маршрута
    if current_target_index >= len(optimal_path):
        return optimal_path[-1]
    
    # Получаем текущую целевую точку
    target_point = optimal_path[current_target_index]
    
    # Если текущая точка уже захвачена, переходим к следующей
    while current_target_index < len(optimal_path) - 1 and target_point in captured_items:
        current_target_index += 1
        target_point = optimal_path[current_target_index]
    
    return target_point

def calculate_total_path_length(path):
    """Вычисляет общую длину пути"""
    if len(path) < 2:
        return 0
    
    total = 0
    for i in range(len(path) - 1):
        total += calculate_distance(path[i], path[i + 1])
    
    return total

while True:
    ret, frame = vid.read()
    if not ret:
        break
    
    # Получаем размеры кадра
    frame_height, frame_width = frame.shape[:2]
    
    # Преобразование в HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Маски для каждого цвета
    mask_yellow = cv2.inRange(hsv, yellow_LOWER_COLOR, yellow_UPPER_COLOR)
    mask_purple = cv2.inRange(hsv, purple_LOWER_COLOR, purple_UPPER_COLOR)
    mask_green = cv2.inRange(hsv, green_LOWER_COLOR, green_UPPER_COLOR)
    mask_red = cv2.inRange(hsv, red_LOWER_COLOR, red_UPPER_COLOR)
    mask_blue = cv2.inRange(hsv, blue_LOWER_COLOR, blue_UPPER_COLOR)

    # Обработка масок
    masks = [mask_blue, mask_red, mask_green, mask_yellow, mask_purple]
    colors = [(255, 0, 0), (0, 0, 255), (0, 255, 0), (0, 255, 255), (255, 0, 255)]
    color_names = ["Blue", "Red", "Green", "Yellow", "Purple"]

    # Очищаем список объектов для нового кадра
    objects_info = []
    
    # Переменные для хранения центров робота
    robot_front_center = None
    robot_back_center = None
    
    # Списки для хранения всех объектов
    all_objects_info = []

    for mask, color, color_name in zip(masks, colors, color_names):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            if Xmin <= x <= Xmax and Ymin <= y <= Ymax:
                center_x, center_y = x + w // 2, y + h // 2
                
                # Обработка зелёных объектов
                if color == (0, 255, 0):
                    if area > 180:
                        cv2.drawContours(frame, [approx], 0, (0, 128, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 128, 0), -1)
                        objects_info.append([(0, 128, 0), (center_x, center_y), "Dark Green", area])
                        all_objects_info.append([(0, 128, 0), (center_x, center_y), "Dark Green", area])
                    elif area > MAX_SMALL_AREA:
                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                        objects_info.append([(0, 255, 0), (center_x, center_y), "Green", area])
                        all_objects_info.append([(0, 255, 0), (center_x, center_y), "Green", area])
                
                # Обработка жёлтых объектов
                elif color == (0, 255, 255):
                    if area > MIN_BIG_AREA_YELLOW:
                        cv2.drawContours(frame, [approx], 0, color, 3)
                        cv2.circle(frame, (center_x, center_y), 8, color, -1)
                        robot_back_center = (center_x, center_y)
                        cv2.putText(frame, f"ROBOT BACK", 
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.6, color, 2)
                        all_objects_info.append([color, (center_x, center_y), "ROBOT", area])
                
                # Обработка фиолетовых объектов
                elif color == (255, 0, 255):
                    if area > MIN_BIG_AREA_PURPLE:
                        cv2.drawContours(frame, [approx], 0, color, 3)
                        cv2.circle(frame, (center_x, center_y), 8, color, -1)
                        robot_front_center = (center_x, center_y)
                        cv2.putText(frame, f"ROBOT FRONT", 
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.6, color, 2)
                        all_objects_info.append([color, (center_x, center_y), "ROBOT", area])
                
                # Обработка синих и красных объектов
                elif color == (255, 0, 0) or color == (0, 0, 255):
                    if area > 10 and area < 250:
                        cv2.drawContours(frame, [approx], 0, color, 2)
                        cv2.circle(frame, (center_x, center_y), 5, color, -1)
                        
                        if color == (255, 0, 0):
                            obj_color_name = "Blue"
                        elif color == (0, 0, 255):
                            obj_color_name = "Red"
                        else:
                            obj_color_name = color_name
                        
                        objects_info.append([color, (center_x, center_y), obj_color_name, area])
                        all_objects_info.append([color, (center_x, center_y), obj_color_name, area])

    # Основная логика построения маршрута
    closest_point = None
    target_corner = None
    current_closest_color_name = None
    
    if robot_front_center and robot_back_center and len(objects_info) > 0:
        # Отделяем объекты от робота
        target_objects = []
        for obj in objects_info:
            obj_color_name = obj[2]
            if obj_color_name not in ["ROBOT", "Yellow", "Purple"]:
                target_objects.append(obj)
        
        if target_objects:
            # Находим ближайший объект
            closest_obj = None
            min_distance = float('inf')
            
            for obj in target_objects:
                obj_center = obj[1]
                distance = calculate_distance(robot_front_center, obj_center)
                if distance < min_distance:
                    min_distance = distance
                    closest_obj = obj
            
            if closest_obj:
                current_closest_color_name = closest_obj[2]
                closest_center = closest_obj[1]
                closest_point = closest_center
                
                # Отображаем информацию
                cv2.putText(frame, f"Target: {current_closest_color_name}", 
                           (frame_width - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
                
                # Находим все объекты того же цвета
                same_color_objects = []
                for obj in target_objects:
                    if obj[2] == current_closest_color_name:
                        same_color_objects.append(obj[1])
                
                if same_color_objects:
                    # Получаем целевой угол
                    target_corner = TARGET_CORNERS.get(current_closest_color_name, 
                                                      (frame_width//2, frame_height//2))
                    
                    # Формируем список препятствий
                    obstacle_centers = []
                    for obj_info in all_objects_info:
                        obj_center = obj_info[1]
                        obj_name = obj_info[2]
                        
                        if obj_name not in ["ROBOT", current_closest_color_name]:
                            obstacle_centers.append(obj_center)
                    
                    # Рисуем препятствия
                    draw_obstacles(frame, obstacle_centers, OBSTACLE_AVOIDANCE_RADIUS, (128, 128, 128))
                    
                    # Строим маршрут (если ещё не построен)
                    if optimal_path is None or route_completed:
                        optimal_path = find_optimal_path_with_avoidance(
                            robot_front_center, 
                            same_color_objects, 
                            target_corner,
                            obstacle_centers,
                            OBSTACLE_AVOIDANCE_RADIUS
                        )
                        current_target_index = 1
                        captured_items = []
                        route_completed = False
                        print(f"Построен новый маршрут к {current_closest_color_name}")
                        print(f"Точек в маршруте: {len(optimal_path)}")
                        print(f"Путь начинается от: {robot_front_center}")
                    
                    # Проверяем захват предметов
                    check_item_captured_simple(robot_front_center)
                    
                    # Проверяем достижение конечной точки
                    check_final_point_reached(robot_front_center)
                    
                    if optimal_path:
                        # Получаем следующую целевую точку
                        next_target_point = get_next_target_point()
                        if next_target_point:
                            closest_point = next_target_point
                        
                        # Рисуем маршрут
                        points_array = np.array(optimal_path, dtype=np.int32)
                        cv2.polylines(frame, [points_array], isClosed=False, 
                                    color=(0, 255, 0), thickness=3)
                        
                        # Рисуем точки маршрута
                        for i, point in enumerate(optimal_path):
                            if i == 0:  # Старт
                                cv2.circle(frame, point, 8, (255, 0, 255), -1)
                                cv2.putText(frame, "START", (point[0] + 15, point[1]), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                            elif i == len(optimal_path) - 1:  # Конец
                                cv2.circle(frame, point, 10, (0, 0, 0), -1)
                                cv2.circle(frame, point, 10, (255, 255, 255), 2)
                                cv2.putText(frame, "END", (point[0] + 15, point[1]), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            else:  # Промежуточные точки
                                if point in captured_items:
                                    cv2.circle(frame, point, 7, (100, 100, 100), -1)
                                    cv2.putText(frame, "CAPTURED", (point[0] + 15, point[1]), 
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                                elif i == current_target_index:
                                    cv2.circle(frame, point, 10, (255, 255, 0), -1)  # Текущая цель
                                    cv2.circle(frame, point, ITEM_CAPTURE_DISTANCE, (0, 255, 255), 1)
                                else:
                                    cv2.circle(frame, point, 7, (255, 255, 255), -1)
                                    cv2.circle(frame, point, 7, (0, 0, 0), 1)
                                
                                cv2.putText(frame, str(i), (point[0] + 10, point[1]), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # ОБРАБОТКА УГЛОВ И ОТПРАВКА КОМАНД РОБОТУ
    if robot_back_center and robot_front_center and closest_point:
        # Рисуем линию к цели
        cv2.line(frame, robot_back_center, closest_point, 
                (255, 100, 0), 3, cv2.LINE_AA)
        
        # Вычисляем углы
        target_angle = calculate_angle(robot_back_center, closest_point)
        current_angle = calculate_angle(robot_back_center, robot_front_center)
        angle_difference = angle_difference_deg(target_angle, current_angle)
        robot_aligned = angle_difference <= delta_angles_ROBOT_FIGURE
        
        # Рисуем линию направления робота
        cv2.line(frame, robot_back_center, robot_front_center,
                (255, 255, 255), 2, cv2.LINE_AA)
        
        # Отображаем информацию об углах
        angle_info_y = 180
        cv2.putText(frame, f"Target angle: {target_angle:.1f}°", 
                   (frame_width - 200, angle_info_y), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (255, 100, 0), 2)
        cv2.putText(frame, f"Current angle: {current_angle:.1f}°", 
                   (frame_width - 200, angle_info_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Angle diff: {angle_difference:.1f}°", 
                   (frame_width - 200, angle_info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 255, 255) if angle_difference > delta_angles_ROBOT_FIGURE else (0, 255, 0), 2)
        
        # Отображаем направление поворота
        direction_text = "LEFT" if move_left else "RIGHT"
        cv2.putText(frame, f"Turn: {direction_text}", 
                   (frame_width - 200, angle_info_y + 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 255, 255), 2)
        
        # Отображаем информацию о маршруте
        cv2.putText(frame, f"Captured: {len(captured_items)}", 
                   (frame_width - 200, angle_info_y + 120), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 165, 255) if captured_items else (200, 200, 200), 2)
        
        if optimal_path:
            cv2.putText(frame, f"Target: {current_target_index}/{len(optimal_path)-1}", 
                       (frame_width - 200, angle_info_y + 150), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 0), 2)
            
            if current_target_index < len(optimal_path):
                current_target = optimal_path[current_target_index]
                distance_to_target = calculate_distance(robot_front_center, current_target)
                cv2.putText(frame, f"Dist: {int(distance_to_target)}px", 
                           (frame_width - 200, angle_info_y + 180), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
        
        # Отображаем статус завершения маршрута
        if route_completed:
            cv2.putText(frame, "ROUTE COMPLETED!", 
                       (frame_width - 200, angle_info_y + 210), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2)
        
        # Отправка команд роботу
        current_time = time.time()
        if current_time - last_send_time > SEND_INTERVAL and not route_completed:
            if robot_aligned:
                dataToSend = 1  # Вперёд
                bytesToSend = pickle.dumps(dataToSend)
                client.send(bytesToSend)
                cv2.putText(frame, "COMMAND: FORWARD", 
                           (frame_width - 200, angle_info_y + 240), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.7, (0, 255, 0), 2)
                last_send_time = current_time
            elif angle_difference > delta_angles_ROBOT_FIGURE:
                if move_left:
                    dataToSend = 3  # Поворот налево
                else:
                    dataToSend = 2  # Поворот направо
                
                bytesToSend = pickle.dumps(dataToSend)
                client.send(bytesToSend)
                cv2.putText(frame, f"COMMAND: TURN {direction_text}", 
                           (frame_width - 200, angle_info_y + 240), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.7, (0, 0, 255), 2)
                last_send_time = current_time
    
    # Границы поиска
    cv2.line(frame, (Xmin, 0), (Xmin, frame_height), (0, 0, 255), 2)
    cv2.line(frame, (Xmax, 0), (Xmax, frame_height), (0, 0, 255), 2)
    cv2.line(frame, (0, Ymin), (frame_width, Ymin), (0, 0, 255), 2)
    cv2.line(frame, (0, Ymax), (frame_width, Ymax), (0, 0, 255), 2)
    
    # Целевые углы
    for color_name, corner in TARGET_CORNERS.items():
        if color_name in ["Red", "Blue", "Dark Green", "Green"]:
            if color_name == "Red":
                marker_color = (0, 0, 255)
            elif color_name == "Blue":
                marker_color = (255, 0, 0)
            elif color_name == "Dark Green":
                marker_color = (0, 128, 0)
            else:
                marker_color = (0, 255, 0)
            
            cv2.circle(frame, corner, 10, marker_color, 2)
            cv2.putText(frame, f"{color_name}", (corner[0] + 20, corner[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, marker_color, 1)
    
    # Информация о системе
    y_pos = 50
    objects_info_sorted = sorted(objects_info, key=lambda x: x[3] if len(x) > 3 else 0, reverse=True)
        
    for obj in objects_info_sorted:
        color_name = obj[2]
        coords = obj[1]
        area = obj[3] if len(obj) > 3 else 0
        text = f"{color_name}: {coords}, Area: {int(area)}"
        cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_pos += 20
    
    # Статусная информация внизу
    cv2.putText(frame, f"YELLOW THRESH: {MIN_BIG_AREA_YELLOW}", (10, frame_height - 150), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(frame, f"PURPLE THRESH: {MIN_BIG_AREA_PURPLE}", (10, frame_height - 130), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    cv2.putText(frame, f"CAPTURE DIST: {ITEM_CAPTURE_DISTANCE}px", (10, frame_height - 110), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(frame, f"OBSTACLE RADIUS: {OBSTACLE_AVOIDANCE_RADIUS}px", (10, frame_height - 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    
    if optimal_path:
        cv2.putText(frame, f"ROUTE POINTS: {len(optimal_path)}", (10, frame_height - 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"CURRENT TARGET: {current_target_index}", (10, frame_height - 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"CAPTURED: {len(captured_items)}", (10, frame_height - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255) if captured_items else (200, 200, 200), 1)
        if route_completed:
            cv2.putText(frame, "STATUS: COMPLETED", (10, frame_height - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Отображение кадров
    cv2.imshow("Simple Path Following", frame)
    
    # Управление с клавиатуры
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('y'):
        MIN_BIG_AREA_YELLOW += 10
        print(f"MIN_BIG_AREA_YELLOW: {MIN_BIG_AREA_YELLOW}")
    elif key == ord('u'):
        MIN_BIG_AREA_YELLOW = max(10, MIN_BIG_AREA_YELLOW - 10)
        print(f"MIN_BIG_AREA_YELLOW: {MIN_BIG_AREA_YELLOW}")
    elif key == ord('p'):
        MIN_BIG_AREA_PURPLE += 10
        print(f"MIN_BIG_AREA_PURPLE: {MIN_BIG_AREA_PURPLE}")
    elif key == ord('o'):
        MIN_BIG_AREA_PURPLE = max(10, MIN_BIG_AREA_PURPLE - 10)
        print(f"MIN_BIG_AREA_PURPLE: {MIN_BIG_AREA_PURPLE}")
    elif key == ord('c'):
        ITEM_CAPTURE_DISTANCE += 5
        print(f"CAPTURE DISTANCE: {ITEM_CAPTURE_DISTANCE}px")
    elif key == ord('v'):
        ITEM_CAPTURE_DISTANCE = max(10, ITEM_CAPTURE_DISTANCE - 5)
        print(f"CAPTURE DISTANCE: {ITEM_CAPTURE_DISTANCE}px")
    elif key == ord('r'):
        # Сброс маршрута
        optimal_path = None
        captured_items = []
        current_target_index = 1
        route_completed = False
        print("Маршрут сброшен")
    elif key == ord(' '):
        # Принудительный захват текущей точки (для отладки)
        if robot_front_center and optimal_path and current_target_index < len(optimal_path):
            current_target = optimal_path[current_target_index]
            if current_target not in captured_items:
                captured_items.append(current_target)
                current_target_index += 1
                print(f"Принудительный захват точки {current_target_index-1}")

# Отправляем команду остановки
try:
    dataToSend = 0  # Стоп
    bytesToSend = pickle.dumps(dataToSend)
    client.send(bytesToSend)
    print("Sent STOP command to robot")
except:
    pass

print('Программа завершена')
print(f'Захвачено предметов: {len(captured_items)}')
print(f'Маршрут завершён: {"ДА" if route_completed else "НЕТ"}')

# Освобождаем ресурсы
vid.release()
cv2.destroyAllWindows()
client.close()
mySocket.close()
