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

# Диапазоны цветов в HSV ХОРОШИЕ!!!!!!!!!
# Калиброванные параметры жёлтого
yellow_LOWER_COLOR = np.array([15, 103, 138])
yellow_UPPER_COLOR = np.array([100, 255, 255])

# Параметры фиолетового
purple_LOWER_COLOR = np.array([129, 45, 95])
purple_UPPER_COLOR = np.array([156, 156, 162])

# САМЫЕ КРУТЫЕ ЦВЕТА - НЕ ТРОГАТь!!!!!!!!!!!
green_LOWER_COLOR = np.array([45, 45, 60])
green_UPPER_COLOR = np.array([95, 255, 255])

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
Xmin = 100
Xmax = 500

# границы поиска по y
Ymin = 0
Ymax = 425

# Пороги площадей для больших объектов
MIN_BIG_AREA_YELLOW = 250  # Минимальная площадь для больших жёлтых пятен
MIN_BIG_AREA_PURPLE = 250  # Минимальная площадь для больших фиолетовых пятен
MAX_SMALL_AREA = 50  # Максимальная площадь для маленьких объектов

# Целевые углы в зависимости от цвета объектов
# Формат: (x, y) координаты угла кадра
TARGET_CORNERS = {
    "Red": (100, 50),           # Левый верхний угол (с небольшим отступом)
    "Blue": (100, 470),         # Левый нижний угол
    "Dark Green": (560, 50),   # Правый верхний угол
    "Green": (560, 470),       # Правый нижний угол
    "Yellow": (320, 240),      # Центр (по умолчанию для жёлтых)
    "Purple": (320, 240)       # Центр (по умолчанию для фиолетовых)
}

# Параметры для избегания препятствий
OBSTACLE_AVOIDANCE_RADIUS = 25  # Радиус безопасности вокруг объектов (пиксели)
PATH_SMOOTHING = True  # Включить сглаживание пути
SINGLE_PATH_MODE = True  # Режим одного стабильного маршрута

# Глобальные переменные для углов
target_angle = None  # Угол от жёлтого к первой точке маршрута
current_angle = None  # Текущий угол робота (от жёлтого к фиолетовому)
angle_difference = None  # Разница углов
robot_aligned = False  # Флаг выравнивания робота

# Глобальная переменная для хранения стабильного маршрута
stable_optimal_path = None
stable_closest_color_name = None
stable_obstacles = []

# глобальный флаг для работы робота
move_left = False

# Для управления частотой отправки команд
last_send_time = 0
SEND_INTERVAL = 0.05  # 50 мс между отправками команд

# Добавлено: для стабильности направления поворота
last_turn_direction = None  # Последнее направление поворота
TURN_DIRECTION_HYSTERESIS = 5.0  # Гистерезис в градусах для стабильности

# Добавлено: для фиксации маршрута после захвата предмета
route_locked = False  # Флаг фиксации маршрута
locked_optimal_path = None  # Зафиксированный маршрут
locked_closest_color_name = None  # Цвет зафиксированного маршрута
locked_obstacles = []  # Препятствия в момент фиксации
ITEM_CAPTURE_DISTANCE = 30  # Расстояние для определения захвата предмета (пиксели)
FINAL_POINT_DISTANCE = 10  # Расстояние до конечной точки для завершения (пиксели)
captured_items = []  # Список захваченных предметов
current_target_index = 1  # Текущий индекс целевой точки в маршруте (начинается с 1, т.к. 0 - старт)

def calculate_distance(point1, point2):
    """Вычисляет Евклидово расстояние между двумя точками"""
    return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def calculate_angle(point1, point2):
    """Вычисляет угол между горизонталью и линией point1->point2 в градусах (0-360)"""
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    angle_rad = atan2(dy, dx)  # Угол в радианах (-π до π)
    angle_deg = degrees(angle_rad)  # Угол в градусах (-180 до 180)
    
    # Преобразуем к диапазону 0-360°
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg

def normalize_angle(angle):
    """Нормализует угол к диапазону 0-360°"""
    angle %= 360
    if angle < 0:
        angle += 360
    return angle

def angle_difference_deg(angle1, angle2):
    """Вычисляет минимальную разницу между двумя углами в градусах (0-180) и определяет направление поворота"""
    global move_left, last_turn_direction
    
    # Вычисляем разницу углов с учётом круговой природы
    diff = (angle2 - angle1) % 360
    
    # Определяем кратчайший путь поворота
    if diff > 180:
        # Если разница больше 180°, кратчайший путь - в другую сторону
        angle_diff = 360 - diff
        turn_left = False  # Поворот налево (против часовой стрелки) короче
    else:
        angle_diff = diff
        turn_left = True  # Поворот направо (по часовой стрелке) короче
    
    # Применяем гистерезис для стабильности направления
    # Если направление уже было определено и разница углов небольшая,
    # сохраняем прежнее направление
    if last_turn_direction is not None and angle_diff < TURN_DIRECTION_HYSTERESIS:
        if last_turn_direction == "left":
            turn_left = True
        else:
            turn_left = False
    else:
        # Обновляем последнее направление только если разница значительная
        if angle_diff >= TURN_DIRECTION_HYSTERESIS:
            last_turn_direction = "left" if turn_left else "right"
    
    # Устанавливаем глобальный флаг
    move_left = turn_left
    
    return angle_diff

def line_intersects_circle(p1, p2, center, radius):
    """Проверяет, пересекает ли линия p1-p2 круг с центром center и радиусом radius"""
    # Вектор от p1 до p2
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    # Вектор от p1 до центра круга
    fx = center[0] - p1[0]
    fy = center[1] - p1[1]
    
    # Длина отрезка
    segment_length_squared = dx*dx + dy*dy
    
    # Параметр проекции точки на линию
    if segment_length_squared > 0:
        t = max(0, min(1, (fx*dx + fy*dy) / segment_length_squared))
    else:
        t = 0
    
    # Ближайшая точка на отрезке к центру круга
    closest_x = p1[0] + t * dx
    closest_y = p1[1] + t * dy
    
    # Расстояние от ближайшей точки до центра круга
    distance = sqrt((center[0] - closest_x)**2 + (center[1] - closest_y)**2)
    
    return distance <= radius

def find_best_avoidance_side(start, goal, obstacle, obstacle_radius, other_obstacles):
    """Находит лучшую сторону для обхода препятствия (детерминированно)"""
    # Вычисляем вектор направления
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    
    # Нормализуем
    length = sqrt(dx*dx + dy*dy)
    if length > 0:
        dx /= length
        dy /= length
    
    # Перпендикулярные векторы
    perp1 = (-dy, dx)   # Левая сторона
    perp2 = (dy, -dx)   # Правая сторона
    
    # Оцениваем обе стороны
    left_point = (
        int(obstacle[0] + perp1[0] * obstacle_radius * 1.5),
        int(obstacle[1] + perp1[1] * obstacle_radius * 1.5)
    )
    
    right_point = (
        int(obstacle[0] + perp2[0] * obstacle_radius * 1.5),
        int(obstacle[1] + perp2[1] * obstacle_radius * 1.5)
    )
    
    # Критерии выбора (детерминированные):
    # 1. Ближе к цели
    left_to_goal = calculate_distance(left_point, goal)
    right_to_goal = calculate_distance(right_point, goal)
    
    # 2. Меньше пересечений с другими препятствиями
    left_clear = True
    right_clear = True
    
    for other_obstacle in other_obstacles:
        if calculate_distance(left_point, other_obstacle) < obstacle_radius:
            left_clear = False
        if calculate_distance(right_point, other_obstacle) < obstacle_radius:
            right_clear = False
    
    # 3. Приоритет по X координате (для стабильности)
    # Если препятствие слева от линии - обходим справа, и наоборот
    
    # Определяем положение препятствия относительно линии start-goal
    # Используем векторное произведение для определения стороны
    cross_product = (goal[0] - start[0]) * (obstacle[1] - start[1]) - \
                    (goal[1] - start[1]) * (obstacle[0] - start[0])
    
    # Если cross_product > 0 - препятствие слева от линии
    # Если cross_product < 0 - препятствие справа от линии
    
    # Детерминированное правило:
    # Если препятствие слева от линии - обходим справа (более безопасно)
    # Если препятствие справа от линии - обходим слева
    if cross_product > 0:
        # Препятствие слева - обходим справа
        return "right", right_point
    else:
        # Препятствие справа - обходим слева
        return "left", left_point
    
    # Если правило выше не сработало, используем другие критерии
    if left_clear and not right_clear:
        return "left", left_point
    elif right_clear and not left_clear:
        return "right", right_point
    elif left_to_goal < right_to_goal:
        return "left", left_point
    else:
        return "right", right_point

def find_path_with_obstacle_avoidance(start, goal, obstacles, obstacle_radius):
    """
    Находит путь от start до goal с обходом препятствий
    Всегда выбирает одну и ту же сторону обхода для стабильности
    """
    # Если нет препятствий или они далеко, возвращаем прямой путь
    if not obstacles:
        return [start, goal]
    
    # Проверяем, пересекает ли прямая линия какое-либо препятствие
    direct_path_clear = True
    blocking_obstacles = []
    
    for obstacle in obstacles:
        if line_intersects_circle(start, goal, obstacle, obstacle_radius):
            direct_path_clear = False
            blocking_obstacles.append(obstacle)
    
    # Если путь свободен, возвращаем прямой маршрут
    if direct_path_clear:
        return [start, goal]
    
    # Сортируем препятствия по расстоянию от start (для детерминированного порядка)
    blocking_obstacles.sort(key=lambda obs: calculate_distance(start, obs))
    
    # Строим обходной маршрут
    path_points = [start]
    current_pos = start
    
    # Для каждого препятствия добавляем точку обхода
    for i, obstacle in enumerate(blocking_obstacles):
        # Оцениваем, нужно ли ещё обходить это препятствие
        # (возможно, мы уже его обошли предыдущими точками)
        
        # Проверяем, не пересекает ли текущий путь до цели это препятствие
        if not line_intersects_circle(current_pos, goal, obstacle, obstacle_radius):
            continue
        
        # Находим лучшую точку обхода (детерминированно)
        other_obstacles = [o for o in blocking_obstacles if o != obstacle]
        side, avoid_point = find_best_avoidance_side(current_pos, goal, obstacle, obstacle_radius, other_obstacles)
        
        # Проверяем, что точка обхода валидна
        point_valid = True
        for obs in obstacles:
            if calculate_distance(avoid_point, obs) < obstacle_radius * 0.8:
                point_valid = False
                break
        
        if point_valid:
            # Добавляем промежуточную точку ближе к препятствию для плавности
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
                
                # Проверяем, не пересекает ли подход к точке препятствия
                approach_clear = True
                for obs in obstacles:
                    if line_intersects_circle(current_pos, approach_point, obs, obstacle_radius):
                        approach_clear = False
                        break
                
                if approach_clear:
                    path_points.append(approach_point)
                    current_pos = approach_point
            
            # Добавляем точку обхода
            path_points.append(avoid_point)
            current_pos = avoid_point
    
    # Добавляем конечную точку
    path_points.append(goal)
    
    # Сглаживаем путь (если включено)
    if PATH_SMOOTHING and len(path_points) > 2:
        path_points = smooth_path(path_points, obstacles, obstacle_radius)
    
    return path_points

def smooth_path(path, obstacles, obstacle_radius):
    """Сглаживает путь, убирая лишние точки, но сохраняя обход препятствий"""
    if len(path) <= 2:
        return path
    
    smoothed_path = [path[0]]
    
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        best_j = i + 1
        
        while j > i + 1:
            # Проверяем, можно ли соединить точку i с точкой j напрямую
            path_clear = True
            for obstacle in obstacles:
                if line_intersects_circle(path[i], path[j], obstacle, obstacle_radius):
                    path_clear = False
                    break
            
            if path_clear:
                best_j = j
                break
            j -= 1
        
        # Добавляем следующую точку
        if best_j > i + 1:
            # Можно соединить напрямую через несколько точек
            smoothed_path.append(path[best_j])
            i = best_j
        else:
            # Нельзя соединить напрямую, добавляем следующую точку
            smoothed_path.append(path[i + 1])
            i += 1
    
    return smoothed_path

def find_optimal_path_with_avoidance(start_point, points, end_point, all_obstacles, obstacle_radius):
    """
    Находит оптимальный путь с обходом препятствий
    Всегда строит один и тот же маршрут для одинаковых входных данных
    """
    if not points:
        return [start_point, end_point]
    
    # Сортируем точки по координатам (для детерминированного порядка)
    points_sorted = sorted(points, key=lambda p: (p[0], p[1]))
    
    # Жадный алгоритм с фиксированным начальным выбором
    full_path = [start_point]
    unvisited = points_sorted.copy()
    current = start_point
    
    while unvisited:
        # Находим ближайшую точку (жадный, но стабильный)
        nearest = None
        min_dist = float('inf')
        
        for point in unvisited:
            dist = calculate_distance(current, point)
            if dist < min_dist:
                min_dist = dist
                nearest = point
        
        if nearest:
            # Находим путь до ближайшей точки с обходом препятствий
            segment_path = find_path_with_obstacle_avoidance(current, nearest, all_obstacles, obstacle_radius)
            
            # Пропускаем первую точку (она уже есть в full_path)
            if len(segment_path) > 1:
                full_path.extend(segment_path[1:])
            
            current = nearest
            unvisited.remove(nearest)
    
    # Добавляем путь до конечной точки
    final_segment = find_path_with_obstacle_avoidance(current, end_point, all_obstacles, obstacle_radius)
    if len(final_segment) > 1:
        full_path.extend(final_segment[1:])
    
    return full_path

def find_optimal_path(start_point, points, end_point):
    """
    Находит оптимальный порядок обхода точек (упрощённый алгоритм TSP)
    для минимизации общей длины пути от start_point через все points до end_point
    
    Использует алгоритм ближайшего соседа (greedy) для скорости
    """
    if not points:
        return [start_point, end_point]
    
    # Для небольшого количества точек можно использовать полный перебор
    if len(points) <= 6:
        return find_optimal_path_bruteforce(start_point, points, end_point)
    else:
        # Для большего количества точек используем жадный алгоритм
        return find_optimal_path_greedy(start_point, points, end_point)

def find_optimal_path_bruteforce(start_point, points, end_point):
    """Полный перебор всех возможных путей (точное решение для небольшого N)"""
    if not points:
        return [start_point, end_point]
    
    best_path = None
    best_length = inf
    
    # Генерируем все перестановки точек
    for perm in itertools.permutations(points):
        # Создаем полный путь: start -> точки -> end
        current_path = [start_point] + list(perm) + [end_point]
        
        # Вычисляем общую длину пути
        current_length = 0
        for i in range(len(current_path) - 1):
            current_length += calculate_distance(current_path[i], current_path[i + 1])
        
        # Обновляем лучший путь, если нашли короче
        if current_length < best_length:
            best_length = current_length
            best_path = current_path
    
    return best_path

def find_optimal_path_greedy(start_point, points, end_point):
    """Жадный алгоритм: всегда идём к ближайшей ещё не посещённой точке"""
    if not points:
        return [start_point, end_point]
    
    # Копируем список точек
    unvisited = points.copy()
    path = [start_point]
    current = start_point
    
    # Последовательно выбираем ближайшую точку
    while unvisited:
        # Находим ближайшую не посещённую точку
        nearest = min(unvisited, key=lambda p: calculate_distance(current, p))
        path.append(nearest)
        unvisited.remove(nearest)
        current = nearest
    
    # Добавляем конечную точку
    path.append(end_point)
    
    return path

def calculate_total_path_length(path):
    """Вычисляет общую длину пути"""
    if len(path) < 2:
        return 0
    
    total = 0
    for i in range(len(path) - 1):
        total += calculate_distance(path[i], path[i + 1])
    
    return total

def draw_obstacles(frame, obstacles, radius, color=(128, 128, 128)):
    """Рисует препятствия на кадре"""
    for obstacle in obstacles:
        cv2.circle(frame, obstacle, radius, color, 2)
        cv2.circle(frame, obstacle, 2, color, -1)

def should_update_stable_path(current_color_name, current_obstacles):
    """Определяет, нужно ли обновлять стабильный маршрут"""
    global stable_closest_color_name, stable_obstacles
    
    if stable_optimal_path is None:
        return True
    
    # Проверяем, изменился ли цвет
    if current_color_name != stable_closest_color_name:
        return True
    
    # Проверяем, изменились ли препятствия (количество или положение)
    if len(current_obstacles) != len(stable_obstacles):
        return True
    
    # Проверяем положение препятствий (с допуском)
    for i, (obs1, obs2) in enumerate(zip(sorted(current_obstacles), sorted(stable_obstacles))):
        if calculate_distance(obs1, obs2) > 20:  # Допуск 20 пикселей
            return True
    
    return False

def check_item_captured(robot_front_center, current_optimal_path, captured_items):
    """Проверяет, был ли захвачен предмет из маршрута"""
    global current_target_index
    
    if robot_front_center is None or current_optimal_path is None or len(current_optimal_path) < 2:
        return False
    
    # Проверяем текущую целевую точку (если она в пределах индексов)
    if current_target_index < len(current_optimal_path) - 1:  # -1 потому что последняя точка - конечная
        current_target = current_optimal_path[current_target_index]
        
        # Если точка ещё не была захвачена и робот достаточно близко к ней
        if current_target not in captured_items and calculate_distance(robot_front_center, current_target) < ITEM_CAPTURE_DISTANCE:
            captured_items.append(current_target)
            print(f"Предмет {current_target_index} захвачен!")
            
            # Переходим к следующей точке маршрута
            current_target_index += 1
            print(f"Переход к точке {current_target_index}")
            
            return True
    
    return False

def check_final_point_reached(robot_front_center, current_optimal_path):
    """Проверяет, достиг ли робот конечной точки"""
    global current_target_index
    
    if robot_front_center is None or current_optimal_path is None or len(current_optimal_path) < 2:
        return False
    
    # Получаем конечную точку маршрута
    final_point = current_optimal_path[-1]
    
    # Проверяем расстояние до конечной точки
    if calculate_distance(robot_front_center, final_point) < FINAL_POINT_DISTANCE:
        # Сбрасываем индекс целевой точки
        current_target_index = 1
        return True
    
    return False

def find_closest_point_to_target(robot_front_center, target_objects, locked_color_name):
    """Находит ближайшую точку к роботу из объектов заданного цвета (для незафиксированного маршрута)"""
    if not target_objects:
        return None, None, None
    
    closest_obj = None
    min_distance = float('inf')
    
    for obj in target_objects:
        obj_center = obj[1]
        distance = calculate_distance(robot_front_center, obj_center)
        if distance < min_distance:
            min_distance = distance
            closest_obj = obj
    
    if closest_obj:
        return closest_obj[2], closest_obj[1], closest_obj
    else:
        return None, None, None

def get_next_target_point(robot_front_center, optimal_path, captured_items):
    """Получает следующую целевую точку в маршруте"""
    global current_target_index
    
    if optimal_path is None or len(optimal_path) < 2:
        return None
    
    # Если робот ещё не начал движение по маршруту или текущая точка захвачена
    if current_target_index < len(optimal_path):
        # Получаем текущую целевую точку
        target_point = optimal_path[current_target_index]
        
        # Если текущая точка уже захвачена, переходим к следующей
        while current_target_index < len(optimal_path) - 1 and target_point in captured_items:
            current_target_index += 1
            target_point = optimal_path[current_target_index]
        
        return target_point
    
    return optimal_path[-1]  # Возвращаем конечную точку, если дошли до конца

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
    robot_front_center = None  # Перед (фиолетовый)
    robot_back_center = None   # Зад (жёлтый)
    
    # Переменные для хранения информации о роботе
    robot_back_area = 0
    robot_front_area = 0
    
    # Списки для хранения всех объектов (включая препятствия)
    all_objects_centers = []
    all_objects_info = []

    for mask, color, color_name in zip(masks, colors, color_names):
        # Обнаружение контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Аппроксимируем контур
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Обводим контур любой формы
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            
            # Проверяем границы по X
            if Xmin <= x <= Xmax and Ymin <= y <= Ymax:
                center_x, center_y = x + w // 2, y + h // 2
                
                # Сохраняем информацию о всех объектах
                all_objects_centers.append((center_x, center_y))
                
                # Обработка зелёных объектов
                if color == (0, 255, 0):  # Зелёные объекты
                    if area > 180:  # зелёный куб (тёмно-зелёный)
                        cv2.drawContours(frame, [approx], 0, (0, 128, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 128, 0), -1)
                        objects_info.append([(0, 128, 0), (center_x, center_y), "Dark Green", area])
                        all_objects_info.append([(0, 128, 0), (center_x, center_y), "Dark Green", area])
                    elif area > MAX_SMALL_AREA:  # Маленькая зелёная призма
                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                        objects_info.append([(0, 255, 0), (center_x, center_y), "Green", area])
                        all_objects_info.append([(0, 255, 0), (center_x, center_y), "Green", area])
                
                # Обработка ЖЁЛТЫХ объектов с поиском больших пятен
                elif color == (0, 255, 255):  # Жёлтые объекты
                    if area > MIN_BIG_AREA_YELLOW:  # Большое жёлтое пятно (зад робота)
                        cv2.drawContours(frame, [approx], 0, color, 3)  # Более толстый контур
                        cv2.circle(frame, (center_x, center_y), 8, color, -1)  # Большая точка
                        # Сохраняем центр зада робота
                        robot_back_center = (center_x, center_y)
                        robot_back_area = area
                        
                        # Добавляем текст с площадью
                        cv2.putText(frame, f"BIG Yellow: {int(area)}", 
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.6, color, 2)
                        objects_info.append([color, (center_x, center_y), "BIG Yellow", area])
                        all_objects_info.append([color, (center_x, center_y), "BIG Yellow", area])
                    elif area > 10:  # Маленькие жёлтые объекты
                        cv2.drawContours(frame, [approx], 0, color, 2)
                        cv2.circle(frame, (center_x, center_y), 5, color, -1)
                        objects_info.append([color, (center_x, center_y), "Yellow", area])
                        all_objects_info.append([color, (center_x, center_y), "Yellow", area])
                
                # Обработка ФИОЛЕТОВЫХ объектов с поиском больших пятен
                elif color == (255, 0, 255):  # Фиолетовые объекты
                    if area > MIN_BIG_AREA_PURPLE:  # Большое фиолетовое пятно (перед робота)
                        cv2.drawContours(frame, [approx], 0, color, 3)  # Более толстый контур
                        cv2.circle(frame, (center_x, center_y), 8, color, -1)  # Большая точка
                        # Сохраняем центр переда робота
                        robot_front_center = (center_x, center_y)
                        robot_front_area = area
                        
                        # Добавляем текст с площадью
                        cv2.putText(frame, f"BIG Purple: {int(area)}", 
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.6, color, 2)
                        objects_info.append([color, (center_x, center_y), "BIG Purple", area])
                        all_objects_info.append([color, (center_x, center_y), "BIG Purple", area])
                    elif area > 10:  # Маленькие фиолетовые объекты
                        cv2.drawContours(frame, [approx], 0, color, 2)
                        cv2.circle(frame, (center_x, center_y), 5, color, -1)
                        objects_info.append([color, (center_x, center_y), "Purple", area])
                        all_objects_info.append([color, (center_x, center_y), "Purple", area])
                
                # Обработка остальных цветов (синий, красный)
                elif color == (255, 0, 0) or color == (0, 0, 255):  # Синие и красные объекты
                    if area > 10 and area < 250:
                        cv2.drawContours(frame, [approx], 0, color, 2)
                        cv2.circle(frame, (center_x, center_y), 5, color, -1)
                        
                        # Добавляем имя цвета в зависимости от BGR значения
                        if color == (255, 0, 0):
                            obj_color_name = "Blue"
                        elif color == (0, 0, 255):
                            obj_color_name = "Red"
                        else:
                            obj_color_name = color_name
                        
                        objects_info.append([color, (center_x, center_y), obj_color_name, area])
                        all_objects_info.append([color, (center_x, center_y), obj_color_name, area])

    # Переменные для маршрута
    closest_point = None
    target_corner = None
    optimal_path = None
    current_closest_color_name = None
    
    # Если найдены оба центра робота, находим ближайший объект и строим ОПТИМАЛЬНУЮ ломаную
    if robot_front_center and robot_back_center and len(objects_info) > 0:
        # Отделяем объекты от робота (исключаем BIG Purple и BIG Yellow)
        target_objects = []
        for obj in objects_info:
            obj_color_name = obj[2]
            # Исключаем части робота и маленькие фиолетовые/жёлтые
            if obj_color_name not in ["BIG Purple", "BIG Yellow", "Purple", "Yellow"]:
                target_objects.append(obj)
        
        if target_objects:
            # ЕСЛИ МАРШРУТ ЗАФИКСИРОВАН - ИСПОЛЬЗУЕМ ЗАФИКСИРОВАННЫЙ ЦВЕТ
            if route_locked and locked_closest_color_name:
                current_closest_color_name = locked_closest_color_name
                closest_obj = None
                # Ищем объект зафиксированного цвета (ближайший к роботу)
                for obj in target_objects:
                    if obj[2] == locked_closest_color_name:
                        obj_center = obj[1]
                        distance = calculate_distance(robot_front_center, obj_center)
                        if closest_obj is None or distance < calculate_distance(robot_front_center, closest_obj[1]):
                            closest_obj = obj
                
                if closest_obj:
                    closest_center = closest_obj[1]
                    closest_point = closest_center
            else:
                # ЕСЛИ МАРШРУТ НЕ ЗАФИКСИРОВАН - ИЩЕМ БЛИЖАЙШИЙ ОБЪЕКТ
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
            
            if closest_obj:
                # Если маршрут не зафиксирован, обновляем текущий цвет
                if not route_locked and current_closest_color_name is None:
                    current_closest_color_name = closest_obj[2]
                
                # Отображаем информацию о ближайшем объекте
                cv2.putText(frame, f"Closest: {current_closest_color_name}", 
                           (frame_width - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
                if closest_center:
                    cv2.putText(frame, f"Dist: {int(calculate_distance(robot_front_center, closest_center))} px", 
                               (frame_width - 200, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.6, (255, 255, 255), 2)
                
                # Находим все объекты того же цвета
                same_color_objects = []
                same_color_info = []
                for obj in target_objects:
                    if obj[2] == current_closest_color_name:
                        same_color_objects.append(obj[1])
                        same_color_info.append(obj)
                
                # Если есть другие объекты того же цвета
                if same_color_objects:
                    # Получаем целевой угол для этого цвета
                    target_corner = TARGET_CORNERS.get(current_closest_color_name, 
                                                      (frame_width//2, frame_height//2))
                    
                    # Формируем список всех точек этого цвета
                    all_color_points = same_color_objects.copy()
                    
                    # Формируем список препятствий (все объекты НЕ этого цвета)
                    obstacle_centers = []
                    for obj_info in all_objects_info:
                        obj_center = obj_info[1]
                        obj_name = obj_info[2]
                        
                        # Исключаем робота и целевые объекты этого цвета
                        if obj_name not in ["BIG Purple", "BIG Yellow", current_closest_color_name, "Purple", "Yellow"]:
                            # Также исключаем объекты, которые являются целевыми точками
                            if obj_center not in all_color_points:
                                obstacle_centers.append(obj_center)
                    
                    # Рисуем препятствия
                    draw_obstacles(frame, obstacle_centers, OBSTACLE_AVOIDANCE_RADIUS, (128, 128, 128))
                    
                    # ЕСЛИ МАРШРУТ НЕ ЗАФИКСИРОВАН - ПРОВЕРЯЕМ ОБНОВЛЕНИЕ
                    if not route_locked:
                        if SINGLE_PATH_MODE:
                            if should_update_stable_path(current_closest_color_name, obstacle_centers):
                                # Пересчитываем маршрут
                                stable_optimal_path = find_optimal_path_with_avoidance(
                                    robot_front_center, 
                                    all_color_points, 
                                    target_corner,
                                    obstacle_centers,
                                    OBSTACLE_AVOIDANCE_RADIUS
                                )
                                stable_closest_color_name = current_closest_color_name
                                stable_obstacles = obstacle_centers.copy()
                                # Сбрасываем индекс при новом маршруте
                                current_target_index = 1
                            
                            # Используем стабильный маршрут
                            optimal_path = stable_optimal_path
                        else:
                            # Режим пересчёта каждого кадра
                            optimal_path = find_optimal_path_with_avoidance(
                                robot_front_center, 
                                all_color_points, 
                                target_corner,
                                obstacle_centers,
                                OBSTACLE_AVOIDANCE_RADIUS
                            )
                            # Сбрасываем индекс при новом маршруте
                            current_target_index = 1
                    else:
                        # ЕСЛИ МАРШРУТ ЗАФИКСИРОВАН - ИСПОЛЬЗУЕМ ЗАФИКСИРОВАННЫЙ
                        if locked_optimal_path:
                            optimal_path = locked_optimal_path
                            # Используем препятствия из момента фиксации
                            obstacle_centers = locked_obstacles.copy()
                    
                    # Проверяем захват предмета (если маршрут ещё не зафиксирован)
                    if not route_locked and optimal_path:
                        if check_item_captured(robot_front_center, optimal_path, captured_items):
                            # Фиксируем маршрут после захвата первого предмета
                            route_locked = True
                            locked_optimal_path = optimal_path.copy()
                            locked_closest_color_name = current_closest_color_name
                            locked_obstacles = obstacle_centers.copy()
                            print(f"Маршрут зафиксирован! Захвачен предметов: {len(captured_items)}")
                    
                    # Проверяем достижение конечной точки
                    if route_locked and optimal_path and check_final_point_reached(robot_front_center, optimal_path):
                        # Сбрасываем фиксацию маршрута
                        route_locked = False
                        locked_optimal_path = None
                        locked_closest_color_name = None
                        locked_obstacles = []
                        captured_items = []
                        stable_optimal_path = None  # Сбрасываем стабильный маршрут
                        print("Конечная точка достигнута! Маршрут разблокирован.")
                    
                    if optimal_path:
                        # Получаем следующую целевую точку для робота
                        next_target_point = get_next_target_point(robot_front_center, optimal_path, captured_items)
                        if next_target_point:
                            closest_point = next_target_point
                        
                        # Преобразуем точки в формат для cv2.polylines
                        points_array = np.array(optimal_path, dtype=np.int32)
                        
                        # Рисуем ОПТИМАЛЬНУЮ ломаную линию с обходом препятствий
                        # Цвет зависит от статуса фиксации маршрута
                        if route_locked:
                            cv2.polylines(frame, [points_array], isClosed=False, 
                                        color=(0, 165, 255), thickness=3)  # Оранжевый для зафиксированного
                        else:
                            cv2.polylines(frame, [points_array], isClosed=False, 
                                        color=(0, 255, 0), thickness=3)  # Зелёный для обычного
                        
                        # Вычисляем и отображаем общую длину пути
                        total_length = calculate_total_path_length(optimal_path)
                        cv2.putText(frame, f"Path length: {int(total_length)} px", 
                                   (frame_width - 200, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.6, (0, 255, 0), 2)
                        
                        # Отображаем информацию о препятствиях
                        cv2.putText(frame, f"Obstacles: {len(obstacle_centers)}", 
                                   (frame_width - 200, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.6, (128, 128, 128), 2)
                        
                        # Отображаем режим маршрута
                        path_mode = "STABLE" if SINGLE_PATH_MODE else "DYNAMIC"
                        cv2.putText(frame, f"Path mode: {path_mode}", 
                                   (frame_width - 200, 150), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.6, (0, 255, 255), 2)
                        
                        # Отображаем статус фиксации маршрута
                        if route_locked:
                            cv2.putText(frame, "ROUTE LOCKED", 
                                       (frame_width - 200, 180), cv2.FONT_HERSHEY_SIMPLEX, 
                                       0.7, (0, 165, 255), 2)
                            cv2.putText(frame, f"Captured: {len(captured_items)} items", 
                                       (frame_width - 200, 210), cv2.FONT_HERSHEY_SIMPLEX, 
                                       0.6, (0, 165, 255), 2)
                            cv2.putText(frame, f"Target index: {current_target_index}/{len(optimal_path)-1}", 
                                       (frame_width - 200, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                                       0.6, (0, 165, 255), 2)
                        
                        # Рисуем круги в вершинах ломаной с разными цветами
                        for i, point in enumerate(optimal_path):
                            if i == 0:  # Начало - перед робота
                                cv2.circle(frame, point, 8, (255, 0, 255), -1)  # Фиолетовый
                                cv2.putText(frame, "START", (point[0] + 15, point[1]), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                            elif i == len(optimal_path) - 1:  # Конец - целевой угол
                                cv2.circle(frame, point, 10, (0, 0, 0), -1)  # Чёрный
                                cv2.circle(frame, point, 10, (255, 255, 255), 2)  # Белая обводка
                                cv2.putText(frame, "END", (point[0] + 15, point[1]), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            else:  # Промежуточные точки
                                # Проверяем, является ли точка целевым объектом или точкой обхода
                                is_target_point = point in all_color_points
                                
                                if is_target_point:
                                    # Проверяем, захвачена ли эта точка
                                    if point in captured_items:
                                        # Захваченные предметы - серым цветом
                                        cv2.circle(frame, point, 7, (100, 100, 100), -1)
                                        cv2.putText(frame, "CAPTURED", (point[0] + 15, point[1]), 
                                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                                    else:
                                        # Незахваченные предметы - обычным цветом
                                        for obj_info in same_color_info:
                                            if obj_info[1] == point:
                                                obj_color = obj_info[0]
                                                # Подсвечиваем текущую целевую точку
                                                if i == current_target_index:
                                                    cv2.circle(frame, point, 10, (255, 255, 0), -1)  # Жёлтый для текущей цели
                                                    cv2.circle(frame, point, 10, obj_color, 3)  # Обводка цветом объекта
                                                else:
                                                    cv2.circle(frame, point, 7, obj_color, -1)  # Цвет объекта
                                                    cv2.circle(frame, point, 7, (255, 255, 255), 1)  # Белая обводка
                                                break
                                else:
                                    # Точка обхода (голубой цвет)
                                    cv2.circle(frame, point, 6, (255, 200, 0), -1)  # Голубой
                                    cv2.circle(frame, point, 6, (255, 255, 255), 1)  # Белая обводка
                                
                                # Подписываем номер точки в оптимальном маршруте
                                if point not in captured_items:  # Только для незахваченных
                                    cv2.putText(frame, str(i), (point[0] + 10, point[1]), 
                                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Рисуем линию от робота до ближайшего объекта (красный пунктир)
                if closest_center:
                    cv2.line(frame, robot_front_center, closest_center, 
                            (0, 0, 255), 2, cv2.LINE_AA)

    # ОБРАБОТКА УГЛОВ И ОТПРАВКА КОМАНД РОБОТУ
    if robot_back_center and robot_front_center and closest_point:
        # 1. Рисуем линию от задней части робота до ближайшей точки маршрута (синяя толстая линия)
        cv2.line(frame, robot_back_center, closest_point, 
                (255, 100, 0), 3, cv2.LINE_AA)
        
        # 2. Вычисляем целевой угол (от жёлтого к точке)
        target_angle = calculate_angle(robot_back_center, closest_point)
        
        # 3. Вычисляем текущий угол робота (от жёлтого к фиолетовому)
        current_angle = calculate_angle(robot_back_center, robot_front_center)
        
        # 4. Вычисляем разницу углов и направление поворота
        angle_difference = angle_difference_deg(target_angle, current_angle)
        
        # 5. Проверяем выравнивание (разница ≤ delta_angles_ROBOT_FIGURE)
        robot_aligned = angle_difference <= delta_angles_ROBOT_FIGURE
        
        # 6. Рисуем линию направления робота (от жёлтого к фиолетовому)
        cv2.line(frame, robot_back_center, robot_front_center,
                (255, 255, 255), 2, cv2.LINE_AA)
        
        # 7. Отображаем информацию об углах
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
        
        # 8. Отображаем направление поворота
        direction_text = "LEFT" if move_left else "RIGHT"
        cv2.putText(frame, f"Turn: {direction_text}", 
                   (frame_width - 200, angle_info_y + 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 255, 255), 2)
        
        # 9. Отображаем гистерезис
        cv2.putText(frame, f"Hysteresis: {TURN_DIRECTION_HYSTERESIS}°", 
                   (frame_width - 200, angle_info_y + 120), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (200, 200, 200), 1)
        
        # 10. Отображаем статус фиксации маршрута
        route_status = "LOCKED" if route_locked else "UNLOCKED"
        route_status_color = (0, 165, 255) if route_locked else (0, 255, 0)
        cv2.putText(frame, f"Route: {route_status}", 
                   (frame_width - 200, angle_info_y + 150), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, route_status_color, 2)
        
        # 11. Отображаем количество захваченных предметов
        cv2.putText(frame, f"Captured: {len(captured_items)}", 
                   (frame_width - 200, angle_info_y + 180), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 165, 255) if captured_items else (200, 200, 200), 2)
        
        # 12. Отображаем текущую целевую точку
        if optimal_path and current_target_index < len(optimal_path):
            cv2.putText(frame, f"Target: {current_target_index}/{len(optimal_path)-1}", 
                       (frame_width - 200, angle_info_y + 210), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 0), 2)
        
        # 13. Отправка команд роботу с управлением частотой
        current_time = time.time()
        if current_time - last_send_time > SEND_INTERVAL:
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

    # Проведение вертикальных красных линий
    cv2.line(frame, (Xmin, 0), (Xmin, frame_height), (0, 0, 255), 2)
    cv2.line(frame, (Xmax, 0), (Xmax, frame_height), (0, 0, 255), 2)
    cv2.line(frame, (0, Ymin), (frame_width, Ymin), (0, 0, 255), 2)
    cv2.line(frame, (0, Ymax), (frame_width, Ymax), (0, 0, 255), 2)
    
    # Рисуем целевые углы
    for color_name, corner in TARGET_CORNERS.items():
        if color_name in ["Red", "Blue", "Dark Green", "Green"]:
            # Определяем цвет маркера угла
            if color_name == "Red":
                marker_color = (0, 0, 255)  # Красный
            elif color_name == "Blue":
                marker_color = (255, 0, 0)  # Синий
            elif color_name == "Dark Green":
                marker_color = (0, 128, 0)  # Тёмно-зелёный
            else:  # Green
                marker_color = (0, 255, 0)  # Зелёный
            
            # Рисуем маркер угла
            cv2.circle(frame, corner, 10, marker_color, 2)
            cv2.putText(frame, f"{color_name} TARGET", (corner[0] + 20, corner[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, marker_color, 1)
    
    # Вывод списка на изображение с сортировкой по площади (самые большие сначала)
    y_pos = 50
    # Сортируем объекты по площади (от больших к маленьким)
    objects_info_sorted = sorted(objects_info, key=lambda x: x[3] if len(x) > 3 else 0, reverse=True)
        
    for obj in objects_info_sorted:
        color_name = obj[2]  # Используем сохранённое имя цвета
        coords = obj[1]
        area = obj[3] if len(obj) > 3 else 0
        text = f"{color_name}: {coords}, Area: {int(area)}"
        cv2.putText(frame, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_pos += 20
    
    # Добавляем информацию о порогах больших объектов
    cv2.putText(frame, f"YELLOW BIG AREA: {MIN_BIG_AREA_YELLOW}", (10, frame_height - 150), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(frame, f"PURPLE BIG AREA: {MIN_BIG_AREA_PURPLE}", (10, frame_height - 130), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    
    # Информация о препятствиях
    cv2.putText(frame, f"OBSTACLE RADIUS: {OBSTACLE_AVOIDANCE_RADIUS}px", (10, frame_height - 110), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    
    # Информация о режиме
    mode_text = "SINGLE STABLE PATH" if SINGLE_PATH_MODE else "DYNAMIC PATH"
    cv2.putText(frame, f"MODE: {mode_text}", (10, frame_height - 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255) if SINGLE_PATH_MODE else (255, 255, 0), 1)
    
    # Информация о гистерезисе
    cv2.putText(frame, f"TURN HYSTERESIS: {TURN_DIRECTION_HYSTERESIS}°", (10, frame_height - 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
    
    # Информация о фиксации маршрута
    route_status = "LOCKED" if route_locked else "UNLOCKED"
    route_status_color = (0, 165, 255) if route_locked else (0, 255, 0)
    cv2.putText(frame, f"ROUTE: {route_status}", (10, frame_height - 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, route_status_color, 1)
    
    # Информация о захваченных предметах
    cv2.putText(frame, f"CAPTURED ITEMS: {len(captured_items)}", (10, frame_height - 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255) if captured_items else (200, 200, 200), 1)
    
    # Отображение текущей целевой точки
    if optimal_path and current_target_index < len(optimal_path):
        cv2.putText(frame, f"CURRENT TARGET: {current_target_index}", (10, frame_height - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    # Отображение кадров
    cv2.imshow("robozari 2026 v0.9 - SINGLE STABLE PATH", frame)
    
    # Отображение масок
    cv2.imshow("YELLOW mask", mask_yellow)
    cv2.imshow("PURPLE mask", mask_purple)
    cv2.imshow("GREEN mask", mask_green)
    cv2.imshow("RED mask", mask_red)
    cv2.imshow("BLUE mask", mask_blue)
    
    # Управление с клавиатуры
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    # Управление порогом для жёлтого
    elif key == ord('y'):
        MIN_BIG_AREA_YELLOW += 10
        print(f"MIN_BIG_AREA_YELLOW increased to: {MIN_BIG_AREA_YELLOW}")
    elif key == ord('u'):
        MIN_BIG_AREA_YELLOW = max(10, MIN_BIG_AREA_YELLOW - 10)
        print(f"MIN_BIG_AREA_YELLOW decreased to: {MIN_BIG_AREA_YELLOW}")
    # Управление порогом для фиолетового
    elif key == ord('p'):
        MIN_BIG_AREA_PURPLE += 10
        print(f"MIN_BIG_AREA_PURPLE increased to: {MIN_BIG_AREA_PURPLE}")
    elif key == ord('o'):
        MIN_BIG_AREA_PURPLE = max(10, MIN_BIG_AREA_PURPLE - 10)
        print(f"MIN_BIG_AREA_PURPLE decreased to: {MIN_BIG_AREA_PURPLE}")
    # Управление радиусом препятствий
    elif key == ord('a'):
        OBSTACLE_AVOIDANCE_RADIUS += 5
        stable_optimal_path = None  # Сбрасываем стабильный маршрут
        print(f"OBSTACLE_AVOIDANCE_RADIUS increased to: {OBSTACLE_AVOIDANCE_RADIUS}")
    elif key == ord('d'):
        OBSTACLE_AVOIDANCE_RADIUS = max(10, OBSTACLE_AVOIDANCE_RADIUS - 5)
        stable_optimal_path = None  # Сбрасываем стабильный маршрут
        print(f"OBSTACLE_AVOIDANCE_RADIUS decreased to: {OBSTACLE_AVOIDANCE_RADIUS}")
    # Управление гистерезисом
    elif key == ord('h'):
        TURN_DIRECTION_HYSTERESIS += 1.0
        print(f"TURN_DIRECTION_HYSTERESIS increased to: {TURN_DIRECTION_HYSTERESIS}°")
    elif key == ord('g'):
        TURN_DIRECTION_HYSTERESIS = max(1.0, TURN_DIRECTION_HYSTERESIS - 1.0)
        print(f"TURN_DIRECTION_HYSTERESIS decreased to: {TURN_DIRECTION_HYSTERESIS}°")
    # Переключение режима
    elif key == ord('m'):
        SINGLE_PATH_MODE = not SINGLE_PATH_MODE
        stable_optimal_path = None  # Сбрасываем стабильный маршрут
        mode_text = "SINGLE STABLE PATH" if SINGLE_PATH_MODE else "DYNAMIC PATH"
        print(f"Mode changed to: {mode_text}")
    # Управление частотой отправки команд
    elif key == ord('s'):
        SEND_INTERVAL += 0.01
        print(f"SEND_INTERVAL increased to: {SEND_INTERVAL:.3f}s")
    elif key == ord('f'):
        SEND_INTERVAL = max(0.01, SEND_INTERVAL - 0.01)
        print(f"SEND_INTERVAL decreased to: {SEND_INTERVAL:.3f}s")
    # Принудительная разблокировка маршрута (для отладки)
    elif key == ord('l'):
        if route_locked:
            route_locked = False
            locked_optimal_path = None
            locked_closest_color_name = None
            locked_obstacles = []
            captured_items = []
            current_target_index = 1
            print("Маршрут принудительно разблокирован")
    # Сброс порогов
    elif key == ord('r'):
        MIN_BIG_AREA_YELLOW = 250
        MIN_BIG_AREA_PURPLE = 250
        OBSTACLE_AVOIDANCE_RADIUS = 40
        SEND_INTERVAL = 0.05
        TURN_DIRECTION_HYSTERESIS = 5.0
        route_locked = False
        locked_optimal_path = None
        locked_closest_color_name = None
        locked_obstacles = []
        captured_items = []
        current_target_index = 1
        stable_optimal_path = None  # Сбрасываем стабильный маршрут
        print(f"Reset: YELLOW={MIN_BIG_AREA_YELLOW}, PURPLE={MIN_BIG_AREA_PURPLE}, OBSTACLE_RADIUS={OBSTACLE_AVOIDANCE_RADIUS}, SEND_INTERVAL={SEND_INTERVAL}, HYSTERESIS={TURN_DIRECTION_HYSTERESIS}")

# Отправляем команду остановки при завершении программы
try:
    dataToSend = 0  # Стоп
    bytesToSend = pickle.dumps(dataToSend)
    client.send(bytesToSend)
    print("Sent STOP command to robot")
except:
    pass

print('Программа завершена')
print(f'Финальные пороги: YELLOW={MIN_BIG_AREA_YELLOW}, PURPLE={MIN_BIG_AREA_PURPLE}')
print(f'Радиус препятствий: {OBSTACLE_AVOIDANCE_RADIUS}px')
print(f'Режим: {"SINGLE STABLE PATH" if SINGLE_PATH_MODE else "DYNAMIC PATH"}')
print(f'Интервал отправки команд: {SEND_INTERVAL*1000:.0f}ms')
print(f'Гистерезис направления поворота: {TURN_DIRECTION_HYSTERESIS}°')
print(f'Маршрут зафиксирован: {"ДА" if route_locked else "НЕТ"}')
print(f'Захвачено предметов: {len(captured_items)}')
print(f'Текущая целевая точка: {current_target_index}')
if target_angle is not None:
    print(f'Целевой угол: {target_angle:.1f}°')
if current_angle is not None:
    print(f'Текущий угол: {current_angle:.1f}°')
if angle_difference is not None:
    print(f'Разница углов: {angle_difference:.1f}°')
    print(f'Робот выровнен: {"ДА" if robot_aligned else "НЕТ"}')

# Освобождаем ресурсы
vid.release()
cv2.destroyAllWindows()
client.close()
mySocket.close()
