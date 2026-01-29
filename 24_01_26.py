"""
Серверная часть, её НЕ ТРОГАТЬ

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
from math import sqrt, atan2, degrees, pi

# ==============================
# КОНФИГУРАЦИЯ ПАРАМЕТРОВ
# ==============================

# Диапазоны цветов в HSV (оптимизировано для скорости)
COLOR_RANGES = {
    'yellow': (np.array([15, 103, 138]), np.array([100, 255, 255])),
    'purple': (np.array([110, 60, 60]), np.array([255, 255, 255])),
    'green': (np.array([45, 45, 60]), np.array([100, 255, 255])),
    'red': (np.array([0, 95, 95]), np.array([10, 255, 255])),
    'blue': (np.array([60, 45, 65]), np.array([115, 255, 255]))
}

# Цвета для отрисовки (BGR)
DRAW_COLORS = {
    'Blue': (255, 0, 0),
    'Red': (0, 0, 255),
    'Green': (0, 255, 0),
    'Dark Green': (0, 128, 0),
    'Yellow': (0, 255, 255),
    'Purple': (255, 0, 255)
}

# Параметры системы
CONFIG = {
    'delta_angles_ROBOT_FIGURE': 3.0,
    'Xmin': 70, 'Xmax': 540,
    'Ymin': 0, 'Ymax': 425,
    'botXmin': 0, 'botXmax': 600,
    'MIN_BIG_AREA_YELLOW': 250,
    'MIN_BIG_AREA_PURPLE': 250,
    'MAX_SMALL_AREA': 50,
    'OBSTACLE_AVOIDANCE_RADIUS': 25,
    'SEND_INTERVAL': 0.0005,
    'TURN_DIRECTION_HYSTERESIS': 5.0,
    'ITEM_CAPTURE_DISTANCE': 10,
    'FINAL_POINT_DISTANCE': 15,
    'GRID_MIN_AREA': 180
}

# Целевые углы в зависимости от цвета объектов
TARGET_CORNERS = {
    "Red": (75, 75),           # Левый верхний угол
    "Blue": (65, 320),         # Левый нижний угол
    "Dark Green": (565, 325),  # Правый верхний угол
    "Green": (565, 75),        # Правый нижний угол
    "Yellow": (320, 240),      # Центр
    "Purple": (320, 240)       # Центр
}

# ==============================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ==============================

def calculate_distance(p1, p2):
    """Вычисляет Евклидово расстояние между двумя точками (оптимизировано)"""
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def calculate_angle(p1, p2):
    """Вычисляет угол между горизонталью и линией p1->p2 в градусах (0-360)"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle_deg = degrees(atan2(dy, dx)) % 360
    return angle_deg

def angle_difference_deg(angle1, angle2):
    """Вычисляет минимальную разницу между двумя углами в градусах"""
    diff = (angle2 - angle1) % 360
    return 360 - diff if diff > 180 else diff

def line_intersects_circle(p1, p2, center, radius):
    """Проверяет, пересекает ли линия p1-p2 круг"""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    fx, fy = center[0] - p1[0], center[1] - p1[1]
    
    seg_len_sq = dx*dx + dy*dy
    
    if seg_len_sq > 0:
        t = max(0, min(1, (fx*dx + fy*dy) / seg_len_sq))
    else:
        t = 0
    
    closest_x = p1[0] + t * dx
    closest_y = p1[1] + t * dy
    
    return sqrt((center[0] - closest_x)**2 + (center[1] - closest_y)**2) <= radius

def get_avoidance_point(start, goal, obstacle, radius, side):
    """Получает точку обхода препятствия с указанной стороны"""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    length = sqrt(dx*dx + dy*dy)
    
    if length > 0:
        dx /= length
        dy /= length
    
    # Векторы перпендикуляров
    if side == "left":
        perp = (-dy, dx)
    else:  # "right"
        perp = (dy, -dx)
    
    return (
        int(obstacle[0] + perp[0] * radius * 1.5),
        int(obstacle[1] + perp[1] * radius * 1.5)
    )

def find_path_with_obstacle_avoidance(start, goal, obstacles, radius):
    """Находит путь от start до goal с обходом препятствий"""
    if not obstacles:
        return [start, goal]
    
    blocking_obstacles = []
    for obstacle in obstacles:
        if line_intersects_circle(start, goal, obstacle, radius):
            blocking_obstacles.append(obstacle)
    
    if not blocking_obstacles:
        return [start, goal]
    
    # Сортируем по расстоянию до старта
    blocking_obstacles.sort(key=lambda obs: calculate_distance(start, obs))
    
    path = [start]
    current = start
    
    for obstacle in blocking_obstacles:
        if not line_intersects_circle(current, goal, obstacle, radius):
            continue
        
        # Определяем сторону обхода
        cross = (goal[0] - start[0]) * (obstacle[1] - start[1]) - \
                (goal[1] - start[1]) * (obstacle[0] - start[0])
        side = "right" if cross > 0 else "left"
        
        avoid_point = get_avoidance_point(current, goal, obstacle, radius, side)
        
        # Проверяем валидность точки обхода
        point_valid = all(calculate_distance(avoid_point, obs) >= radius * 0.8 for obs in obstacles)
        
        if point_valid:
            path.append(avoid_point)
            current = avoid_point
    
    path.append(goal)
    return path

# ==============================
# КЛАССЫ ДЛЯ УПРАВЛЕНИЯ СОСТОЯНИЕМ
# ==============================

class RobotController:
    """Управляет состоянием робота и отправкой команд"""
    def __init__(self, client_connection):
        self.client = client_connection
        self.last_send_time = 0
        self.last_turn_direction = None
        self.move_left = False
        self.commands = {
            'stop': 0,
            'forward': 1,
            'right': 2,
            'left': 3,
            'special': 5
        }
    
    def send_command(self, command_name, current_time):
        """Отправляет команду роботу с учетом интервала"""
        if current_time - self.last_send_time < CONFIG['SEND_INTERVAL']:
            if command_name == "special":
                print("command err, data:", command_name, current_time)
            #return False
        
        command_code = self.commands.get(command_name)
        if command_code is not None:
            bytes_to_send = pickle.dumps(command_code)
            self.client.send(bytes_to_send)
            self.last_send_time = current_time
            print("command:", command_name)
            return True
        return False
            
    def update_turn_direction(self, angle_diff, raw_diff, turn_left):
        """Обновляет направление поворота с учетом гистерезиса и явного условия ≥180°"""
        # Явное условие: если raw_diff >= 180, то точно поворачиваем влево
        if raw_diff >= 180:
            self.move_left = True
            self.last_turn_direction = "left"
            return self.move_left
        
        # Для остальных случаев — старая логика с гистерезисом
        if (self.last_turn_direction and 
            angle_diff < CONFIG['TURN_DIRECTION_HYSTERESIS']):
            if self.last_turn_direction == "left":
                self.move_left = True
            else:
                self.move_left = False
        else:
            self.move_left = turn_left
            if angle_diff >= CONFIG['TURN_DIRECTION_HYSTERESIS']:
                self.last_turn_direction = "left" if turn_left else "right"
        
        return self.move_left

class PathPlanner:
    """Планирует и управляет маршрутом робота"""
    def __init__(self):
        self.optimal_path = None
        self.captured_items = []
        self.current_target_index = 1
        self.route_completed = False
    
    def find_optimal_path(self, start, points, end, obstacles):
        """Находит оптимальный путь с обходом препятствий"""
        if not points:
            return [start, end]
        
        # Сортируем точки по координатам для детерминированного маршрута
        points_sorted = sorted(points, key=lambda p: (p[0], p[1]))
        
        full_path = [start]
        unvisited = points_sorted.copy()
        current = start
        
        while unvisited:
            # Находим ближайшую точку
            nearest = min(unvisited, key=lambda p: calculate_distance(current, p))
            
            # Добавляем путь к ближайшей точке
            segment = find_path_with_obstacle_avoidance(
                current, nearest, obstacles, CONFIG['OBSTACLE_AVOIDANCE_RADIUS']
            )
            
            if len(segment) > 1:
                full_path.extend(segment[1:])
            
            current = nearest
            unvisited.remove(nearest)
        
        # Добавляем путь к конечной точке
        final_segment = find_path_with_obstacle_avoidance(
            current, end, obstacles, CONFIG['OBSTACLE_AVOIDANCE_RADIUS']
        )
        
        if len(final_segment) > 1:
            full_path.extend(final_segment[1:])
        
        return full_path
    
    def check_item_captured(self, robot_front):
        """Проверяет захват текущего предмета"""
        if (not robot_front or not self.optimal_path or 
            self.current_target_index >= len(self.optimal_path) - 1):
            return False
        
        current_target = self.optimal_path[self.current_target_index]
        
        if (current_target not in self.captured_items and
            calculate_distance(robot_front, current_target) < CONFIG['ITEM_CAPTURE_DISTANCE']):
            
            self.captured_items.append(current_target)
            self.current_target_index += 1
            
            print(f"✓ Предмет {self.current_target_index-1} захвачен!")
            
            if self.current_target_index < len(self.optimal_path):
                next_target = self.optimal_path[self.current_target_index]
                print(f"→ Переход к точке {self.current_target_index} ({next_target})")
            
            return True
        
        return False
    
    def check_final_point_reached(self, robot_front):
        """Проверяет достижение конечной точки"""
        if (not robot_front or not self.optimal_path or 
            len(self.optimal_path) < 2):
            return False
        
        final_point = self.optimal_path[-1]
        distance = calculate_distance(robot_front, final_point)
        
        if distance < CONFIG['FINAL_POINT_DISTANCE'] and not self.route_completed:
            self.route_completed = True
            return True
        
        return False

class ObjectDetector:
    """Обнаруживает и классифицирует объекты на кадре"""
    def __init__(self):
        self.objects_info = []
        self.all_objects_info = []
        self.robot_front = None
        self.robot_back = None
    
    def process_frame(self, frame, hsv):
        """Обрабатывает кадр и обнаруживает объекты"""
        self.objects_info = []
        self.all_objects_info = []
        self.robot_front = None
        self.robot_back = None
        
        # Создаем маски для всех цветов
        masks = {}
        for color_name, (lower, upper) in COLOR_RANGES.items():
            masks[color_name] = cv2.inRange(hsv, lower, upper)
        
        # Обрабатываем каждый цвет
        color_handlers = {
            'yellow': self._handle_yellow,
            'purple': self._handle_purple,
            'green': self._handle_green,
            'red': self._handle_red_blue,
            'blue': self._handle_red_blue
        }
        
        for color_name, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            handler = color_handlers.get(color_name)
            if handler:
                handler(contours, frame, color_name)
    
    def _handle_yellow(self, contours, frame, color_name):
        """Обрабатывает жёлтые объекты (задняя часть робота)"""
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > CONFIG['MIN_BIG_AREA_YELLOW']:
                x, y, w, h = cv2.boundingRect(contour)
                if CONFIG['botXmin'] <= x <= CONFIG['botXmax']:
                    center = (x + w//2, y + h//2)
                    self.robot_back = center
                    self._draw_object(frame, contour, center, "ROBOT BACK", DRAW_COLORS['Yellow'])
                    self.all_objects_info.append(['Yellow', center, "ROBOT", area])
    
    def _handle_purple(self, contours, frame, color_name):
        """Обрабатывает фиолетовые объекты (передняя часть робота)"""
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > CONFIG['MIN_BIG_AREA_PURPLE']:
                x, y, w, h = cv2.boundingRect(contour)
                if CONFIG['botXmin'] <= x <= CONFIG['botXmax']:
                    center = (x + w//2, y + h//2)
                    self.robot_front = center
                    self._draw_object(frame, contour, center, "ROBOT FRONT", DRAW_COLORS['Purple'])
                    self.all_objects_info.append(['Purple', center, "ROBOT", area])
    
    def _handle_green(self, contours, frame, color_name):
        """Обрабатывает зелёные объекты (цели)"""
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if CONFIG['Xmin'] <= x <= CONFIG['Xmax'] and CONFIG['Ymin'] <= y <= CONFIG['Ymax']:
                area = cv2.contourArea(contour)
                center = (x + w//2, y + h//2)
                
                if area > CONFIG['GRID_MIN_AREA']:
                    color = DRAW_COLORS['Dark Green']
                    obj_name = "Dark Green"
                elif area > CONFIG['MAX_SMALL_AREA']:
                    color = DRAW_COLORS['Green']
                    obj_name = "Green"
                else:
                    continue
                
                self._draw_object(frame, contour, center, obj_name, color)
                self.objects_info.append([color, center, obj_name, area])
                self.all_objects_info.append([color, center, obj_name, area])
    
    def _handle_red_blue(self, contours, frame, color_name):
        """Обрабатывает красные и синие объекты (цели)"""
        color = DRAW_COLORS['Red'] if color_name == 'red' else DRAW_COLORS['Blue']
        obj_name = "Red" if color_name == 'red' else "Blue"
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (CONFIG['Xmin'] <= x <= CONFIG['Xmax'] and 
                CONFIG['Ymin'] <= y <= CONFIG['Ymax']):
                area = cv2.contourArea(contour)
                
                if 10 < area < 250:
                    center = (x + w//2, y + h//2)
                    self._draw_object(frame, contour, center, obj_name, color)
                    self.objects_info.append([color, center, obj_name, area])
                    self.all_objects_info.append([color, center, obj_name, area])
    
    def _draw_object(self, frame, contour, center, label, color):
        """Рисует объект на кадре"""
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        cv2.drawContours(frame, [approx], 0, color, 2)
        cv2.circle(frame, center, 5, color, -1)
        cv2.putText(frame, label, (center[0] - 40, center[1] - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

# ==============================
# ОСНОВНОЙ ЦИКЛ ПРОГРАММЫ
# ==============================

def main():
    # Инициализация компонентов
    robot_controller = RobotController(client)
    path_planner = PathPlanner()
    object_detector = ObjectDetector()
    
    # Инициализация видеозахвата
    vid = cv2.VideoCapture(0)
    if not vid.isOpened():
        print("Ошибка: не удалось открыть камеру")
        return
    
    print("Система инициализирована. Нажмите 'q' для выхода.")
    
    while True:
        ret, frame = vid.read()
        if not ret:
            print("Ошибка чтения кадра")
            break
        
        frame_height, frame_width = frame.shape[:2]
        
        # Преобразование в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Обнаружение объектов
        object_detector.process_frame(frame, hsv)
        
        robot_front = object_detector.robot_front
        robot_back = object_detector.robot_back
        objects_info = object_detector.objects_info
        
        # Основная логика построения маршрута
        if robot_front and robot_back and objects_info:
            # Фильтруем только целевые объекты
            target_objects = [
                obj for obj in objects_info 
                if obj[2] not in ["ROBOT", "Yellow", "Purple"]
            ]
            
            if target_objects:
                # Находим ближайший объект
                closest_obj = min(
                    target_objects, 
                    key=lambda obj: calculate_distance(robot_front, obj[1])
                )
                
                current_color = closest_obj[2]
                target_corner = TARGET_CORNERS.get(current_color, (frame_width//2, frame_height//2))
                
                # Находим все объекты того же цвета
                same_color_objects = [
                    obj[1] for obj in target_objects 
                    if obj[2] == current_color
                ]
                
                # Определяем препятствия
                obstacle_centers = [
                    obj[1] for obj in object_detector.all_objects_info
                    if obj[2] not in ["ROBOT", current_color]
                ]
                
                # Рисуем препятствия
                for center in obstacle_centers:
                    cv2.circle(frame, center, CONFIG['OBSTACLE_AVOIDANCE_RADIUS'], 
                              (128, 128, 128), 2)
                    cv2.circle(frame, center, 2, (128, 128, 128), -1)
                
                # Строим маршрут (если ещё не построен)
                if path_planner.optimal_path is None or path_planner.route_completed:
                    path_planner.optimal_path = path_planner.find_optimal_path(
                        robot_front, same_color_objects, target_corner, obstacle_centers
                    )
                    path_planner.current_target_index = 1
                    path_planner.captured_items = []
                    path_planner.route_completed = False
                    print(f"Построен новый маршрут к {current_color}")
                
                # Проверяем захват предметов
                path_planner.check_item_captured(robot_front)
                
                # Проверяем достижение конечной точки
                if path_planner.check_final_point_reached(robot_front):
                    robot_controller.send_command('special', time.time())
                    print("✓ Маршрут завершён!")
                    robot_controller.send_command('forward', current_time)
                
                # Рисуем и обрабатываем маршрут
                if path_planner.optimal_path:
                    # Рисуем маршрут
                    points_array = np.array(path_planner.optimal_path, dtype=np.int32)
                    cv2.polylines(frame, [points_array], False, (0, 255, 0), 2)
                    
                    # Рисуем точки маршрута
                    for i, point in enumerate(path_planner.optimal_path):
                        if i == 0:  # Старт
                            cv2.circle(frame, point, 8, DRAW_COLORS['Purple'], -1)
                        elif i == len(path_planner.optimal_path) - 1:  # Конец
                            cv2.circle(frame, point, 10, (255, 255, 255), -1)
                        else:  # Промежуточные точки
                            color = (100, 100, 100) if point in path_planner.captured_items else (255, 255, 255)
                            cv2.circle(frame, point, 7, color, -1)
        
        # Обработка углов и отправка команд
        if robot_back and robot_front and path_planner.optimal_path:
            # Получаем следующую целевую точку
            if path_planner.current_target_index < len(path_planner.optimal_path):
                target_point = path_planner.optimal_path[path_planner.current_target_index]
            else:
                target_point = path_planner.optimal_path[-1]
            
            # В основном цикле, где вычисляются углы:
            target_angle = calculate_angle(robot_back, target_point)
            current_angle = calculate_angle(robot_back, robot_front)

            # Сырая разница (может быть отрицательной)
            raw_diff = (target_angle - current_angle) % 360  # Это уже нормализовано к [0, 360)

            # Минимальная разница для обычной логики
            angle_diff = angle_difference_deg(target_angle, current_angle)

            # Определяем направление поворота (старая логика)
            turn_left = angle_diff > 180

            # Обновляем с учетом нового условия ≥ 180°
            robot_controller.update_turn_direction(angle_diff, raw_diff, turn_left)
            
            # Рисуем линии и информацию
            cv2.line(frame, robot_back, target_point, (255, 100, 0), 2)
            cv2.line(frame, robot_back, robot_front, (255, 255, 255), 2)
            
            # Отображаем информацию
            info_y = 180
            infos = [
                (f"Target: {target_angle:.1f}°", (255, 100, 0)),
                (f"Current: {current_angle:.1f}°", (255, 255, 255)),
                (f"Diff: {angle_diff:.1f}°", 
                 (0, 255, 0) if angle_diff <= CONFIG['delta_angles_ROBOT_FIGURE'] else (0, 0, 255)),
                (f"Turn: {'LEFT' if robot_controller.move_left else 'RIGHT'}", (0, 255, 255))
            ]
            
            for i, (text, color) in enumerate(infos):
                cv2.putText(frame, text, (frame_width - 200, info_y + i*30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
        # Отправляем команды роботу
        if not path_planner.route_completed:
            current_time = time.time()
            if angle_diff <= CONFIG['delta_angles_ROBOT_FIGURE']:
                robot_controller.send_command('forward', current_time)
            elif robot_controller.move_left:  # теперь move_left уже определено с учетом ≥180°
                robot_controller.send_command('left', current_time)
            else:
                robot_controller.send_command('right', current_time)        

        # Рисуем границы
        cv2.line(frame, (CONFIG['Xmin'], 0), (CONFIG['Xmin'], frame_height), (0, 0, 255), 1)
        cv2.line(frame, (CONFIG['Xmax'], 0), (CONFIG['Xmax'], frame_height), (0, 0, 255), 1)
        cv2.line(frame, (0, CONFIG['Ymin']), (frame_width, CONFIG['Ymin']), (0, 0, 255), 1)
        cv2.line(frame, (0, CONFIG['Ymax']), (frame_width, CONFIG['Ymax']), (0, 0, 255), 1)
        
        # Отображаем кадр
        cv2.imshow("Optimized Robot Control", frame)
        
        # Обработка клавиш
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            path_planner.optimal_path = None
            path_planner.captured_items = []
            path_planner.current_target_index = 1
            path_planner.route_completed = False
            print("Маршрут сброшен")
    
    # Завершение работы
    robot_controller.send_command('stop', time.time())
    print("Отправлена команда STOP")
    
    vid.release()
    cv2.destroyAllWindows()
    client.close()
    mySocket.close()
    print("Программа завершена")

if __name__ == "__main__":
    main()
