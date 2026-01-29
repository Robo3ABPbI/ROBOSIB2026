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

a = 0

import pygame
import math

# Инициализация Pygame
pygame.init()

# Настройки окна
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Minecraft-style Square Movement")

# Цвета
PURPLE = (128, 0, 128)
BLACK = (0, 0, 0)
RED = (255, 0, 0)  # Для направления

# Параметры квадрата
square_size = 40
square_x = WIDTH // 2
square_y = HEIGHT // 2
angle = 0  # Угол поворота в градусах
speed = 5  # Скорость движения

# Основной цикл
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Обработка клавиш
    keys = pygame.key.get_pressed()
    
    # Поворот с помощью A и D
    if keys[pygame.K_a]:
        angle -= 5  # Поворот против часовой стрелки
        a = 3
    if keys[pygame.K_d]:
        angle += 5  # Поворот по часовой стрелке
        a = 4
        
    else:
        a = 0
    # Нормализация угла
    angle %= 360
    
    # Движение вперед и назад с помощью W и S
    if keys[pygame.K_w]:
        # Преобразуем угол в радианы для тригонометрических функций
        angle_rad = math.radians(angle)
        square_x += speed * math.sin(angle_rad)
        square_y -= speed * math.cos(angle_rad)
        a = 1
    if keys[pygame.K_s]:
        angle_rad = math.radians(angle)
        square_x -= speed * math.sin(angle_rad)
        square_y += speed * math.cos(angle_rad)
        a = 2

    else:
        a = 0
    # Ограничение движения в пределах экрана
    square_x = max(square_size // 2, min(WIDTH - square_size // 2, square_x))
    square_y = max(square_size // 2, min(HEIGHT - square_size // 2, square_y))
    
    # Отрисовка
    screen.fill(BLACK)
    
    # Создаем поверхность для квадрата с альфа-каналом
    square_surface = pygame.Surface((square_size, square_size), pygame.SRCALPHA)
    pygame.draw.rect(square_surface, PURPLE, (0, 0, square_size, square_size))
    
    # Поворачиваем квадрат
    rotated_square = pygame.transform.rotate(square_surface, angle)
    
    # Получаем новый rect для центрирования
    rotated_rect = rotated_square.get_rect(center=(square_x, square_y))
    
    # Рисуем повернутый квадрат
    screen.blit(rotated_square, rotated_rect.topleft)
    
    # Рисуем линию направления (опционально)
    direction_length = 30
    direction_x = square_x + direction_length * math.sin(math.radians(angle))
    direction_y = square_y - direction_length * math.cos(math.radians(angle))
    pygame.draw.line(screen, RED, (square_x, square_y), (direction_x, direction_y), 3)
    
    pygame.display.flip()
    clock.tick(60)

    dataToSend = a
    bytesToSend = pickle.dumps(dataToSend) # преобразование данных в байтовую систему для EV3
    print(a)
    client.send(bytesToSend) # отправка данных в EV3

pygame.quit()
