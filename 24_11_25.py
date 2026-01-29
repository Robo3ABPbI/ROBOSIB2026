import cv2
import numpy as np

print("cool libs imported")

vid = cv2.VideoCapture(0)

print("loaded cam")


"""
Диапазоны цветов в HSV

"""

# КРУТЫЕ ЦВЕТА ДЛЯ РОБОТА (НЕ УДАЛЯТЬ!!!!!!)
yellow_LOWER_COLOR = np.array([16, 35, 189])
yellow_UPPER_COLOR = np.array([85, 255, 255])

purple_LOWER_COLOR = np.array([109, 57, 184])
purple_UPPER_COLOR = np.array([255, 109, 243])

# ЦВЕТАААААААААААААААААААААААААААААААААААА
green_LOWER_COLOR = np.array([16, 131, 45])
green_UPPER_COLOR = np.array([83, 255, 116])


"""
ОСНОВНОЙ ЦИКЛ

"""
while True:
    ret, frame = vid.read()
    if not ret:
        break
    
    # УБИРАЕМ ПРАВУЮ 1/3 КАДРА
    height, width = frame.shape[:2]
    remove_width = width // 4
    # frame[:, width - remove_width:] = 0

    # Конвертируем в HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    """
    МАСКИ!!!!!!
    
    """
    # ЦВЕТ
    mask_yellow = cv2.inRange(hsv, yellow_LOWER_COLOR, yellow_UPPER_COLOR)
    
    # МИНУС ШУМ
    mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
    mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)

    # ЦВЕТ
    mask_purp = cv2.inRange(hsv, purple_LOWER_COLOR, purple_UPPER_COLOR)
    
    # МИНУС ШУМ
    mask_purp = cv2.erode(mask_purp, None, iterations=2)
    mask_purp = cv2.dilate(mask_purp, None, iterations=2)

    # ЦВЕТ
    mask_green = cv2.inRange(hsv, green_LOWER_COLOR, green_UPPER_COLOR)
    
    # МИНУС ШУМ
    mask_green = cv2.erode(mask_purp, None, iterations=2)
    mask_green = cv2.dilate(mask_purp, None, iterations=2)


    """
    ГДЕ КОНТУРЫ!?!?!?!?!
    
    """
    contours_yellow, i = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_purp, i = cv2.findContours(mask_purp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours_yellow:
        # Аппроксимируем контур
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Если 4 вершины - прямоугольник
        if len(approx) == 4:
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
    
    for contour in contours_purp:
        # Аппроксимируем контур
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Если 4 вершины - прямоугольник
        if len(approx) == 4:
            cv2.drawContours(frame, [approx], 0, (255, 0, 0), 3)
    
    cv2.imshow("robozari 2026 v0.3", frame)
    cv2.imshow("cool YELLOW mask", mask_yellow)
    cv2.imshow("cool PURPLE mask", mask_purp)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
