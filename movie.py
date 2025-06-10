import cv2
import numpy as np
import heapq

# Загрузка изображения
# frame = cv2.imread('new1.jpg')

def get_contours(frame, x1, y1, x2, y2, gain=0):
    # Применение медианного размытия
    blurVar = 0
    frame = cv2.medianBlur(frame, 1 + blurVar * 2)


    # Создание структурного элемента для морфологических операций
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))

    # Применение морфологической операции открытия
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)


    # Определение координат прямоугольника
    top_left = (x1-gain, y1-gain)  # замените на ваши координаты
    bottom_right = (x2+gain, y2+gain)  # замените на ваши координаты


    # _, threshold = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow("Threshold", threshold)

    # контуры препятствий
    # cv2.imshow("Mask", mask)
    contours, _ = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

    # Вывод результатов
    # cv2.imshow('Original Frame with Rectangle', frame)  # Отображение оригинального изображения с прямоугольником
    # cv2.imshow('Mask', mask)


    # # Отображение маски

    # # Ожидание нажатия клавиши для закрытия окон
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
def hide_robot(frame, x1, y1, x2, y2, gain=0):
    # Применение медианного размытия
    blurVar = 1
    frame = cv2.medianBlur(frame, 1 + blurVar * 2)

    # Создание маски
    mask = cv2.inRange(frame, (10, 10, 20), (110, 96, 95))

    # Создание структурного элемента для морфологических операций
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))

    # Применение морфологической операции открытия
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    height, width, channels = frame.shape
    print(height, width)

    # Определение координат прямоугольника
    top_left = (x1-gain, y1-gain)  # замените на ваши координаты
    bottom_right = (x2+gain, y2+gain)  # замените на ваши координаты

    # Рисование черного прямоугольника на изображении
    cv2.rectangle(mask, top_left, bottom_right, (0, 0, 0), -1)  # -1 означает заполнение

    # _, threshold = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY_INV)

    # cv2.imshow("Threshold", threshold)

    # контуры препятствий
    # cv2.imshow("Mask", mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return mask

# print(a(frame, 450, 100, 900, 200))