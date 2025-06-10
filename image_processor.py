import cv2
import numpy as np
import socket
import time
from path_error import PathError

from frame_processor import (
    remove_fisheye,
    detect_objects,
    detect_obstacles,
    get_robot_center,
    create_obstacle_map,
    find_path,
)

from movie1 import get_contours
from movie1 import hide_robot

import matplotlib.pyplot as plt

import asyncio

# Параметры калибровки камеры (встроены в код)
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Без дисторсии

# Глобальные переменные для хранения координат точек
clicked_point_start = None
clicked_point_goal = None

XVECTOR = np.array([1, 0])
YVECTOR = np.array([0, 1])


# Функция обработки клика мыши
def mouse_callback(event, x, y, flags, param):
    global clicked_point_start, clicked_point_goal
    if event == cv2.EVENT_LBUTTONDOWN:
        # Нормализуем координаты (от 0 до 1)
        height, width = param.shape[:2]
        x_norm = x / width
        y_norm = y / height
        if clicked_point_start is None:
            clicked_point_start = (x_norm, y_norm)
            print(f"Начальная точка установлена: ({x_norm:.2f}, {y_norm:.2f})")
        else:
            clicked_point_goal = (x_norm, y_norm)
            print(f"Конечная точка установлена: ({x_norm:.2f}, {y_norm:.2f})")

def get_vector_sing(current_point, prev_point):
    # if prev_point[0] * current_point[1] - current_point[0] * prev_point[1] < 0:
    #     return 1
    # return -1

    if current_point[0] < prev_point[0] or current_point[1] < prev_point[1]:
        return -1

    return 1


# Основная функция для обработки изображения
async def process_image(frame, robot_cords, robot_radius=20, scale_factor=5, safety_margin=1):
    global clicked_point_start, clicked_point_goal
    frame = cv2.medianBlur(frame,5)
    print(f"Координаты робота: {robot_cords}")
    x1,y1,x2,y2 = robot_cords
    robot_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    mask = hide_robot(frame, x1,y1,x2,y2, gain=5)


    
    # Применение дилатации (расширение белых областей)

    kernel = np.ones((2, 3), np.uint8)  # Ядро 5x5 для дилатации
    dilated_mask = cv2.dilate(mask, kernel, iterations=20)  # 1 итерация
    # Если робот не обнаружен, предлагаем установить начальную точку вручную
    # 4. Добавляем безопасную зону вокруг препятствий
    safety_kernel = np.ones((safety_margin*2, safety_margin*2), np.uint8)
    safe_obstacles = cv2.dilate(dilated_mask, safety_kernel, iterations=1)
    contours = get_contours(safe_obstacles, x1,y1,x2,y2, gain=5)
    # 5. Отображаем для отладки
    #cv2.imshow("Dilated Obstacles", dilated_mask)
    cv2.imshow("Safe Obstacles", safe_obstacles)
    cv2.waitKey(1)  # Добавляем небольшую задержку для обновления окна
    # cv2.imshow("MASK", mask)
    if robot_center is None:
        print(
            "Робот не обнаружен. Установите начальную"
            "точку вручную, кликнув на изображении."
        )
        
        cv2.setMouseCallback("Processed Image", mouse_callback, frame)

        # Ждем, пока пользователь установит начальную точку
        while clicked_point_start is None:
            cv2.waitKey(1)

        # Преобразуем нормализованные координаты в пиксельныеF
        height, width = frame.shape[:2]
        robot_center = (
            int(clicked_point_start[0] * width),
            int(clicked_point_start[1] * height),
        )
        print(f"Робот установлен вручную: {robot_center}")

    # Отображаем центр робота
    cv2.circle(frame, robot_center, 5, (0, 255, 0), -1)

    # Предлагаем установить конечную точку вручную
    print("Установите конечную точку вручную, кликнув на изображении.")
    cv2.setMouseCallback("Safe Obstacles", mouse_callback, frame)

    # Ждем, пока пользователь установит конечную точку
    while clicked_point_goal is None:
        cv2.waitKey(1)

    # Преобразуем нормализованные координаты в пиксельные
    height, width = frame.shape[:2]
    goal_center = (
        int(clicked_point_goal[0] * width),
        int(clicked_point_goal[1] * height),
    )
    print(f"Конечная точка установлена: {goal_center}")

    # Отображаем конечную точку
    cv2.circle(frame, goal_center, 5, (255, 0, 0), -1)

    # Создаем карту препятствий
    obstacle_map, scale_factor = create_obstacle_map(
        safe_obstacles, contours, scale_factor*6, robot_radius + safety_margin
    )
    cv2.imshow("Processed Image", frame)
    cv2.waitKey(1)

    # Определяем начальную и конечную точки (в уменьшенном масштабе)
    start_scaled = (robot_center[0] // scale_factor, robot_center[1] // scale_factor)
    goal_scaled = (goal_center[0] // scale_factor, goal_center[1] // scale_factor)

    print(f"Робот установлен вручную: {robot_center}")
    print(f"Конечная точка установлена: {goal_center}")
    
    # Строим путь
    path = await find_path(start_scaled, goal_scaled, obstacle_map)
    
    # Проверяем, найден ли путь
    if path is None:
        print(
            "Ошибка: Путь не найден. Возможно, начальная"
            " или конечная точка находятся внутри препятствия."
        )
        cv2.imshow("Processed Image", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        raise PathError()

    # Отрисовка пути (в увеличенном масштабе)
    angles = []
    cords_x = []
    cords_y = []

    if path:
        for point in path:
            x, y = point
            cords_x.append(x)
            cords_y.append(y)
            x_large = x * scale_factor
            y_large = y * scale_factor
            cv2.circle(frame, (x_large, y_large), 5, (0, 0, 255), -1)

    plt.plot(cords_x, cords_y, ".k")
    plt.show()

    if path:

        prev_angle = 0

        # prev_point = np.array(path[1])
        # prev_vector = np.array(prev_point - path[0])
        prev_point = np.array(path[0])
        prev_vector = XVECTOR
        for i in range(1, len(path) - 1):
            current_point = np.array(path[i])
            vector = current_point - prev_point

            sign = get_vector_sing(current_point, prev_point)
            angle = np.arccos(
                np.dot(vector, prev_vector) / (np.linalg.norm(vector) * np.linalg.norm(prev_vector))          
            ) * sign
            if angle != prev_angle:
                processed_angle = angle
                prev_angle = angle
            else:
                processed_angle = 0

            # print(processed_angle)
            angles.append(processed_angle)

            prev_point = current_point
            prev_vector = vector

    processed_angles = []
    counter = 1
    for i in range(1, len(angles)):
        if i == len(angles) - 1:
            processed_angles.append((angles[i], counter+1))
            break

        if angles[i] != angles[i - 1]:
            processed_angles.append((angles[i - 1], counter))
            counter = 1
        else:
            counter += 1
        # print(angles)

    # Отображение результата
    cv2.imshow("Processed Image", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return angles, processed_angles

# Основная функция для обработки изображения
async def secound_process_image(frame, robot_cords, robot_radius=20, scale_factor=5,safety_margin=1):
    frame = cv2.medianBlur(frame,5)
    print(f"Координаты робота: {robot_cords}")
    x1,y1,x2,y2 = robot_cords
    prev_robot_center = None
    robot_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    mask = hide_robot(frame, x1,y1,x2,y2, gain=5)


    
    # Применение дилатации (расширение белых областей)

    kernel = np.ones((2, 3), np.uint8)  # Ядро 5x5 для дилатации
    dilated_mask = cv2.dilate(mask, kernel, iterations=3)  # 1 итерация
    # Если робот не обнаружен, предлагаем установить начальную точку вручную
    # 4. Добавляем безопасную зону вокруг препятствий
    safety_kernel = np.ones((safety_margin*2, safety_margin*2), np.uint8)
    safe_obstacles = cv2.dilate(dilated_mask, safety_kernel, iterations=1)
    contours = get_contours(safe_obstacles, x1,y1,x2,y2, gain=5)


    # Создаем карту препятствий
    obstacle_map, scale_factor = create_obstacle_map(
        safe_obstacles, contours, scale_factor*6, robot_radius + safety_margin
    )

    height, width = frame.shape[:2]
    goal_center = (
        int(clicked_point_goal[0] * width),
        int(clicked_point_goal[1] * height),
    )

    # Определяем начальную и конечную точки (в уменьшенном масштабе)
    start_scaled = (robot_center[0] // scale_factor, robot_center[1] // scale_factor)
    goal_scaled = (goal_center[0] // scale_factor, goal_center[1] // scale_factor)


    #print(f"Робот установлен вручную: {robot_center}")
    #print(f"Конечная точка установлена: {goal_center}")
    
    # Строим путь

    path = await find_path(start_scaled, goal_scaled, obstacle_map)


    # Проверяем, найден ли путь
    if path is None:
        print(
            "Ошибка: Путь не найден. Возможно, начальная"
            " или конечная точка находятся внутри препятствия."
        )
        #cv2.imshow("Processed Image", frame)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #cv2.imshow("Error", safe_obstacles)
        raise PathError()

    # Отрисовка пути (в увеличенном масштабе)
    angles = []
    cords_x = []
    cords_y = []

    if path:
        for point in path:
            x, y = point
            cords_x.append(x)
            cords_y.append(y)
            x_large = x * scale_factor
            y_large = y * scale_factor


            # --- ПОКАЗАТЬ ПУТЬ ---
            #cv2.circle(frame, (x_large, y_large), 5, (0, 0, 255), -1)
            # ---------------------
        # --- Отображаем центр робота ---
        #cv2.circle(frame, robot_center, 5, (0, 255, 0), -1)
        # --------------------------------
        # if prev_robot_center:
        #     cv2.circle(frame, prev_robot_center, 5, (0, 255, 0), -1)
        # prev_robot_center = robot_center


    plt.plot(cords_x, cords_y, ".k")
    #plt.show()


    if path:

        prev_angle = 0

        # prev_point = np.array(path[1])
        # prev_vector = np.array(prev_point - path[0])
        prev_point = np.array(path[0])
        prev_vector = XVECTOR
        for i in range(1, len(path) - 1):
            current_point = np.array(path[i])
            vector = current_point - prev_point

            sign = get_vector_sing(current_point, prev_point)
            angle = np.arccos(
                np.dot(vector, prev_vector) / (np.linalg.norm(vector) * np.linalg.norm(prev_vector))
            ) * sign
            if angle != prev_angle:
                processed_angle = angle
                prev_angle = angle
            else:
                processed_angle = 0

            # print(processed_angle)
            angles.append(processed_angle)

            prev_point = current_point

    processed_angles = []
    counter = 1
    for i in range(1, len(angles)):
        if i == len(angles) - 1:
            processed_angles.append((angles[i], counter+1))
            break

        if angles[i] != angles[i - 1]:
            processed_angles.append((angles[i - 1], counter))
            counter = 1
        else:
            counter += 1
        # print(angles)


    #region -------------Отображение результата-------------

    # cv2.imshow("Processed Image", frame)
    # cv2.waitKey(0)

    #endregion ----------------------------------------------


    # cv2.destroyAllWindows()

    return angles, processed_angles

# Запуск обработки изображения
if __name__ == "__main__":
    # asyncio.run(process_image("maze2w.jpg", robot_radius=65, scale_factor=4))
    asyncio.run(process_image("1.jpg", robot_radius=65, scale_factor=4))
    # asyncio.run(process_image("maze2c.png", robot_radius=90, scale_factor=4))
