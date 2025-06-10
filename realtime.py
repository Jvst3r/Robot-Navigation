import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import numpy as np
from art import *

import image_processor
from movie1 import get_contours
from movie1 import hide_robot

from path_error import PathError

import asyncio

#Количество шагов
STEP_COUNT=1
ROBOT_RADIUS = 10
CORRECT_TIME = 160
COORDINATE_DIFFERENCE_FOR_CORRECT = 2
# SCALE_FACTOR = 10

DELTA_ANGLE = 0


#Скорость гусениц
left_speed=0x31
right_speed=0x32

# Повернуть на угол
time_to_90 = 600
time_to_180 = 900
time_to_1step_forward = 600

RED = "\033[31m"
RESET = "\033[0m"
print("\033[32m", end="")
print(RESET)

print(RED, end="")
print(RESET)

# Моделька
model = YOLO('best.pt')
# Камера
rtsp_url = "rtsp://admin:UrFU_ISIT@10.32.9.223:554/Streaming/channels/101"
# Подключимся
print("Подключаюсь к камере")
cap = cv2.VideoCapture(rtsp_url)
print("Подключено")

#region Устанавливаем параметры камеры
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Минимальный размер буфера
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Используем MJPEG
cap.set(cv2.CAP_PROP_FPS, 30)  # Устанавливаем частоту кадров
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Параметры камеры: FPS={30}, Разрешение={width}x{height}")
#endregion

# Обрезка
crop_x = 588
crop_y = 690
crop_w = 1315
crop_h = 230

# Ширина, высота кадра
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Матрица камеры и коэффициенты искажения~
camera_matrix = np.array([[5.47045703e+02, 0.00000000e+00, 1.27382808e+03],
                        [0.00000000e+00, 5.28615087e+02, 9.63441154e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coefs = np.array([-0.31517132, 0.10796146, -0.0019241, 0.00132282, -0.01692133])

# Получаем оптимальную новую матрицу камеры и область обрезки
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (frame_width, frame_height), 1,
                                                    (frame_width, frame_height))

print(model.names)

#region movement
host = "192.168.1.1"
port = 2001


# Команды
go_forward = b"\xff\x00\x01\x00\xff"
go_back = b"\xff\x00\x02\x00\xff"
turn_right = b"\xff\x00\x04\x00\xff"
turn_left = b"\xff\x00\x03\x00\xff"

stop_command = b"\xff\x00\x00\x00\xff"

vector = 0
true_vector = None

def kyda_napravlen_po_vectory(vector):
    if vector == 1:
        return("ВНИЗ")
    if vector == 0:
        return("ВПРАВО")
    if vector == -1:
        return("ВВЕРХ")
    if vector == -2:
        return("ВЛЕВО")

def send_command(command):
    try:
        # Создаем сокет
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Соединение с {host}:{port}")

        # Устанавливаем соединение
        s.connect((host, port))
        print(f"Отправка команды: {command}")

        # Отправляем команду
        s.sendall(command)

        # Добавляем небольшой задержку между отправками команд
        time.sleep(0.7)

        return True
    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        return False
    finally:
        # Закрываем соединение
        s.close()
        print("Соединение закрыто")


def connect_to_robot():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Создаем сокет
        
    print(f"Соединение с {host}:{port}")

    # Устанавливаем соединение
    s.connect((host, port))

    return s

def close_connection(s):
    s.close()
    print("Соединение закрыто")

def set_speed(left_speed, right_speed):
    send_command(b'\xff\x02\x01' + bytes([left_speed]) + b'\xff')
    send_command(b'\xff\x02\x02' + bytes([right_speed]) + b'\xff')

# Сделать команду по времени
def do(connection, milisecounds, command):
    try:
        # Отправляем команду
        connection.sendall(command)
        sleep(milisecounds)

        #Корректировка влево
        # connection.sendall(turn_left)
        # sleep(1)

        #Команда стоп
        # connection.sendall(b"\xff\x00\x00\x00\xff")
        sleep(2)
        connection.sendall(stop_command)

        
    except socket.error as e:
        print(f"Ошибка сокета: {e}")
        connection.close()
        return False

# Задержка
def sleep(milisecounds):
    time.sleep(milisecounds / 1500)

#endregion

connection = connect_to_robot()
#Установим скорость
set_speed(left_speed, right_speed)
# close_connection(connection)

#Проверка конфигурации
# while True:
#     connection = None
#     sleep(5)
#     do(connection, time_to_90+DELTA_ANGLE, turn_left)
#     sleep(500)

#     do(connection, time_to_90, turn_right)
#     sleep(500)

    

i =0
angles = None
prev_robot_center = None
while True:
    #region Подключаемся к камере и настраиваем ее
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    #endregion
    i+=1
    print(f"Это шаг: {i} ?")
    
    # Очищаем буфер
    for _ in range(2):
        cap.grab()

    # Получаем самый свежий кадр
    ret, frame = cap.read()

    # Сразу закрываем соединение
    cap.release()
            
    # Убираем эффект "рыбьего глаза"
    undist = cv2.undistort(frame, camera_matrix, dist_coefs, None, new_camera_matrix)

    # Обрезаем изображение на основе ROI
    resized_frame = undist[crop_y:crop_y + crop_h, crop_x:crop_x + crop_w]
    
    # Детекция объектов YOLOv8
    results = model.predict(resized_frame, conf=0.3)

    robot_center = None
    x1,y1,x2,y2 = None,None,None,None
    # Создание маски
    mask = cv2.inRange(resized_frame, (10, 10, 20), (110, 96, 95))

    # Получаем координаты робота из результатов детекции
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Проверяем, что это робот (класс 0)
            if box.cls[0].item() == 0:
                # Получаем координаты bbox в формате (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                # Вычисляем центр робота
                robot_center_x = int((x1 + x2) / 2)
                robot_center_y = int((y1 + y2) / 2)
                print(f"Координаты центра робота: x={robot_center_x}, y={robot_center_y}")

    if x1:
        print(f"корды робота: {x1}, {y1}, {x2}, {y2}")

    


    annotated_frame = results[-1].plot()  # Кадр с bbox'ами

    #Найти путь
    print(F"Поиск i, {i}")
    if angles == None and i == 1:
        if x1:
            try:
                angles, processed_angles = asyncio.run( image_processor.process_image(
                    resized_frame, 
                    robot_radius=ROBOT_RADIUS,
                    scale_factor = 5,#SCALE_FACTOR,
                    robot_cords=(int(x1), int(y1), int(x2), int(y2)),
                ))
            except PathError:
                continue
            
            print(processed_angles)
            print(angles)
            new_angles=[]
            for angle in angles:
                if angle == -np.pi:
                    new_angles.append(-2)
                elif angle == -np.pi/2:
                    new_angles.append(-1)
                elif angle == np.pi/2:
                    new_angles.append(1)
                elif angle == 0:
                    new_angles.append(0)
            print(new_angles)


    elif angles == None and i > 1:
        if x1:
            try:
                angles, processed_angles = asyncio.run( image_processor.secound_process_image(
                    resized_frame, 
                    robot_radius=ROBOT_RADIUS,
                    scale_factor = 5,#SCALE_FACTOR,
                    robot_cords=(int(x1), int(y1), int(x2), int(y2)),
                ))
            except PathError:
                continue
            
            print(processed_angles)
            print(angles)
            new_angles=[]
            for angle in angles:
                if angle == -np.pi:
                    new_angles.append(-2)
                elif angle == -np.pi/2:
                    new_angles.append(-1)
                elif angle == np.pi/2:
                    new_angles.append(1)
                elif angle == 0:
                    new_angles.append(0)
            print(new_angles)

    else:
        c=0
        # if new_angles[2] != 0:
        #    new_angles = new_angles[1:]

        if not new_angles:
            print("\033[32m", end="")
            print(RESET)
            break

        #Предикт поворота
        if new_angles[1] != 0:
            new_angles = new_angles[1:]
            print("ПРЕДИКТ ПОВОРОТА!!!")
        

        for new_angle in new_angles: 
            print(f"РОБОТ НАПРАВЛЕН - {kyda_napravlen_po_vectory(vector)}")
            print(f"СЛЕДУЮЩИЙ ШАГ - {kyda_napravlen_po_vectory(new_angle)}")
            print(f"Вектор - {vector}")
            print(f"Следующий шаг - {new_angle}")
            # connection = connect_to_robot()  

            # Нужен разворот
            if abs(new_angle - vector) == 2:
                print(f"Разворот {kyda_napravlen_po_vectory(vector)} -> {kyda_napravlen_po_vectory(new_angle)}")
                if new_angle == 1 and vector == -1:
                    do(connection, time_to_180, turn_right)

                    vector +=2
                if new_angle == -1 and vector == 1:
                    do(connection, time_to_180, turn_left)

                    vector -=2
                if new_angle == 0 and vector == -2:
                    do(connection, time_to_180, turn_right)

                    vector +=2
                # Надо повернуться влево, робот направлен вправо
                if new_angle == -2 and vector == 0:
                    do(connection, time_to_180, turn_left)
                    
                    vector -=2
                

            # Робот повернут налево, надо направиться вниз
            elif new_angle == 1 and vector == -2:
                do(connection, time_to_90, turn_left)
                print("НАЛЕВО")
                vector +=3
            # Робот повренут вниз, надо направиться влево
            elif vector == 1 and new_angle == -2:
                do(connection, time_to_90, turn_right)
                print("НАПРАВО")
                vector -=3
            elif new_angle > vector:
                do(connection, time_to_90, turn_right)
                print("НАПРАВО")
                vector +=1
            elif new_angle < vector:
                do(connection, time_to_90, turn_left)
                print("НАЛЕВО")
                vector -=1
            else:
                do(connection, time_to_1step_forward, go_forward)
                print("ВПЕРЕД")
                robot_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                robot_center_x = robot_center[0]
                robot_center_y = robot_center[1]
                
                if prev_robot_center:
                    prev_robot_center_x = prev_robot_center[0]
                    prev_robot_center_y = prev_robot_center[1]

                    #region --- Считаем настоящий вектор ---

                    # Проверка деления на ноль
                    if abs(robot_center_y - prev_robot_center_y) != 0:
                        # Двигается влево / вправо
                        if abs(robot_center_x - prev_robot_center_x) / abs(robot_center_y - prev_robot_center_y) > 2:
                            print("1) Движение по оси X")
                            # Направлен вправо
                            if robot_center_x - prev_robot_center_x > 0:
                                true_vector = 0
                            # Направлен влево
                            elif robot_center_x - prev_robot_center_x < 0:
                                true_vector = -2

                    # Проверка деления на ноль
                    if abs(robot_center_x - prev_robot_center_x) != 0:
                        # Двигается вверх / вниз
                        if abs(robot_center_y - prev_robot_center_y) / abs(robot_center_x - prev_robot_center_x) > 2:
                            print("1) Движение по оси Y")
                            #Направлен вниз
                            if robot_center_y - prev_robot_center_y > 0: # новый центр > предыдущего => новый центр ниже
                                true_vector = 1 #вниз
                            # Направлен вверх
                            elif robot_center_y - prev_robot_center_y < 0: # новый центр < предыдущего => новый центр выше
                                true_vector = -1
                    if abs(robot_center_y - prev_robot_center_y) != 0:
                        if abs(robot_center_x - prev_robot_center_x) / abs(robot_center_y - prev_robot_center_y) > 0.9 \
                        and abs(robot_center_x - prev_robot_center_x) / abs(robot_center_y - prev_robot_center_y) < 1.1:
                            
                            print("----------------------------")
                            print("Робот направлен по диагонали")
                            print("----------------------------")

                            # x новый > старого и y новый > старого => новая точка правее и ниже
                            if (robot_center_x > prev_robot_center_x) and (robot_center_y > prev_robot_center_y):
                                do(connection, time_to_90/2, turn_left)
                                vector = 0
                                true_vector = 0
                            
                            # x новый < старого и y новый > старого => новая точка левее и ниже
                            elif (robot_center_x < prev_robot_center_x) and (robot_center_y > prev_robot_center_y):
                                do(connection, time_to_90/2, turn_right)
                                vector = -2
                                true_vector = -2

                            # x новый < старого и y новый < старого => новая точка левее и выше
                            elif (robot_center_x < prev_robot_center_x) and (robot_center_y < prev_robot_center_y):
                                do(connection, time_to_90/2, turn_left)
                                vector = -2
                                true_vector = -2

                            # x новый > старого и y новый < старого => новая точка правее и выше
                            elif (robot_center_x > prev_robot_center_x) and (robot_center_y < prev_robot_center_y):
                                do(connection, time_to_90/2, turn_right)
                                vector = 0
                                true_vector = 0
                    print(f"Настоящий вектор - {kyda_napravlen_po_vectory(true_vector)}")

                    #endregion -----------------
                    if true_vector != None:
                        if true_vector != vector:
                            print("----------------------------------------")
                            print(f"Поменял вектор {kyda_napravlen_po_vectory(vector)} на {kyda_napravlen_po_vectory(true_vector)}")
                            print("----------------------------------------")

                            vector = true_vector
                            



                    if abs(robot_center_x - prev_robot_center_x) >2:
                        print(f"Новый центр x - {robot_center_x}, Предыдущий центр x - {prev_robot_center_x}")
                    if abs(robot_center_y - prev_robot_center_y) >2:
                        print(f"Новый центр y - {robot_center_y}, Предыдущий центр y - {prev_robot_center_y}")

                    if vector == 0:
                        if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_left)
                            print("Корректировка влево")
                        elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_right)
                            print("Корректировка вправо")
                    if vector == 1:
                        if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT*2:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_right)
                            print("Корректировка вправо")
                        elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT*2:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_left)
                            print("Корректировка влево")
                    if vector == -1:
                        if robot_center_x - prev_robot_center_x > COORDINATE_DIFFERENCE_FOR_CORRECT*2:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_left)
                            print("Корректировка влево")
                        elif robot_center_x - prev_robot_center_x < -COORDINATE_DIFFERENCE_FOR_CORRECT*2:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_right)
                            print("Корректировка вправо")
                    if vector == -2:
                        if robot_center_y - prev_robot_center_y > COORDINATE_DIFFERENCE_FOR_CORRECT:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_right)
                            print("Корректировка вправо")
                        elif robot_center_y - prev_robot_center_y < -COORDINATE_DIFFERENCE_FOR_CORRECT:
                            sleep(100)
                            do(connection, CORRECT_TIME, turn_left)
                            print("Корректировка влево")


                prev_robot_center = robot_center
            # close_connection(connection)
            c +=1

            
            

            # Количество шагов
            if c == STEP_COUNT:
                angles = None
                break
            
    

close_connection(connection)
# Освобождаем ресурсы и закрываем окна
cap.release()
cv2.destroyAllWindows()