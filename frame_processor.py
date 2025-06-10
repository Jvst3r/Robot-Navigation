import cv2
import numpy as np
from astar import AStar

from ultralytics import YOLO

# Загрузка модели YOLOv8
model = YOLO("best.pt")


# Функция для устранения эффекта "рыбьего глаза
def remove_fisheye(frame, camera_matrix, dist_coeffs):
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted_frame = cv2.undistort(
        frame, camera_matrix, dist_coeffs, None, new_camera_matrix
    )
    return undistorted_frame


# Функция для обнаружения объектов с помощью YOLOv8
def detect_objects(frame):
    results = model(frame)  # Предсказание на кадре
    boxes = []
    class_ids = []
    confidences = []

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  # Координаты bounding box
            class_id = int(box.cls[0].cpu().numpy())  # Класс объекта
            confidence = box.conf[0].cpu().numpy()  # Уверенность

            if confidence > 0.5:  # Порог уверенности
                boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
                class_ids.append(class_id)
                confidences.append(float(confidence))

    # Применение Non-Maximum Suppression для устранения дублирующих боксов
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    return boxes, indexes


# Функция для обнаружения препятствий
def detect_obstacles(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    # контуры препятствий
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.imshow("Threshold", threshold)
    return contours


# Функция для получения центра робота
def get_robot_center(boxes, indexes):
    for i in indexes:
        x, y, w, h = boxes[i]
        center_x = x + w // 2
        center_y = y + h // 2
        return (center_x, center_y)
    return None


# Функция для создания карты препятствий (уменьшенной)
def create_obstacle_map(frame, contours, scale_factor=4, robot_radius=10):
    small_frame = cv2.resize(
        frame, (frame.shape[1] // scale_factor, frame.shape[0] // scale_factor)
    )
    obstacle_map = np.zeros(small_frame.shape[:2], dtype=np.uint8)
    for contour in contours:
        contour = contour / scale_factor
        cv2.drawContours(obstacle_map, [contour.astype(int)], -1, 1, -1)

    # Учитываем радиус робота (расширяем препятствия, но с уменьшенным радиусом)
    kernel_size = int((robot_radius * 0.795) / scale_factor)  # Уменьшаем радиус на 30%
    if kernel_size > 0:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        obstacle_map = cv2.dilate(obstacle_map, kernel, iterations=1)

    return obstacle_map, scale_factor


# AStar
class MyAStar(AStar):
    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def distance_between(self, n1, n2):
        # Манхэттенское расстояние
        return abs(n1[0] - n2[0]) + abs(n1[1] - n2[1])

    def heuristic_cost_estimate(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def neighbors(self, node):
        # 4-связность
        x, y = node
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < self.obstacle_map.shape[1]
                and 0 <= ny < self.obstacle_map.shape[0]
            ):
                if self.obstacle_map[ny, nx] == 0:  # Проверка на препятствие
                    neighbors.append((nx, ny))
        return neighbors


# Функция для поиска пути с использованием A*
async def find_path(start, goal, obstacle_map):
    grid = MyAStar(obstacle_map)
    try:
        path = list(grid.astar(start, goal))  # Используем метод astar
        return path
    except Exception as e:
        print(f"Ошибка при поиске пути: {e}")
        return None
