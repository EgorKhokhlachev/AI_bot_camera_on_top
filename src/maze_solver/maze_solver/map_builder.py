import cv2
import numpy as np

class MapBuilder:
    """Строит карту лабиринта по изображению с верхней камеры."""

    def __init__(self, maze_width_m=3.0, maze_height_m=3.0):
        self.maze_width_m = maze_width_m
        self.maze_height_m = maze_height_m
        self.occupancy_grid = None   # 0 свободно, 1 стена/шар
        self.grid_resolution = None  # метров на пиксель
        self.red_ball_goal = None    # мировые координаты красного шара (заполняется извне)
        self.exit_goal = None        # мировые координаты выхода

    def build_map(self, image):
        """Создаёт occupancy grid из BGR-изображения."""
        h, w = image.shape[:2]
        self.grid_resolution = self.maze_width_m / w

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, wall_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Красные тона
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Зелёные тона
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        balls_mask = red_mask | green_mask
        obstacle_mask = wall_mask | balls_mask

        self.occupancy_grid = (obstacle_mask > 0).astype(np.uint8)
        return self.occupancy_grid

    def pixel_to_world(self, u, v):
        """Пересчёт пиксельных координат (начало в левом верхнем углу) в мировые (начало в левом нижнем)."""
        x = u * self.grid_resolution
        y = (self.occupancy_grid.shape[0] - v) * self.grid_resolution
        return x, y

    def world_to_grid(self, x, y):
        """Перевод мировых координат в индексы сетки [row, col]."""
        col = int(x / self.grid_resolution)
        row = int((self.maze_height_m - y) / self.grid_resolution)
        h, w = self.occupancy_grid.shape
        col = max(0, min(col, w-1))
        row = max(0, min(row, h-1))
        return row, col

    def detect_robot_initial(self, image):
        """Находит синюю метку робота и возвращает мировые (x, y)."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            h, w = image.shape[:2]
            cx, cy = w/2, h/2
        else:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
            else:
                return 0.0, 0.0
        return self.pixel_to_world(cx, cy)

    def detect_exit(self):
        """Определяет координаты выхода как самый большой разрыв на границе карты."""
        grid = self.occupancy_grid
        h, w = grid.shape

        top_free = [(0, c) for c in range(w) if grid[0, c] == 0]
        bot_free = [(h-1, c) for c in range(w) if grid[h-1, c] == 0]
        left_free = [(r, 0) for r in range(h) if grid[r, 0] == 0]
        right_free = [(r, w-1) for r in range(h) if grid[r, w-1] == 0]

        def largest_gap(cells):
            if not cells:
                return None
            groups = [[cells[0]]]
            for i in range(1, len(cells)):
                prev = cells[i-1]
                cur = cells[i]
                if (prev[0] == cur[0] and abs(prev[1] - cur[1]) == 1) or \
                   (prev[1] == cur[1] and abs(prev[0] - cur[0]) == 1):
                    groups[-1].append(cur)
                else:
                    groups.append([cur])
            longest = max(groups, key=len)
            return longest[len(longest)//2]

        candidates = [largest_gap(top_free), largest_gap(bot_free),
                      largest_gap(left_free), largest_gap(right_free)]
        candidates = [c for c in candidates if c is not None]
        if not candidates:
            self.exit_goal = None
            return None
        exit_cell = candidates[0]
        col, row = exit_cell[1], exit_cell[0]
        x = col * self.grid_resolution + self.grid_resolution / 2
        y = (h - 1 - row) * self.grid_resolution + self.grid_resolution / 2
        self.exit_goal = (x, y)
        return self.exit_goal