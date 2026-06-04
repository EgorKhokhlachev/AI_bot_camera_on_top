import math
import heapq
import numpy as np

class PathPlanner:
    """Планирование пути A* и управление движением (включая точный подъезд по лидару)."""

    def __init__(self, grid, grid_resolution, maze_height_m,
                 kp_lin=0.5, kp_ang=1.5,
                 waypoint_tol=0.05, goal_tol=0.08,
                 approach_dist=0.12):
        self.grid = grid
        self.grid_res = grid_resolution
        self.maze_h = maze_height_m
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self.waypoint_tol = waypoint_tol
        self.goal_tol = goal_tol
        self.approach_dist = approach_dist

        self.current_path = None
        self.waypoint_idx = 0

    def astar(self, start_xy, goal_xy):
        """Планирование пути A* на сетке. Возвращает список мировых точек."""
        h, w = self.grid.shape
        start = self._world_to_grid(*start_xy)
        goal = self._world_to_grid(*goal_xy)

        if self.grid[start] == 1 or self.grid[goal] == 1:
            return None

        neighbors = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1),
                     (-1,-1,1.414), (-1,1,1.414), (1,-1,1.414), (1,1,1.414)]

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: math.hypot(start[0]-goal[0], start[1]-goal[1])}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [(c[1]*self.grid_res + self.grid_res/2,
                         (h-1-c[0])*self.grid_res + self.grid_res/2) for c in path]

            for dr, dc, cost in neighbors:
                neighbor = (current[0]+dr, current[1]+dc)
                if 0 <= neighbor[0] < h and 0 <= neighbor[1] < w and self.grid[neighbor] == 0:
                    tentative = g_score[current] + cost
                    if tentative < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative
                        f = tentative + math.hypot(neighbor[0]-goal[0], neighbor[1]-goal[1])
                        f_score[neighbor] = f
                        heapq.heappush(open_set, (f, neighbor))
        return None

    def _world_to_grid(self, x, y):
        col = int(x / self.grid_res)
        row = int((self.maze_h - y) / self.grid_res)
        h, w = self.grid.shape
        return (max(0, min(row, h-1)), max(0, min(col, w-1)))

    def compute_motion(self, current_pose, target_xy):
        """Выдаёт Twist, чтобы двигаться к одной путевой точке."""
        rx, ry, rtheta = current_pose
        tx, ty = target_xy
        angle = math.atan2(ty - ry, tx - rx)
        err_angle = self._normalize(angle - rtheta)
        err_dist = math.hypot(tx - rx, ty - ry)

        if abs(err_angle) > 0.1:
            return (0.0, self.kp_ang * err_angle)
        else:
            lin = min(self.kp_lin * err_dist, 0.3)
            return (lin, self.kp_ang * err_angle)

    @staticmethod
    def _normalize(a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

    def approach_red_ball(self, laser_ranges, laser_angles):
        """Точный подъезд по данным лидара. Возвращает (done, v, w)."""
        if laser_ranges is None:
            return False, 0.0, 0.5
        ranges = np.array(laser_ranges)
        angles = np.array(laser_angles)
        front_mask = (angles >= -math.pi/2) & (angles <= math.pi/2)
        if not np.any(front_mask):
            return False, 0.0, 0.5
        inf_mask = ~np.isfinite(ranges)
        ranges[inf_mask] = np.inf
        # Ищем минимальное расстояние в передней полусфере
        masked = np.where(front_mask, ranges, np.inf)
        min_idx = np.argmin(masked)
        min_dist = ranges[min_idx]
        min_ang = angles[min_idx]

        if np.isinf(min_dist) or min_dist > 1.0:
            return False, 0.0, 0.5

        err_dist = min_dist - self.approach_dist
        err_ang = min_ang

        if abs(err_dist) < 0.02 and abs(err_ang) < math.radians(3):
            return True, 0.0, 0.0

        if abs(err_ang) > math.radians(5):
            v, w = 0.0, self.kp_ang * err_ang
        else:
            v = min(self.kp_lin * err_dist, 0.15)
            w = self.kp_ang * err_ang
        return False, v, w