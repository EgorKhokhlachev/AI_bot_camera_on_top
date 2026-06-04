import math
import numpy as np
from path_planner import PathPlanner

def test_approach_red_ball():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid_res = 0.1
    maze_h = grid.shape[0] * grid_res
    planner = PathPlanner(grid, grid_res, maze_h)

    # Синтетический lidar: 181 луч от -90 до +90 градусов
    angles = np.linspace(-math.pi/2, math.pi/2, 181)

    # 1) Шар строго по центру на расстоянии 0.5 м
    ranges = np.full_like(angles, np.inf, dtype=float)
    center_idx = len(angles) // 2
    ranges[center_idx] = 0.5

    done, v, w = planner.approach_red_ball(ranges, angles)
    print("Тест 1: центр 0.5м ->", "done:", done, "v:", v, "w:", w)

    # 2) Шар слева под углом -30° (−pi/6) и ближе нормы
    ranges2 = np.full_like(angles, np.inf, dtype=float)
    idx_left = np.argmin(np.abs(angles + math.pi/6))
    ranges2[idx_left] = 0.25

    done2, v2, w2 = planner.approach_red_ball(ranges2, angles)
    print("Тест 2: слева 0.25м ->", "done:", done2, "v:", v2, "w:", w2)

    # 3) Шар уже на нужной дистанции ~approach_dist
    ranges3 = np.full_like(angles, np.inf, dtype=float)
    ranges3[center_idx] = planner.approach_dist
    done3, v3, w3 = planner.approach_red_ball(ranges3, angles)
    print("Тест 3: по центру на нужной дистанции ->", "done:", done3, "v:", v3, "w:", w3)

if __name__ == "__main__":
    test_approach_red_ball()