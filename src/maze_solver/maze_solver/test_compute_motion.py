import math
from path_planner import PathPlanner
import numpy as np

def test_compute_motion():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid_res = 0.1
    maze_h = grid.shape[0] * grid_res

    planner = PathPlanner(grid, grid_res, maze_h)

    target = (0.5, 0.5)

    test_poses = [
        (0.1, 0.1, 0.0),              # смотрит вправо
        (0.1, 0.1, math.pi/2),        # смотрит вверх
        (0.4, 0.5, 0.0),              # почти рядом по x
        (0.5, 0.5, math.pi),          # стоит в точке, но смотрит назад
    ]

    for i, pose in enumerate(test_poses):
        v, w = planner.compute_motion(pose, target)
        print(f"Тест {i}: pose={pose}, target={target} -> v={v:.3f}, w={w:.3f}")

if __name__ == "__main__":
    test_compute_motion()