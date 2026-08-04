import numpy as np
from path_planner import PathPlanner  # скорректируй импорт под своё имя файла

def test_astar():
    # 0 — свободно, 1 — стена
    grid = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 1],
        [0, 0, 0, 0, 0],
    ], dtype=np.uint8)

    grid_res = 0.1      # 10 см на клетку
    maze_h = grid.shape[0] * grid_res  # высота карты в метрах

    planner = PathPlanner(grid, grid_res, maze_h)

    start_xy = (0.1, 0.1)  # мир (метры)
    goal_xy  = (maze_h - 0.05, maze_h - 0.05)

    path = planner.astar(start_xy, goal_xy)

    if path is None:
        print("Путь не найден")
        return

    print("Путь (мировые координаты):")
    for i, (x, y) in enumerate(path):
        print(f"{i:02d}: x={x:.3f}, y={y:.3f}")

if __name__ == "__main__":
    test_astar()