# ROS 2 Maze Solver Robot

Проект мобильного робота на ROS 2 и Gazebo для прохождения лабиринта, поиска красного шара и движения к выходу.

## Что делает проект

Робот:
- получает изображение с камеры;
- получает данные лидара и одометрии;
- строит карту лабиринта;
- находит красный шар;
- планирует путь;
- подъезжает к цели и затем движется к выходу.

## Стек

- ROS 2 Jazzy
- Gazebo / ros_gz_sim
- ros2_control
- Python
- OpenCV
- cv_bridge

## Структура проекта

```text
camera_on_top_ws/
├── src/
│   └── maze_solver/
│       ├── launch/
│       ├── config/
│       ├── urdf/
│       ├── worlds/
│       ├── maze_solver/
│       │   ├── main_node.py
│       │   ├── map_builder.py
│       │   ├── ball_detector.py
│       │   └── path_planner.py
│       ├── package.xml
│       └── setup.py
├── build/
├── install/
└── log/
```

## Возможности

- запуск робота в симуляции;
- мосты Gazebo ↔ ROS 2 для камеры и лидара;
- управление через `diff_drive_controller`;
- построение карты по изображению;
- детекция красного шара;
- планирование маршрута A*;
- режим точного подъезда к цели.

## Требования

Перед запуском должны быть установлены:
- ROS 2 Jazzy;
- colcon;
- ros_gz_sim;
- ros_gz_bridge;
- ros2_control;
- cv_bridge;
- OpenCV;
- Python-зависимости проекта.

## Сборка

```bash
cd ~/camera_on_top_ws
colcon build --packages-select maze_solver --symlink-install
source install/setup.bash
```

## Запуск

```bash
ros2 launch maze_solver maze_solver.launch.py
```

## Полезные команды для отладки

Проверить топики:

```bash
ros2 topic list
```

Посмотреть одометрию:

```bash
ros2 topic echo /diff_drive_controller/odom
```

Посмотреть команды движения:

```bash
ros2 topic echo /diff_drive_controller/cmd_vel
```

Проверить тип топика:

```bash
ros2 topic info /diff_drive_controller/cmd_vel
```

Проверить контроллеры:

```bash
ros2 control list_controllers
```

## Известные моменты

- важно следить за типом сообщения в `/diff_drive_controller/cmd_vel`;
- важно корректно запускать `diff_drive_controller`;
- при использовании `gz_ros_control` не нужно поднимать лишний `ros2_control_node`, если controller manager уже создаётся внутри Gazebo.

## Планы

- улучшить инициализацию карты;
- стабилизировать bootstrap движения до появления odom;
- добавить более надёжную обработку состояний и отладочное логирование;
- улучшить launch-файл и последовательность старта узлов.

## Автор

Egor

## Лицензия

Добавь здесь выбранную лицензию, например MIT.