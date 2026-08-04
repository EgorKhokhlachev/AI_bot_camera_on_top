#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from maze_solver.map_builder import MapBuilder
from maze_solver.ball_detector import BallDetector
from maze_solver.path_planner import PathPlanner

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

print(">>> LOADED maze_solver.main_node FROM SRC (debug tag v1)")


class MazeRobotNode(Node):
    def __init__(self):
        super().__init__('maze_robot_node')

        package_share = Path(get_package_share_directory('maze_solver'))
        default_model_path = package_share / 'config' / 'ball_detector.pt'

        # Параметры
        self.declare_parameter('model_path', str(default_model_path))
        self.declare_parameter('maze_width_m', 3.0)
        self.declare_parameter('maze_height_m', 3.0)
        self.declare_parameter('approach_distance', 0.12)

        model_path = self.get_parameter('model_path').value
        maze_w = self.get_parameter('maze_width_m').value
        maze_h = self.get_parameter('maze_height_m').value
        self.approach_dist = self.get_parameter('approach_distance').value

        # Модули
        self.map_builder = MapBuilder(maze_w, maze_h)
        self.ball_detector = BallDetector(model_path)
        self.planner = None

        # ROS
        self.bridge = CvBridge()

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, odom_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )


        # Состояния
        self.state = 'INIT'
        self.latest_odom = None
        self.latest_scan = None

        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.initial_pose_set = False
        self.map_initialized = False
        self.waiting_for_odom_logged = False
        self.first_odom_logged = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Control timer created")
        self.get_logger().info('Maze robot node ready')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        if not self.first_odom_logged:
            self.get_logger().info('First odometry received')
            self.first_odom_logged = True

        if not self.initial_pose_set:
            self.stop_robot()

    def laser_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def image_callback(self, msg: Image):
        if self.state != 'INIT' or self.map_initialized:
            return

        if self.latest_odom is None:
            if not self.waiting_for_odom_logged:
                self.get_logger().warn('Waiting for odometry before map initialization')
                self.waiting_for_odom_logged = True
            return

        self.get_logger().info('Building map from camera...')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        # Карта
        self.map_builder.build_map(cv_image)

        # Красный шар
        pixel_center = self.ball_detector.detect_red_ball(cv_image)
        if pixel_center:
            wx, wy = self.map_builder.pixel_to_world(*pixel_center)
            self.map_builder.red_ball_goal = (wx, wy)
        else:
            self.get_logger().error('Red ball not found')

        # Выход
        self.map_builder.detect_exit()

        # Начальная поза робота по изображению
        robot_x, robot_y = self.map_builder.detect_robot_initial(cv_image)

        odom_pose = self.latest_odom.pose.pose

        self.get_logger().info(
            f"Robot image pose=({robot_x:.2f}, {robot_y:.2f}), "
            f"odom pose=({odom_pose.position.x:.2f}, {odom_pose.position.y:.2f}), "
            f"red_ball_goal={self.map_builder.red_ball_goal}"
        )

        self.odom_offset_x = robot_x - odom_pose.position.x
        self.odom_offset_y = robot_y - odom_pose.position.y
        self.initial_pose_set = True
        self.map_initialized = True

        self.get_logger().info(
            f'Offset: ({self.odom_offset_x:.3f}, {self.odom_offset_y:.3f})'
        )

        # Инициализация планировщика
        self.planner = PathPlanner(
            self.map_builder.occupancy_grid,
            self.map_builder.grid_resolution,
            self.map_builder.maze_height_m,
            approach_dist=self.approach_dist
        )

        if self.map_builder.red_ball_goal and self.map_builder.exit_goal:
            self.state = 'GO_TO_RED'
            self.get_logger().info('Start navigation to red ball')
        else:
            self.state = 'DONE'
            self.stop_robot()
            self.get_logger().warn('Map initialized, but goal data is incomplete')

    def get_pose(self):
        if not self.initial_pose_set or self.latest_odom is None:
            return None

        p = self.latest_odom.pose.pose
        map_x = p.position.x + self.odom_offset_x
        map_y = p.position.y + self.odom_offset_y

        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)

        return map_x, map_y, theta

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):
        print("+++++++++++++++")
        self.get_logger().info(
            f"[LOOP] state={self.state}, "
            f"planner={self.planner is not None}, "
            f"odom_is_none={self.latest_odom is None}"
        )


        if self.state in ['INIT', 'DONE'] or self.planner is None:
            return


        pose = self.get_pose()
        if pose is None:
            return

        rx, ry, rtheta = pose

        if self.state == 'GO_TO_RED':
            goal = self.map_builder.red_ball_goal
        elif self.state == 'GO_TO_EXIT':
            goal = self.map_builder.exit_goal
        elif self.state == 'APPROACH_RED':
            goal = self.map_builder.red_ball_goal
        else:
            return

        if goal is None:
            self.get_logger().error('Current goal is None')
            self.stop_robot()
            self.state = 'DONE'
            return

        # Точный подъезд к красному шару
        if self.state == 'APPROACH_RED':
            if self.latest_scan is not None:
                ranges = self.latest_scan.ranges
                angles = [
                    self.latest_scan.angle_min + i * self.latest_scan.angle_increment
                    for i in range(len(ranges))
                ]
                done, v, w = self.planner.approach_red_ball(ranges, angles)
                if done:
                    self.state = 'GO_TO_EXIT'
                    self.planner.current_path = None
                    self.get_logger().info('Approach done, heading to exit')
                    self.stop_robot()
                    return

                self.publish_cmd(v, w)
            else:
                self.stop_robot()
            return

        # Обычная навигация
        dist_to_goal = math.hypot(rx - goal[0], ry - goal[1])

        self.get_logger().info(
            f"[{self.state}] pose=({rx:.2f}, {ry:.2f}, {rtheta:.2f}), "
            f"goal=({goal[0]:.2f}, {goal[1]:.2f}), dist={dist_to_goal:.2f}"
        )

        if self.state == 'GO_TO_RED' and dist_to_goal < 0.3:
            self.state = 'APPROACH_RED'
            self.planner.current_path = None
            self.get_logger().info('Switching to precise approach')
            self.stop_robot()
            return

        if self.state == 'GO_TO_EXIT' and dist_to_goal < self.planner.goal_tol:
            self.get_logger().info('Reached exit!')
            self.stop_robot()
            self.state = 'DONE'
            return

        # Если путь исчерпан или не существует – перепланируем
        if self.planner.current_path is None or self.planner.waypoint_idx >= len(self.planner.current_path):
            self.planner.current_path = None
            path = self.planner.astar((rx, ry), goal)
            if path is None:
                self.get_logger().error('A* failed to find path')
                self.stop_robot()
                self.state = 'DONE'
                return
            self.planner.current_path = path
            self.planner.waypoint_idx = 0

        # Движение по текущему пути
        idx = self.planner.waypoint_idx
        if idx < len(self.planner.current_path):
            wx, wy = self.planner.current_path[idx]

            if math.hypot(rx - wx, ry - wy) < self.planner.waypoint_tol:
                self.planner.waypoint_idx += 1
                # Если после этого путь закончился – остановим и дадим следующему циклу перепланировать
                if self.planner.waypoint_idx >= len(self.planner.current_path):
                    self.stop_robot()
                    return
                wx, wy = self.planner.current_path[self.planner.waypoint_idx]

            if self._obstacle_ahead():
                self.publish_cmd(0.0, 0.5)
                return

            v, w = self.planner.compute_motion((rx, ry, rtheta), (wx, wy))
            self.get_logger().info(
                f"[{self.state}] cmd v={v:.2f}, w={w:.2f}, "
                f"wp=({wx:.2f}, {wy:.2f}), idx={self.planner.waypoint_idx}"
            )
            self.publish_cmd(v, w)
        else:
            # На всякий случай сбросим путь (обычно сюда не должны попадать)
            self.planner.current_path = None

    def _obstacle_ahead(self):
        if self.latest_scan is None:
            return False

        ranges = self.latest_scan.ranges
        ang = self.latest_scan.angle_min
        inc = self.latest_scan.angle_increment

        idx = [i for i in range(len(ranges)) if -0.26 <= ang + i * inc <= 0.26]
        vals = [ranges[i] for i in idx if math.isfinite(ranges[i])]

        return len(vals) > 0 and min(vals) < 0.15


def main(args=None):
    print(">>> MAIN() START maze_solver.main_node (debug tag v1)")
    rclpy.init(args=args)
    print("MAIN FROM SRC VERSION 0.0.2")
    node = MazeRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()