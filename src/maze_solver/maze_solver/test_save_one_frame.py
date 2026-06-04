import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraFrameSaver(Node):
    def __init__(self):
        super().__init__('camera_frame_saver')
        self.bridge = CvBridge()
        self.saved = False
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        if self.saved:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/tmp/camera_frame.png', frame)
        self.get_logger().info('Saved frame to /tmp/camera_frame.png')
        self.saved = True

def main():
    rclpy.init()
    node = CameraFrameSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()