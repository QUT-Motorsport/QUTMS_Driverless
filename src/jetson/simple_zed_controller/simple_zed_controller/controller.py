import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

cv_bridge = CvBridge()

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.create_subscription(
            Image,
            "/zed2i/zed_node/left/image_rect_color",
            self.image_callback,
            10
        )
        self.get_logger().info("Controller Node Initalised")

    def image_callback(self, msg):
        self.get_logger().info("Recieved image")
        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        cv2.imshow("frame", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
