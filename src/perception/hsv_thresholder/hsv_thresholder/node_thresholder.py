import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image
from std_msgs.msg import String

from driverless_common.common import QOS_LATEST

from .threshold import Threshold

cv_bridge = CvBridge()


class ThresholderNode(Node):
    def __init__(self):
        super().__init__("thresholder")
        self.create_subscription(String, "hsv_thresholder/threshold", self.threshold_callback, QOS_LATEST)
        self.create_subscription(Image, "/zed2i/zed_node/rgb/image_rect_color", self.image_callback, QOS_LATEST)
        threshold_img_publisher: Publisher = self.create_publisher(Image, "hsv_thresholder/thresholded_img", 1)

        self.threshold = Threshold(
            lower=[0, 0, 0],
            upper=[255, 255, 255],
        )

        self.get_logger().info("---Thresholder node initalised---")

    def image_callback(self, msg: Image):
        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        hsv_frame: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, self.threshold.lower, self.threshold.upper)

        thresholded_frame = cv2.bitwise_and(frame, frame, mask=mask)
        self.threshold_img_publisher.publish(cv_bridge.cv2_to_imgmsg(thresholded_frame, encoding="bgra8"))

    def threshold_callback(self, msg: String):
        self.threshold = Threshold.from_json(msg.data)
        self.get_logger().info(f"lower={list(self.threshold.lower)},")
        self.get_logger().info(f"upper={list(self.threshold.upper)},")
        self.get_logger().info(f"")


def main(args=None):
    rclpy.init(args=args)

    thresholder_node = ThresholderNode()

    rclpy.spin(thresholder_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
