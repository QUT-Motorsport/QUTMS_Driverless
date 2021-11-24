import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


cv_bridge = CvBridge()


def get_thresh():
    vmax = cv2.getTrackbarPos("VMax", "Thresholder_App")
    vmin = cv2.getTrackbarPos("VMin", "Thresholder_App")
    smax = cv2.getTrackbarPos("SMax", "Thresholder_App")
    smin = cv2.getTrackbarPos("SMin", "Thresholder_App")
    hmax = cv2.getTrackbarPos("HMax", "Thresholder_App")
    hmin = cv2.getTrackbarPos("HMin", "Thresholder_App")

    min_ = np.array([hmin, smin, vmin])
    max_ = np.array([hmax, smax, vmax])

    return min_, max_

def print_thresh(_):
    min_, max_ = get_thresh()

    print(f"lower={list(min_)},")
    print(f"upper={list(max_)},")
    print()


cv2.namedWindow("Thresholder_App")

cv2.createTrackbar("HMax", "Thresholder_App", 0, 179, print_thresh)
cv2.createTrackbar("HMin", "Thresholder_App", 0, 179, print_thresh)
cv2.createTrackbar("SMax", "Thresholder_App", 0, 255, print_thresh)
cv2.createTrackbar("SMin", "Thresholder_App", 0, 255, print_thresh)
cv2.createTrackbar("VMax", "Thresholder_App", 0, 255, print_thresh)
cv2.createTrackbar("VMin", "Thresholder_App", 0, 255, print_thresh)

cv2.setTrackbarPos("VMax", "Thresholder_App", 255)
cv2.setTrackbarPos("VMin", "Thresholder_App", 0)
cv2.setTrackbarPos("SMax", "Thresholder_App", 255)
cv2.setTrackbarPos("SMin", "Thresholder_App", 0)
cv2.setTrackbarPos("HMax", "Thresholder_App", 179)
cv2.setTrackbarPos("HMin", "Thresholder_App", 0)


class ThresholderNode(Node):
    def __init__(self):
        super().__init__("thresholder")
        self.create_subscription(
            Image,
            "/zed2i/zed_node/left/image_rect_color",
            self.image_callback,
            10,
        )
        self.get_logger().info("Thresholder Node Initalised")
    
    def image_callback(self, msg):
        self.get_logger().info("Recieved image")
        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        hsv_frame: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        min_, max_ = get_thresh()
        mask = cv2.inRange(hsv_frame, min_, max_)

        thresholded_frame = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Thresholded", thresholded_frame)
        k = cv2.waitKey(1) & 0xFF

        # exit if q or esc are pressed
        if k == ord("q") or k == 27:
            cv2.destroyAllWindows()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    thresholder_node = ThresholderNode()

    rclpy.spin(thresholder_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
