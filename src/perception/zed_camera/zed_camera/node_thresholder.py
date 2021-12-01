import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


cv_bridge = CvBridge()

WINDOW_NAME = "Thresholder"


def get_thresh():
    vmax = cv2.getTrackbarPos("VMax", WINDOW_NAME)
    vmin = cv2.getTrackbarPos("VMin", WINDOW_NAME)
    smax = cv2.getTrackbarPos("SMax", WINDOW_NAME)
    smin = cv2.getTrackbarPos("SMin", WINDOW_NAME)
    hmax = cv2.getTrackbarPos("HMax", WINDOW_NAME)
    hmin = cv2.getTrackbarPos("HMin", WINDOW_NAME)

    min_ = np.array([hmin, smin, vmin])
    max_ = np.array([hmax, smax, vmax])

    return min_, max_

def print_thresh(_):
    min_, max_ = get_thresh()

    print(f"lower={list(min_)},")
    print(f"upper={list(max_)},")
    print()


cv2.namedWindow(WINDOW_NAME)

cv2.createTrackbar("HMax", WINDOW_NAME, 0, 179, print_thresh)
cv2.createTrackbar("HMin", WINDOW_NAME, 0, 179, print_thresh)
cv2.createTrackbar("SMax", WINDOW_NAME, 0, 255, print_thresh)
cv2.createTrackbar("SMin", WINDOW_NAME, 0, 255, print_thresh)
cv2.createTrackbar("VMax", WINDOW_NAME, 0, 255, print_thresh)
cv2.createTrackbar("VMin", WINDOW_NAME, 0, 255, print_thresh)

cv2.setTrackbarPos("VMax", WINDOW_NAME, 255)
cv2.setTrackbarPos("VMin", WINDOW_NAME, 0)
cv2.setTrackbarPos("SMax", WINDOW_NAME, 255)
cv2.setTrackbarPos("SMin", WINDOW_NAME, 0)
cv2.setTrackbarPos("HMax", WINDOW_NAME, 179)
cv2.setTrackbarPos("HMin", WINDOW_NAME, 0)


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
        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        hsv_frame: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        min_, max_ = get_thresh()
        mask = cv2.inRange(hsv_frame, min_, max_)

        thresholded_frame = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow(WINDOW_NAME, thresholded_frame)
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
