import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge

import cv2
import numpy as np

from .threshold import Threshold
from .img_proc import get_coloured_objects
from .cone_rect import ConeRect, draw_box

cv_bridge = CvBridge()

YELLOW_THRESH = Threshold(
    lower=[20, 80, 60],
    upper=[45, 255, 255],
)
BLUE_THRESH = Threshold(
    lower=[110, 60, 50],
    upper=[125, 255, 255],
)
ORANGE_THRESH = Threshold(
    lower=[0, 100, 50],
    upper=[15, 255, 255],
)

YELLOW_DISP_COLOUR = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR = (0, 102, 255)  # bgr - orange
TEXT_DISP_COLOUR = (0, 255, 0) # bgr - green


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self.create_subscription(
            Image,
            "/zed2i/zed_node/rgb/image_rect_color",
            self.image_callback,
            10,
        )
        self.get_logger().info("Controller Node Initalised")

        self.create_subscription(
            DisparityImage,
            "/zed2i/zed_node/disparity/disparity_image",
            self.disparity_callback,
            10,
        )
        self.get_logger().info("Depth Node Initalised")

        self.orange_cones = list()
        self.blue_cones = list()
        self.yellow_cones = list()
        self.bounding_box_frame = []
        self.display = True

    def image_callback(self, msg):
        logger = self.get_logger()

        frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        h, w, _ = frame.shape
        left_frame: np.ndarray = frame[0:h, 0:int(w/2-1)]
        resized_frame: np.ndarray = left_frame[int(4*h/7):h, 0:w]
        hsv_frame: np.ndarray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)
        self.orange_cones = get_coloured_objects(hsv_frame, [ORANGE_THRESH], size=20)
        self.blue_cones = get_coloured_objects(hsv_frame, [BLUE_THRESH], size=30)
        self.yellow_cones = get_coloured_objects(hsv_frame, [YELLOW_THRESH], size=30)

        self.bounding_box_frame = np.copy(left_frame)

    def disparity_callback(self, msg):
        logger = self.get_logger()

        depth_map: np.ndarray = cv_bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')
        h, w = depth_map.shape
        offset_y = int(4*h/7)

        for cone in self.yellow_cones:
            draw_box(self.bounding_box_frame, box=cone, disp_colour=YELLOW_DISP_COLOUR, offset=offset_y)

            depth = -msg.f * msg.t / depth_map[(cone.center.y + offset_y, cone.center.x)]
            string = str(round(depth, 3)) + "m"
            cv2.putText(
                self.bounding_box_frame, string, # frame to draw on, label to draw
                (cone.tl.x, cone.tl.y - 3 + offset_y), # position
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, # font, font size
                TEXT_DISP_COLOUR, 1) # thickness 1

        for cone in self.blue_cones:
            draw_box(self.bounding_box_frame, box=cone, disp_colour=BLUE_DISP_COLOUR, offset=offset_y)

            depth = -msg.f * msg.t / depth_map[(cone.center.y + offset_y, cone.center.x)]
            string = str(round(depth, 3)) + "m"
            cv2.putText(
                self.bounding_box_frame, string, # frame to draw on, label to draw
                (cone.tl.x, cone.tl.y - 3 + offset_y), # position
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, # font, font size
                TEXT_DISP_COLOUR, 1) # thickness 1


        if self.display == True and self.bounding_box_frame != []:
            # show bounding boxes
            cv2.imshow("depth", self.bounding_box_frame) # image with bounding boxes
            cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)

    node = DepthNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
