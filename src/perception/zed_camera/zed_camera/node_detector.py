from math import sin, cos, radians

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from driverless_msgs.msg import Cone, ConeDetectionStamped

from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np

from .threshold import Threshold
from .hsv_cv import get_coloured_bounding_boxes
from .rect import Rect, draw_box

from typing import List


cv_bridge = CvBridge()


CAMERA_FOV = 120  # degrees


YELLOW_HSV_THRESH = Threshold(
    lower=[20, 170, 100],
    upper=[40, 255, 255],
)

BLUE_HSV_THRESH = Threshold(
    lower=[120, 160, 50],
    upper=[130, 255, 255],
)

ORANGE_HSV_THRESH = Threshold(
    lower=[120, 160, 50],
    upper=[130, 255, 255],
)


YELLOW_DISP_COLOUR = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR = (0, 165, 255)  # bgr - orange

# thresh, cone_colour, display_colour
CONE_DETECTION_PARAMETERS = [
    (YELLOW_HSV_THRESH, Cone.YELLOW, YELLOW_DISP_COLOUR),
    (BLUE_HSV_THRESH, Cone.BLUE, BLUE_DISP_COLOUR),
    (ORANGE_HSV_THRESH, Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
]


def cone_distance(
    colour_frame_cone_bounding_box: Rect,
    depth_frame: np.ndarray,
) -> float:
    cone_center = colour_frame_cone_bounding_box.center
    # TODO: take vertical slice of center of cone and average for a better distance
    return depth_frame[(cone_center.y, cone_center.x)]


def cone_bearing(
    colour_frame_cone_bounding_box: Rect,
    colour_frame_camera_info: CameraInfo,
) -> float:
    cone_center = colour_frame_cone_bounding_box.center.x
    frame_width = colour_frame_camera_info.width
    center_scaled = (frame_width - cone_center / 2) / (frame_width / 2)  # 1 to -1 left to right
    return CAMERA_FOV/2 * center_scaled


def cone_msg(
    distance: float,
    bearing: float,
    colour: int,  # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
) -> Cone:
    location = Point(
        x=distance*cos(radians(bearing)),
        y=distance*sin(radians(bearing)),
        z=0,
    )

    return Cone(
        location=location,
        color=colour,
    )


class DetectorNode(Node):
    def __init__(self):
        super().__init__("zed_detector")

        # subscribers
        colour_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/rgb/image_rect_color"
        )
        colour_camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/zed2i/zed_node/rgb/camera_info"
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/depth/depth_registered"
        )

        synchronizer = message_filters.TimeSynchronizer(
            fs=[colour_sub, colour_camera_info_sub, depth_sub],
            queue_size=30,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "zed_detector/cone_detection", 1)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "zed_detector/debug_img", 1)

        self.get_logger().info("ZED Detector Node Initalised")


    def callback(self, colour_msg: Image, colour_camera_info_msg: CameraInfo, depth_msg: Image):
        logger = self.get_logger()
        logger.info("Recieved image")

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg)
        depth_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        hsv_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2HSV)

        detected_cones: List[Cone] = []
        for thresh, cone_colour, display_colour in CONE_DETECTION_PARAMETERS:
            for bounding_box in get_coloured_bounding_boxes(hsv_frame, [thresh]):
                bearing = cone_bearing(bounding_box, colour_camera_info_msg)
                distance = cone_distance(bounding_box, depth_frame)
                detected_cones.append(cone_msg(distance, bearing, cone_colour))
                draw_box(colour_frame, box=bounding_box, colour=display_colour, distance=distance)

        detection_msg = ConeDetectionStamped(
            header=colour_msg.header,
            cones=detected_cones,
        )

        self.detection_publisher.publish(detection_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(colour_frame, encoding="bgra8"))


def main(args=None):
    rclpy.init(args=args)

    detector_node = DetectorNode()

    rclpy.spin(detector_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
