from math import sqrt

import cv2
import numpy as np
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from builtin_interfaces.msg import Duration
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from driverless_common.marker import marker_array_from_cone_detection
from driverless_common.point import Point

from typing import List, Tuple

cv_bridge = CvBridge()

SCALE = 20
WIDTH = 30 * SCALE  # 15m either side
HEIGHT = 30 * SCALE  # 30m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH / 2), HEIGHT)

Colour = Tuple[int, int, int]

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 80, 255)  # bgr - orange
UNKNOWN_DISP_COLOUR: Colour = (255, 255, 255)  # bgr - white

LEFT_CONE_COLOUR = Cone.YELLOW
RIGHT_CONE_COLOUR = Cone.BLUE


def cone_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH / 2 - y * SCALE)),
        int(round(HEIGHT - x * SCALE)),
    )


def cone_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH / 2 - y * SCALE)),
        int(round(HEIGHT - x * SCALE)),
    )


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def draw_markers(cones: List[Cone]) -> np.ndarray:
    debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    for cone in cones:
        if cone.color == Cone.YELLOW:
            colour = YELLOW_DISP_COLOUR
        elif cone.color == Cone.BLUE:
            colour = BLUE_DISP_COLOUR
        elif cone.color == Cone.ORANGE_BIG:
            colour = ORANGE_DISP_COLOUR
        else:
            colour = (255, 255, 255)

        cv2.drawMarker(
            debug_img,
            cone_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
            colour,
            markerType=cv2.MARKER_SQUARE,
            markerSize=5,
            thickness=5,
        )

    return debug_img


class DisplayDetections(Node):
    car_x: float = 0.0
    car_y: float = 0.0
    car_theta: float = 0.0

    def __init__(self):
        super().__init__("display_detections")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.vision_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.lidar_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/sim_cones/cone_detection", self.sim_cones_callback, 1)

        self.vision_disp_publisher: Publisher = self.create_publisher(Image, "/vision/vision_det_img", 1)
        self.lidar_disp_publisher: Publisher = self.create_publisher(Image, "/lidar/lidar_det_img", 1)
        self.sim_cones_disp_publisher: Publisher = self.create_publisher(Image, "/sim_cones/sim_cones_det_img", 1)

        self.vision_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/vision_markers", 1)
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/lidar_markers", 1)
        self.sim_cones_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/sim_cones_markers", 1)

        self.get_logger().info("---Cone display node initialised---")

    def vision_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received vision detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        self.vision_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.vision_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def lidar_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received lidar detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        self.lidar_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.lidar_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def sim_cones_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received sim cones detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        self.sim_cones_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.sim_cones_mkr_publisher.publish(marker_array_from_cone_detection(msg))


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
