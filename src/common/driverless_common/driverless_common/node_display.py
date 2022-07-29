from math import cos, pi, sin, sqrt

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

from driverless_common.marker import marker_array_from_cone_detection
from driverless_common.point import Point

from typing import List, Tuple

cv_bridge = CvBridge()

SCALE = 20
WIDTH = 20 * SCALE  # 15m either side
HEIGHT = 20 * SCALE  # 30m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH / 2), HEIGHT)

Colour = Tuple[int, int, int]

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 80, 255)  # bgr - orange
UNKNOWN_DISP_COLOUR: Colour = (255, 255, 255)  # bgr - white

LEFT_CONE_COLOUR = Cone.YELLOW
RIGHT_CONE_COLOUR = Cone.BLUE


def loc_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
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
            loc_to_img(cone.location.x, cone.location.y).to_tuple(),
            colour,
            markerType=cv2.MARKER_SQUARE,
            markerSize=5,
            thickness=5,
        )

    return debug_img


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH / 2 - y * SCALE)),
        int(round(HEIGHT - x * SCALE)),
    )


def draw_steering(debug_img: np.ndarray, steering_angle: float, velocity: float):
    # draw angle line
    cv2.line(
        debug_img,
        (
            int(50 * cos(steering_angle / 4 - pi / 2) + IMG_ORIGIN.x),
            int(50 * sin(steering_angle / 4 - pi / 2) + IMG_ORIGIN.y),
        ),
        IMG_ORIGIN.to_tuple(),
        (0, 0, 255),
    )
    # add text for targets data
    cv2.putText(debug_img, "Targets", (10, HEIGHT - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    text_angle = "Steering: " + str(round(steering_angle, 2))
    cv2.putText(debug_img, text_angle, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    text_vel = "Velocity: " + str(round(velocity, 2))
    cv2.putText(debug_img, text_vel, (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return debug_img


class DisplayDetections(Node):
    steering_angle: float = 0.0
    velocity: float = 0.0

    def __init__(self):
        super().__init__("display_detections")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.vision_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.lidar_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/sim_cones/cone_detection", self.sim_cones_callback, 1)

        # steering angle target sub
        self.create_subscription(AckermannDrive, "/driving_command", self.steering_callback, 1)

        # cv2 rosboard image pubs
        self.vision_disp_publisher: Publisher = self.create_publisher(Image, "/vision/vision_det_img", 1)
        self.lidar_disp_publisher: Publisher = self.create_publisher(Image, "/lidar/lidar_det_img", 1)
        self.sim_cones_disp_publisher: Publisher = self.create_publisher(Image, "/sim_cones/sim_cones_det_img", 1)

        # rviz marker pubs
        self.vision_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/vision_markers", 1)
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/lidar_markers", 1)
        self.sim_cones_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/sim_cones_markers", 1)

        self.get_logger().info("---Cone display node initialised---")

    def steering_callback(self, msg: AckermannDrive):
        self.steering_angle = msg.steering_angle
        self.velocity = msg.speed

    def vision_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received vision detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.vision_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.vision_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def lidar_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received lidar detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.lidar_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.lidar_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def sim_cones_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.debug("Received sim cones detection")

        cones: List[Cone] = msg.cones
        debug_img = draw_markers(cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.sim_cones_disp_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.sim_cones_mkr_publisher.publish(marker_array_from_cone_detection(msg))


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
