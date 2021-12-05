from math import sqrt

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image
from driverless_msgs.msg import Cone, ConeDetectionStamped

from cv_bridge import CvBridge

from .point import Point

from typing import Tuple, List, Optional

Colour = Tuple[int, int, int]


cv_bridge = CvBridge()


SCALE = 20
WIDTH = 20*SCALE  # 10m either side
HEIGHT = 20*SCALE  # 20m forward

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

LEFT_CONE_COLOUR = Cone.YELLOW
RIGHT_CONE_COLOUR = Cone.BLUE


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH/2 - y*SCALE)),
        int(round(HEIGHT - x*SCALE)),
    )


def dist(x: float, y: float) -> float:
    return sqrt(x**2 + y**2)


class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # subscribers
        self.create_subscription(ConeDetectionStamped, "zed_detector/cone_detection", self.cone_detection_callback, 1)

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "simple_controller/debug_img", 1)

        self.get_logger().info("Simple Controller Node Initalised")

    def cone_detection_callback(self, msg: ConeDetectionStamped):
        cones: List[Cone] = msg.cones

        self.get_logger().info("Received detection")

        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        
        for cone in cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
            else:
                colour = (0, 0, 0)
            
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5
            )
        
        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None

        if len(left_cones) > 0:
            closest_left = min(left_cones, key=lambda c: dist(c.location.x, c.location.y))
        if len(right_cones) > 0:
            closest_right = min(right_cones, key=lambda c: dist(c.location.x, c.location.y))

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x)/2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y)/2,
            )
        # elif closest_left is not None:
        #     target = Point(
        #         x=closest_left.location.x,
        #         y=closest_left.location.y - 2,
        #     )
        # elif closest_right is not None:
        #     target = Point(
        #         x=closest_right.location.x,
        #         y=closest_right.location.y + 2,
        #     )
        
        if target is not None:
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(target.x, target.y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=10,
                thickness=2
            )
        
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()

    rclpy.spin(simple_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
