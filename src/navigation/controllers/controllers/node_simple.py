from math import sqrt, atan2, pi, sin, cos

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped

from cv_bridge import CvBridge

from driverless_common.point import Point

from typing import Tuple, List, Optional

Colour = Tuple[int, int, int]


cv_bridge = CvBridge()

SCALE = 20
WIDTH = 20*SCALE  # 10m either side
HEIGHT = 20*SCALE  # 20m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), HEIGHT)

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


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )

class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # subscribers
        self.create_subscription(ConeDetectionStamped, "/detector/cone_detection", self.cone_detection_callback, 1)

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/simple_controller/debug_img", 1)
        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "steering", 1)

        self.get_logger().info("Simple Controller Node Initalised")

    def cone_detection_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.info("Received detection")

        cones: List[Cone] = msg.cones

        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        
        for cone in cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
            else:
                colour = (255, 255, 255)
            
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
            closest_left = min(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        if len(right_cones) > 0:
            closest_right = min(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        
        # if we have two cones, check if they are greater than 5 meters apart
        if closest_left is not None and closest_right is not None:
            if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
                # if so - remove the furthest cone from the targeting
                left_dist = dist(ORIGIN, cone_to_point(closest_left))
                right_dist = dist(ORIGIN, cone_to_point(closest_right))
                if left_dist <= right_dist:
                    closest_right = None
                else:
                    closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x)/2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y)/2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 2,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 2,
            )
        
        if target is not None:
            target_img_pt = robot_pt_to_img_pt(target.x, target.y)
        #     cv2.drawMarker(
        #         debug_img, 
        #         target_img_pt,
        #         (0, 0, 255),
        #         markerType=cv2.MARKER_TILTED_CROSS,
        #         markerSize=10,
        #         thickness=2
        #     )
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            
            cv2.line(
                debug_img,
                (int(50*cos(target_img_angle) + IMG_ORIGIN.x), int(50*sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255)
            )

            steering_angle = -((pi/2) - atan2(target.x, target.y))*5
            steering_msg = AckermannDrive()
            steering_msg.steering_angle = steering_angle
            self.steering_publisher.publish(steering_msg)
            logger.info(f"Published steering angle: {steering_angle}")
        
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()

    rclpy.spin(simple_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
