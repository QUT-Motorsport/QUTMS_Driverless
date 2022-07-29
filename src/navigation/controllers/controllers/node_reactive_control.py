from math import atan2, cos, pi, sin, sqrt

import cv2
import numpy as np

from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Image

from driverless_common.point import Point

from typing import List, Optional, Tuple

Colour = Tuple[int, int, int]

cv_bridge = CvBridge()

SCALE = 20
WIDTH = 20 * SCALE  # 10m either side
HEIGHT = 20 * SCALE  # 20m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH / 2), HEIGHT)

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH / 2 - y * SCALE)),
        int(round(HEIGHT - x * SCALE)),
    )


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


class ReactiveController(Node):
    Kp_ang: float = 5
    Kp_vel: float = 2
    vel_max: float = 5  # m/s
    vel_min = vel_max / 2  # m/s
    throttle_max: float = 0.3
    ang_max: float = pi / 2

    def __init__(self):
        super().__init__("reactive_controller")

        # sync subscribers
        vel_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, "/imu/velocity")
        detection_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/detection/cone_detection")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[vel_sub, detection_sub], queue_size=30, slop=0.3)
        synchronizer.registerCallback(self.callback)

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/reactive_controller/debug_img", 1)
        self.control_publisher: Publisher = self.create_publisher(AckermannDrive, "/reactive_driving_command", 1)

        self.get_logger().info("---Reactive Controller Node Initalised---")

    def callback(self, vel_msg: TwistWithCovarianceStamped, cone_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        # safety critical, set to 0 if not good detection
        control_msg = AckermannDrive()
        control_msg.steering_angle = 0.0
        control_msg.acceleration = 0.0
        control_msg.jerk = 1.0

        cones: List[Cone] = cone_msg.cones

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
                thickness=5,
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
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x) / 2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y) / 2,
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
            # velocity control
            vel = sqrt(vel_msg.twist.twist.linear.x**2 + vel_msg.twist.twist.linear.y**2)
            # target velocity proportional to angle
            target_vel: float = self.vel_max - (abs(atan2(target.x, target.y))) * self.Kp_vel
            if target_vel < self.vel_min:
                target_vel = self.vel_min
            self.get_logger().debug(f"Target vel: {target_vel}")

            # increase proportionally as it approaches target
            throttle_scalar: float = 1 - (vel / target_vel)
            if throttle_scalar > 0:
                calc_throttle = self.throttle_max * throttle_scalar
            elif throttle_scalar <= 0:
                calc_throttle = 0.0  # if its over maximum, cut throttle

            # steering control
            steering_angle = -self.Kp_ang * ((pi / 2) - atan2(target.x, target.y))
            self.get_logger().debug(f"Target angle: {steering_angle}")

            # publish message
            control_msg.steering_angle = steering_angle
            control_msg.acceleration = calc_throttle
            control_msg.jerk = 0.0

            # visualisations
            # draw target
            target_img_pt = robot_pt_to_img_pt(target.x, target.y)
            cv2.drawMarker(
                debug_img,
                target_img_pt.to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=10,
                thickness=2,
            )
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            # draw angle line
            cv2.line(
                debug_img,
                (int(50 * cos(target_img_angle) + IMG_ORIGIN.x), int(50 * sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255),
            )
            # add text for targets data
            cv2.putText(debug_img, "Targets", (10, HEIGHT - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            text_angle = "Steering: " + str(round(steering_angle, 2))
            cv2.putText(debug_img, text_angle, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            text_vel = "Velocity: " + str(round(target_vel, 2))
            cv2.putText(debug_img, text_vel, (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.control_publisher.publish(control_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
