import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, State

from driverless_common.common import QOS_ALL, QOS_LATEST
from driverless_common.point import Point, cone_to_point, dist
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Optional, Tuple

Colour = Tuple[int, int, int]


ORIGIN = Point(0, 0)

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class ReactiveController(Node):
    Kp_ang: float
    target_vel: float
    target_accel: float
    target_cone_count: int
    pub_accel: bool
    ebs_test: bool
    discovering: bool = True
    driving: bool = False

    def __init__(self):
        super().__init__("reactive_controller_node")

        self.ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(self.ebs_test))

        # sub to state broadcast to determine what this controller should be doing
        self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)

        if self.ebs_test:
            self.Kp_ang = 2.0
            self.target_vel = 12.0  # m/s
            self.target_accel = 0.0
            self.target_cone_count = 2
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
            self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, QOS_ALL)
        else:
            self.Kp_ang = 2.0
            self.target_vel = 1.5  # m/s
            self.target_accel = 0.0
            self.target_cone_count = 2
            self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.callback, QOS_ALL)
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)

        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Reactive controller node initalised---")

    def state_callback(self, msg: State):
        if msg.state == State.DRIVING and not self.driving:
            # delay starting driving for 2 seconds to allow for mapping to start
            time.sleep(2)
            self.driving = True
            self.get_logger().info("Ready to drive, discovery started")
        # lap has been completed, stop this controller
        if msg.lap_count > 0 and not self.discovering:
            self.discovering = False
            self.get_logger().debug("Lap completed, discovery stopped")

    def callback(self, msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        if not self.discovering or not self.driving:
            return

        # safety critical, set to 0 if not good detection
        speed = 0.0
        steering_angle = 0.0

        cones: List[Cone] = msg.cones

        if msg.header.frame_id == "velodyne" and self.ebs_test:
            # change colour of cones based on on y pos as this is lidar scan
            for i in range(len(cones)):
                if cones[i].location.y > 0:
                    cones[i].color = LEFT_CONE_COLOUR
                else:
                    cones[i].color = RIGHT_CONE_COLOUR

        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]
        # map orange cones to left and right cones
        orange_cones = [c for c in cones if c.color == Cone.ORANGE_BIG]
        for i in range(len(orange_cones)):
            if cones[i].location.y > 0:
                left_cones.append(cones[i])
            else:
                right_cones.append(cones[i])

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None
        if len(left_cones) > 0:
            closest_left = sorted(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[
                min(self.target_cone_count, len(left_cones) - 1)
            ]

        if len(right_cones) > 0:
            closest_right = sorted(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))[
                min(self.target_cone_count, len(right_cones) - 1)
            ]

        # if we have two cones, check if they are greater than 5 meters apart
        # if closest_left is not None and closest_right is not None:
        # if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
        #     # if so - remove the furthest cone from the targeting
        #     left_dist = dist(ORIGIN, cone_to_point(closest_left))
        #     right_dist = dist(ORIGIN, cone_to_point(closest_right))
        #     if left_dist <= right_dist:
        #         closest_right = None
        #     else:
        #         closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x) / 2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y) / 2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 3,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 3,
            )

        if target is not None:
            # steering control
            steering_angle = self.Kp_ang * np.degrees(np.arctan2(target.y, target.x))
            self.get_logger().debug(f"Target angle: {steering_angle}")
            speed = self.target_vel

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = msg.header.stamp
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = speed
        control_msg.drive.acceleration = self.target_accel
        self.control_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
