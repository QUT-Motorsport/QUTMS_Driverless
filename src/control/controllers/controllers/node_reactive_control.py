from typing import List, Tuple

import numpy as np
from driverless_common.common import QOS_ALL, midpoint, fast_dist
from driverless_msgs.msg import Cone, ConeDetectionStamped

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

ORIGIN = [0, 0]

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class ReactiveController(Node):
    Kp_ang: float
    target_vel: float
    target_accel: float
    target_offset: int
    pub_accel: bool
    ebs_test: bool
    map_sub: Subscription
    control_pub: LifecyclePublisher

    def __init__(self):
        super().__init__("reactive_controller_node")

        self.ebs_test = self.declare_parameter("ebs_control", False).get_parameter_value().bool_value
        self.get_logger().info("EBS Control: " + str(self.ebs_test))

        self.target_accel = 0.0
        self.target_offset = 2
        if self.ebs_test:
            self.Kp_ang = 2.0
            self.target_vel = 12.0  # m/s
            self.target_accel = 0.0
            self.target_cone_count = 2
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
            self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, QOS_ALL)
        else:
            self.Kp_ang = 2.5
            self.target_vel = 1.5  # m/s
            self.target_accel = 0.0
            self.target_cone_count = 2
            self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.callback, QOS_ALL)
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)

        self.get_logger().info("---Reactive controller node initialised---")

    def get_closest_to_origin(self, cones):
        d = float("inf")
        index = None
        for i, cone in enumerate(cones):
            f = fast_dist(ORIGIN, cone)
            if f < d:
                d = f
                index = i
        if index is None:
            return None
        return cones[min(index + self.target_offset, len(cones) - 1)]

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
                    cones[i].color = Cone.BLUE
                else:
                    cones[i].color = Cone.YELLOW

        blues = []
        yellows = []
        for cone in cones:
            c = [cone.location.x, cone.location.y]
            if cone.color == Cone.BLUE:
                blues.append(c)
            elif cone.color == Cone.YELLOW:
                yellows.append(c)
            elif cone.color == Cone.ORANGE_BIG:
                if cone.location.y > 0:
                    blues.append(c)
                else:
                    yellows.append(c)

        closest_blue = self.get_closest_to_origin(blues)
        closest_yellow = self.get_closest_to_origin(yellows)

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

        target = None
        if closest_blue is not None and closest_yellow is not None:
            target = midpoint(closest_blue, closest_yellow)
        elif closest_blue is not None:
            target = [closest_blue[0], closest_blue[1] - 3]
        elif closest_yellow is not None:
            target = [closest_yellow[0], closest_yellow[1] + 3]

        if target is not None:
            # steering control
            steering_angle = self.Kp_ang * np.degrees(np.arctan2(target[1], target[0]))
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
