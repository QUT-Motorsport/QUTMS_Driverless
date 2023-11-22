import time

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Cone, ConeDetectionStamped, State

from driverless_common.common import QOS_ALL, QOS_LATEST, fast_dist, midpoint

from typing import List, Tuple

Colour = Tuple[int, int, int]

ORIGIN = [0, 0]


class ReactiveController(Node):
    def __init__(self, node_name="reactive_controller_node"):
        super().__init__(node_name)

        self.initialise_params()

        if self.ebs_test:
            self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, 1)
        else:
            self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.callback, QOS_ALL)
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)

        self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)

        self.control_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)

        self.get_logger().info("---Reactive controller node initialised---")

    def initialise_params(self):
        self.target_offset = 2
        self.driving = False
        self.discovering = False

        self.declare_parameter("ebs_control", False)
        self.ebs_test = self.get_parameter("ebs_control").value

        self.get_logger().info("EBS Control: " + str(self.ebs_test))

        if self.ebs_test:
            self.Kp_ang = 2.0
            self.target_vel = 12.0  # m/s
        else:
            self.Kp_ang = 1.8
            self.target_vel = 2.0  # m/s

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

    def state_callback(self, msg: State):
        if msg.state == State.DRIVING and not self.driving:
            # delay starting driving for 2 seconds to allow for mapping to start
            time.sleep(2)
            self.driving = True
            self.discovering = True
            self.get_logger().info("Ready to drive, discovery started", once=True)
        # lap has been completed, stop this controller
        if msg.lap_count > 0 and self.discovering:
            time.sleep(2)
            self.discovering = False
            self.get_logger().debug("Lap completed, discovery stopped")

    def can_drive(self):
        if not self.discovering or not self.driving:
            return False
        self.get_logger().info("Can drive", once=True)
        return True

    def callback(self, msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")

        if not self.can_drive():
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
        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
