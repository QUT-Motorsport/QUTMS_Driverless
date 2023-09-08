from typing import List, Tuple

import numpy as np
from driverless_common.common import QOS_ALL, midpoint, fast_dist
from driverless_msgs.msg import Cone, ConeDetectionStamped

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.lifecycle import LifecycleNode, LifecycleState, LifecyclePublisher
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.subscription import Subscription

Colour = Tuple[int, int, int]

ORIGIN = [0, 0]


class ReactiveController(LifecycleNode):
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
            # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
        else:
            self.Kp_ang = 2.5
            self.target_vel = 1.5  # m/s
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
        self.control_pub.publish(control_msg)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.control_pub = self.create_lifecycle_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        topic = "/lidar/cone_detection" if self.ebs_test else "/slam/local_map"
        self.map_sub = self.create_subscription(ConeDetectionStamped, topic, self.callback, QOS_ALL)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.map_sub)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.map_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.map_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.map_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
