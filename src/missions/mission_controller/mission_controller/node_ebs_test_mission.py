from math import sqrt

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped, Shutdown
from geometry_msgs.msg import TwistStamped

from driverless_common.shutdown_node import ShutdownNode

from typing import List


class EBSTestMission(ShutdownNode):
    engage_ebs: bool = False

    def __init__(self):
        super().__init__("ebs_test_mission")

        self.create_subscription(ConeDetectionStamped, "vision/cone_detection", self.vision_callback, 10)
        self.create_subscription(TwistStamped, "imu/velocity", self.vel_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.shutdown_pub: Publisher = self.create_publisher(Shutdown, "shutdown", 1)

        self.get_logger().info("---EBS Test Mission initialised---")

    def vision_callback(self, msg: ConeDetectionStamped):
        cones: List[Cone] = msg.cones
        orange_cones = [cone for cone in cones if cone.color == Cone.ORANGE_BIG]
        if len(orange_cones) > 4:
            self.get_logger().info("ORANGE CONES - Finished")
            self.engage_ebs = True

    def vel_callback(self, msg: TwistStamped):
        vel = sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
        if vel > 40 / 3.6:
            self.get_logger().info("SPEED REACHED - Finished")
            self.engage_ebs = True

    def timer_callback(self):
        shutdown_msg = Shutdown(finished_engage_ebs=self.engage_ebs)
        self.shutdown_pub.publish(shutdown_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EBSTestMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
