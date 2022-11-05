from math import cos, sin, sqrt

from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped
from fs_msgs.msg import Cone as FSCone
from fs_msgs.msg import Track
from nav_msgs.msg import Odometry

from typing import List

MAX_CONE_RANGE = 10  # meters


class ConeDetectionTranslator(Node):
    track: List[FSCone] = []

    def __init__(self) -> None:
        super().__init__("cone_detection_translator")

        # sub to testing only sim topics
        self.create_subscription(Track, "/testing_only/track", self.track_callback, 10)
        self.create_subscription(Odometry, "/testing_only/odom", self.odom_callback, 10)

        # could also be /lidar/cone_detection
        self.detection_publisher: Publisher = self.create_publisher(
            ConeDetectionStamped, "/sim/cone_detection", 1
        )

        self.get_logger().info("---Cone Detection Map Translator Initalised---")

    def track_callback(self, track_msg: Track):
        self.track = track_msg.track

    def odom_callback(self, odom_msg: Odometry):
        # i, j, k angles in rad
        ai, aj, ak = quat2euler(
            [
                odom_msg.pose.pose.orientation.w,
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
            ]
        )

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        detected_cones: List[Cone] = []
        for cone in self.track:
            # displacement from car to cone
            x_dist = cone.location.x - x
            y_dist = cone.location.y - y
            range_ = sqrt(x_dist**2 + y_dist**2)

            if range_ < MAX_CONE_RANGE:
                detected_cone = Cone()
                detected_cone.location.x = x_dist * cos(ak) + y_dist * sin(ak)
                detected_cone.location.y = -x_dist * sin(ak) + y_dist * cos(ak)
                detected_cone.location.z = 0.0
                detected_cone.color = cone.color
                if cone.color == FSCone.YELLOW:
                    detected_cone.color = Cone.YELLOW
                if detected_cone.location.x > 0:
                    detected_cones.append(detected_cone)

        # create message with cones
        det_header = odom_msg.header
        det_header.frame_id = "base_link"
        detection_msg = ConeDetectionStamped(
            header=odom_msg.header,
            cones=detected_cones,
        )

        self.detection_publisher.publish(detection_msg)


def main():
    # begin ros node
    rclpy.init()
    node = ConeDetectionTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
