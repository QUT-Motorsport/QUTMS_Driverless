from math import sin, cos, sqrt
from rclpy.node import Node
from rclpy.publisher import Publisher
from nav_msgs.msg import Odometry
from driverless_msgs.msg import ConeDetectionStamped, Cone
from fs_msgs.msg import Track, Cone as FSCone

from transforms3d.euler import quat2euler

from typing import List


MAX_CONE_RANGE = 10  # meters


class ConeDetectonTranslator(Node):
    track: List[FSCone] = []

    def __init__(self) -> None:
        super().__init__("cone_detection_translator")
        self.create_subscription(Track, "/testing_only/track", self.track_callback, 10)
        self.create_subscription(Odometry, "/testing_only/odom", self.odom_callback, 10)

        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "cone_detection", 1)

        self.get_logger().info("Node cone_detection_translator initalised")
    
    def track_callback(self, track_msg: Track):
        self.track = track_msg.track

    def odom_callback(self, odom_msg: Odometry):
        # i, j, k angles in rad
        ai, aj, ak = quat2euler([
            odom_msg.pose.pose.orientation.w,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
        ])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        detected_cones: List[Cone] = []
        for cone in self.track:
            # displacement from car to cone
            x_dist = cone.location.x - x
            y_dist = cone.location.y - y
            range_ = sqrt(x_dist**2 + y_dist**2)

            if x_dist > 0 and range_ < MAX_CONE_RANGE:
                detected_cone = Cone()
                # bradford said this transform works...
                detected_cone.location.x = x_dist*cos(ak) + y_dist*sin(ak)
                detected_cone.location.y = y_dist*cos(ak) - x_dist*sin(ak)
                detected_cone.location.z = 0.0
                detected_cone.color = cone.color
                detected_cones.append(detected_cone)

        # create message with cones
        detection_msg = ConeDetectionStamped(
            header=odom_msg.header,
            cones=detected_cones,
        )

        self.detection_publisher.publish(detection_msg)
