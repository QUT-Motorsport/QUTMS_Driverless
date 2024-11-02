from colour import Color

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import ConeDetectionStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from driverless_common.common import QOS_LATEST
from driverless_common.marker import marker_array_from_cone_detection

cv_bridge = CvBridge()  # translate ROS image messages to OpenCV

red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))


class DisplayDetections(Node):
    steering_angle: float = 0.0
    velocity: float = 0.0

    def __init__(self):
        super().__init__("display_node")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "lidar/cone_detection", self.lidar_callback, QOS_LATEST)
        self.create_subscription(ConeDetectionStamped, "slam/cone_detection", self.slam_callback, QOS_LATEST)

        # steering angle target sub
        self.create_subscription(AckermannDriveStamped, "control/driving_command", self.steering_callback, QOS_LATEST)

        # lookahead point for foxglove...
        self.create_subscription(PointStamped, "lookahead_point", self.lookahead_callback, QOS_LATEST)

        # marker pubs
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "debug_markers/lidar_markers", 1)
        self.slam_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "debug_markers/slam_markers", 1)
        self.lookahead_mkr_publisher: Publisher = self.create_publisher(Marker, "debug_markers/lookahead_marker", 1)

        self.get_logger().info("---Common visualisation node initialised---")

    def steering_callback(self, msg: AckermannDriveStamped):
        # get most recent driving command msg from a controller
        self.steering_angle = msg.drive.steering_angle
        self.velocity = msg.drive.speed

    def lidar_callback(self, msg: ConeDetectionStamped):
        if self.lidar_mkr_publisher.get_subscription_count() != 0:
            self.lidar_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def slam_callback(self, msg: ConeDetectionStamped):
        if self.slam_mkr_publisher.get_subscription_count() != 0:
            self.slam_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def lookahead_callback(self, msg: PointStamped):
        if self.lookahead_mkr_publisher.get_subscription_count() != 0:
            mkr = Marker()
            mkr.header = msg.header
            mkr.type = Marker.SPHERE
            mkr.action = Marker.ADD
            mkr.pose.position = msg.point
            mkr.pose.orientation.w = 1.0
            mkr.scale.x = 0.25
            mkr.scale.y = 0.25
            mkr.scale.z = 0.25
            mkr.color.r = 1.0
            mkr.color.a = 1.0
            self.lookahead_mkr_publisher.publish(mkr)


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
