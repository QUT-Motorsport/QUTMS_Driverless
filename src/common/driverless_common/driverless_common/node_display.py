from colour import Color

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import ConeDetectionStamped
from visualization_msgs.msg import MarkerArray

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

        # marker pubs
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "debug_markers/lidar_markers", 1)
        self.slam_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "debug_markers/slam_markers", 1)

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


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
