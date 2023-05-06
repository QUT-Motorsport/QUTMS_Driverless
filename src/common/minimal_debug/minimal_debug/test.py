import rclpy
from rclpy.node import Node

from driverless_msgs.msg import ConeDetectionStamped

class DisplayDetections(Node):
    def __init__(self):
        super().__init__("display_node")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.vision_callback, 1)

        self.get_logger().info("---Cone display node initialised---")

    def vision_callback(self, msg: ConeDetectionStamped):
        # subscribed to vision cone detections
        self.get_logger().info("Length: " + str(len(msg.cones)))


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
