
# import custom message libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# other python modules
import time
# import custom message libraries
from driverless_msgs.msg import Can

class LidarProcessing(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.create_subscription(Can, "/can_rosbound", self.callback, 10)

        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 1)

        self.get_logger().info("---Mission Control node initialised---")


    def callback(self, can_msg: Can):
        start: float = time.time()


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = LidarProcessing()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()