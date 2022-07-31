from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Can


class BaseMission(Node):
    r2d: bool = False

    def __init__(self):
        super().__init__("mission")

        self.create_subscription(Can, "/can_rosbound", self.can_callback, 10)

        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

    def can_callback(self, can_msg: Can):
        # listen to EBS check success state
        if can_msg.data[0] == 0:  # 'state' id not ready
            self.get_logger().info("waiting on r2d")

        # next, listen to the RES 'go' state CAN msg.
        # send can msg to EBS VCU
        elif can_msg.data[0] == 1 and not self.r2d:  # state id ready and haven't previously set r2d
            self.get_logger().info("r2d pressed")
            self.r2d = True
