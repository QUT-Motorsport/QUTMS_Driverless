from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Can


class BaseMission(Node):
    r2d: bool = False

    def __init__(self):
        super().__init__("mission")

        testing_override = self.declare_parameter("testing", False).get_parameter_value().bool_value
        self.get_logger().info("Auto mission override: " + str(testing_override))
        if testing_override:
            self.r2d = True

        self.create_subscription(Can, "/can_rosbound", self.can_callback, 10)

        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

    def can_callback(self, can_msg: Can):
        # listen to EBS check success state
        if can_msg.data[0] == 0 and not self.r2d:  # 'state' id not ready
            self.get_logger().debug("waiting on r2d")

        # next, listen to the RES 'go' stateW CAN msg.
        # send can msg to EBS VCU
        elif can_msg.data[0] == 1 and not self.r2d:  # state id ready and haven't previously set r2d
            self.get_logger().debug("r2d pressed")
            self.r2d = True
