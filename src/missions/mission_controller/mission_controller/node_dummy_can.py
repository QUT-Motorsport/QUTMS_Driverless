import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Can


# node class object that gets created
class DummyCAN(Node):
    current_msg: int = 0

    def __init__(self):
        super().__init__("dummy_can")

        time.sleep(3)

        # timer fucntion
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        # dummy CAN publisher
        self.publisher: Publisher = self.create_publisher(Can, "/can_rosbound", 10)

        self.get_logger().info("---Dummy can msg node initialised---")

    def callback(self):
        if self.current_msg == 0:
            can_msg = Can()
            can_msg.id = 0
            can_msg.data = [0]  # state '0' not ready
            self.publisher.publish(can_msg)
            self.current_msg = 1
            self.get_logger().info("Car state: not ready")

        elif self.current_msg == 1:
            can_msg = Can()
            can_msg.id = 0
            can_msg.data = [1]  # state '1' r2d button pressed
            self.publisher.publish(can_msg)
            self.current_msg = 2
            self.get_logger().info("Car state: ready")

        elif self.current_msg == 2:
            can_msg = Can()
            can_msg.id = 0
            can_msg.data = [1]  # state '1' r2d button pressed
            self.publisher.publish(can_msg)
            self.get_logger().info("Car state: driving")


# main run when script is started in the terminal
def main(args=None):
    rclpy.init(args=args)
    node = DummyCAN()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
