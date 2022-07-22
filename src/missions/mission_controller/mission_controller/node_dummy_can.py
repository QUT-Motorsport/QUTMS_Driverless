# import ros2 libraries
# other python modules
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

# import custom message libraries
from driverless_msgs.msg import Can


# node class object that gets created
class NodeName(Node):
    def __init__(self):
        super().__init__("dummy_can")

        time.sleep(3)

        # timer fucntion
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        # dummy CAN publisher
        self.publisher: Publisher = self.create_publisher(Can, "/can_rosbound", 10)

        self.get_logger().info("---Dummy can msg node initialised---")

        self.current_msg = 0

    def callback(self):
        if self.current_msg == 0:
            can_msg = Can()
            can_msg.id = 600  # "res_start"
            can_msg.data = [1]
            self.publisher.publish(can_msg)
            self.current_msg = 1
            self.get_logger().info(f"Can msg: {can_msg.id}")

        elif self.current_msg == 1:
            can_msg = Can()
            can_msg.id = 601  # "ebs_ready"
            can_msg.data = [1]
            self.publisher.publish(can_msg)
            self.current_msg = 0
            self.get_logger().info(f"Can msg: {can_msg.id}")


# main run when script is started in the terminal
def main(args=None):
    rclpy.init(args=args)
    node = NodeName()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
