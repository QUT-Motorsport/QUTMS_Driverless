import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Reset, Shutdown

from driverless_common.common import QOS_LATEST
from driverless_common.shutdown_node import ShutdownNode


class InspectionHandler(ShutdownNode):
    started: bool = False

    def __init__(self):
        super().__init__("inspection_logic_node")
        self.timer = self.create_timer(30, self.timer_callback)
        self.reset_sub = self.create_subscription(Reset, "/system/reset", self.reset_callback, QOS_LATEST)
        self.shutdown_pub: Publisher = self.create_publisher(Shutdown, "/system/shutdown", 1)

        self.get_logger().info("---Inspection handler node initialised---")

    def reset_callback(self, msg: Reset):
        self.timer.reset()
        self.started = True

    def timer_callback(self):
        if self.started:
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
