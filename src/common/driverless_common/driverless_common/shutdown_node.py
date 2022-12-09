import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Shutdown


class ShutdownNode(Node):
    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(Shutdown, "shutdown", self.shutdown_callback, 10)

    def shutdown_callback(self, msg: Shutdown):
        if any([msg.emergency_shutdown, msg.finished_engage_ebs, msg.finished]):
            self.destroy_node()
            raise Exception("Node Shutdown")
