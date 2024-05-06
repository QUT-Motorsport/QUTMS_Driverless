import signal
from subprocess import Popen

from rclpy.node import Node

from driverless_msgs.msg import State

from driverless_common.common import QOS_LATEST


class ShutdownNode(Node):
    process = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)
        self.process: Popen = None

    def state_callback(self, msg: State):
        if msg.state in [State.START, State.SELECT_MISSION, State.ACTIVATE_EBS, State.FINISHED, State.EMERGENCY]:
            if self.process is not None:
                self.process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted process")

                self.process = None
            self.get_logger().error("Exit Node - Shutdown Triggered")
            self.destroy_node()
            exit(1)
