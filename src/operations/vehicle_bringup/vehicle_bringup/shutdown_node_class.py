import signal
from subprocess import Popen

from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped

from driverless_common.common import QOS_LATEST


class ShutdownNode(Node):
    mission_process: Popen = None
    recording_process: Popen = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(AVStateStamped, "system/av_state", self.av_state_callback, QOS_LATEST)

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted MISSION process")
                self.mission_process = None
            if self.recording_process is not None:
                self.recording_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted RECORDING process")
                self.recording_process = None

            self.get_logger().error("Exit Node - Shutdown Triggered")
            self.destroy_node()
            exit(1)
