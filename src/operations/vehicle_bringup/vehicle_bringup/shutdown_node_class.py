import datetime
import signal
from subprocess import Popen

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped
from driverless_msgs.srv import TriggerBagRecord

from std_srvs.srv import Trigger

from driverless_common.common import QOS_LATEST


class ShutdownNode(Node):
    mission_process: Popen = None
    recording = None
    record_future = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(AVStateStamped, "system/av_state", self.av_state_callback, QOS_LATEST)
        self.bag_stop_cli = self.create_client(Trigger, "bag/stop")

        # clients
        self.bag_start_cli = self.create_client(TriggerBagRecord, "bag/start")
        while not self.bag_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'bag/start' not available, waiting...")

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted MISSION process")
                self.mission_process = None
            if self.record_future is not None:
                request = Trigger.Request()
                self.bag_stop_cli.call_async(request)
                # rclpy.spin_until_future_complete(self, self.bag_stop_cli)
                # if self.recording.result is True:
                self.get_logger().error("Interrupted RECORDING process")
                # else:
                # self.get_logger().error("No recording process to interrupt")
                self.record_future = None

            self.get_logger().error("Exit Node - Shutdown Triggered")
            self.destroy_node()
            exit(1)

    def start_recording(self, target_mission: str):
        now = datetime.datetime.now()
        name = f'bags/{target_mission}-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
        request = TriggerBagRecord.Request()
        request.filename = name
        self.record_future = self.bag_start_cli.call_async(request)
        self.get_logger().info(f"Recording reqested")
        # rclpy.spin_until_future_complete(self, self.record_future)
        return self.record_future.result()
