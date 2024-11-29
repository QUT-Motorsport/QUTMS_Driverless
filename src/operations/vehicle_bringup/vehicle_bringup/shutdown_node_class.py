import datetime
import signal
from subprocess import Popen

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()

        self.reset_sub = self.create_subscription(
            AVStateStamped, "system/av_state", self.av_state_callback, QOS_LATEST, callback_group=self.sub_cb_group
        )

        self.cli_callback_group = ReentrantCallbackGroup()
        self.bag_stop_cli = self.create_client(Trigger, "bag/stop", callback_group=self.cli_callback_group)

        # clients
        self.bag_start_cli = self.create_client(TriggerBagRecord, "bag/start", callback_group=self.cli_callback_group)
        while not self.bag_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'bag/start' not available, waiting...")

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted MISSION process")
                self.mission_process = None
            if self.record_future is not None:
                while not self.bag_stop_cli.service_is_ready():
                    self.get_logger().info("Service 'bag/stop' not available, waiting...")
                    self.bag_stop_cli.wait_for_service(timeout_sec=1.0)
                request = Trigger.Request()
                self.bag_stop_cli.call_async(request)
                while not self.record_future.done() and rclpy.ok():
                    self.get_logger().info("Waiting for recording to stop")
                    rclpy.spin_once(self, timeout_sec=1.0)
                if self.record_future.result() is None:
                    self.get_logger().error("Service ERROR, recording may still be running")
                elif self.record_future.result().success:
                    self.get_logger().error("Interrupted RECORDING process")
                else:
                    self.get_logger().error("No recording process to interrupt")
                self.record_future = None

            self.get_logger().error("Exit Node - Shutdown Triggered")
            self.destroy_node()
            exit(1)

    def start_recording(self, target_mission: str):
        now = datetime.datetime.now()
        name = f'bags/{target_mission}-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
        request = TriggerBagRecord.Request()
        request.filename = name
        while not self.bag_start_cli.service_is_ready():
            self.get_logger().info("Service 'bag/start' not available, waiting...")
            self.bag_start_cli.wait_for_service(timeout_sec=1.0)
        self.record_future = self.bag_start_cli.call_async(request)
        self.get_logger().info("Recording reqested")
        while not self.record_future.done() and rclpy.ok():
            self.get_logger().info("Waiting for recording to start")
            rclpy.spin_once(self, timeout_sec=1.0)
        result = self.record_future.result()
        self.get_logger().info(f"Service Call result: {str(result)}")
        if result is None:
            self.get_logger().error("Service ERROR, recording may not have started")
        elif result.success:
            self.get_logger().info("Recording started")
        else:
            self.get_logger().error("Recording failed to start")
        return result.success
