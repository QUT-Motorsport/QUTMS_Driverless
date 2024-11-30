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
    stop_record_future = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_callback_group = ReentrantCallbackGroup()
        self.bag_stop_cli = self.create_client(Trigger, "bag/stop", callback_group=self.cli_callback_group)
        self.bag_start_cli = self.create_client(TriggerBagRecord, "bag/start", callback_group=self.cli_callback_group)

        while not self.bag_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'bag/start' not available, waiting...")

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted MISSION process")
                self.mission_process = None
            request = Trigger.Request()
            self.stop_record_future = self.bag_stop_cli.call_async(request)
            self.trigger_shutdown()

    def trigger_shutdown(self):
        while self.stop_record_future is not None:
            continue
        self.get_logger().error("Exit Node - Shutdown Triggered")
        self.destroy_node()
        exit(1)

    def start_recording(self, target_mission: str):
        now = datetime.datetime.now()
        name = f'bags/{target_mission}-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
        request = TriggerBagRecord.Request()
        request.filename = name
        self.record_future = self.bag_start_cli.call_async(request)
        self.get_logger().info("Recording reqested")

    def start_record_callback(self):
        result = self.record_future.result()
        self.get_logger().info(f"Service Call result: {str(result)}")
        if result is None:
            self.get_logger().error(f"Service ERROR, recording may not have started: {self.record_future.exception()}")
        elif result.success:
            self.get_logger().info("Recording started")
        else:
            self.get_logger().error(f"Recording failed to start: {result.message}")
        self.get_logger().info(f"Returning {result.success}")
        return result.success

    def stop_future_callback(self):
        result = self.stop_record_future.result()
        self.get_logger().info(f"Service Call result: {str(result)}")
        if result is None:
            self.get_logger().error(
                f"Service ERROR, recording may still be running: {self.stop_record_future.exception()}"
            )
        elif result.success:
            self.get_logger().info("Recording stopped")
        else:
            self.get_logger().error(f"Recording failed to stop: {result.message}")
        self.get_logger().info(f"Returning {result.success}")
        return result.success

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.record_future is not None and self.record_future.done():
                self.start_record_callback()
                self.record_future = None
            if self.stop_record_future is not None and self.stop_record_future.done():
                self.stop_future_callback()
                self.stop_record_future = None
