import datetime
import signal
from subprocess import Popen
from typing import Optional, Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped

from rosbag2_interfaces.srv import Record, Stop

from driverless_common.common import QOS_LATEST


class ShutdownNode(Node):
    mission_process: Optional[Popen] = None
    recording: Optional[Any] = None
    record_future: Optional[Any] = None
    stop_record_future: Optional[Any] = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_callback_group = ReentrantCallbackGroup()
        self.bag_stop_cli = self.create_client(Stop, "rosbag2_recorder/stop", callback_group=self.cli_callback_group)
        self.bag_start_cli = self.create_client(Record, "rosbag2_recorder/record", callback_group=self.cli_callback_group)
        self.declare_parameter("recording_delay", 5.0)

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted MISSION process")
                self.mission_process = None
            delay = float(self.get_parameter("recording_delay").value or 5.0)
            self.get_logger().info(f"AVState END reached. Delaying stop recording by {delay} seconds.")
            self.create_timer(delay, self.delayed_stop_recording)

    def delayed_stop_recording(self):
        request = Stop.Request()
        self.stop_record_future = self.bag_stop_cli.call_async(request)
        self.trigger_shutdown()

    def trigger_shutdown(self):
        while self.stop_record_future is not None:
            continue
        self.get_logger().error("Exit Node - Shutdown Triggered")
        self.destroy_node()
        exit(1)

    def start_recording(self, target_mission: str):
        while not self.bag_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'rosbag2_recorder/record' not available, waiting...")

        now = datetime.datetime.now()
        name = f'bags/{target_mission}-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
        request = Record.Request()
        request.uri = name
        self.record_future = self.bag_start_cli.call_async(request)
        self.get_logger().info("Recording requested")

    def start_record_callback(self) -> bool:
        assert self.record_future is not None
        result = self.record_future.result()
        self.get_logger().info(f"Service Call result: {str(result)}")
        if result is None:
            self.get_logger().error(f"Service ERROR, recording may not have started: {self.record_future.exception()}")
        elif getattr(result, "return_code", -1) == 0:
            self.get_logger().info("Recording started")
        else:
            self.get_logger().error(f"Recording failed to start: {getattr(result, 'error_string', '')}")
        success = result is not None and getattr(result, "return_code", -1) == 0
        self.get_logger().info(f"Returning {success}")
        return success

    def stop_future_callback(self) -> bool:
        assert self.stop_record_future is not None
        result = self.stop_record_future.result()
        self.get_logger().info(f"Service Call result: {str(result)}")
        success = result is not None
        if result is None:
            self.get_logger().error(
                f"Service ERROR, recording may still be running: {self.stop_record_future.exception()}"
            )
        else:
            self.get_logger().info("Recording stopped")
        self.get_logger().info(f"Returning {success}")
        return success

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.record_future is not None and self.record_future.done():
                self.start_record_callback()
                self.record_future = None
            if self.stop_record_future is not None and self.stop_record_future.done():
                self.stop_future_callback()
                self.stop_record_future = None
