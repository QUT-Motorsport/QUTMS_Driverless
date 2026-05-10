import datetime
import signal
from subprocess import Popen

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped

from driverless_msgs.srv import IsRecording
from rosbag2_interfaces.srv import Record, Stop

from driverless_common.common import QOS_LATEST

from typing import Any, Optional


class ShutdownNode(Node):
    mission_process: Optional[Popen] = None
    recording: Optional[Any] = None
    record_future: Optional[Any] = None
    stop_record_future: Optional[Any] = None
    is_recording_future: Optional[Any] = None
    pending_record_uri: Optional[str] = None
    pending_stop_check: bool = False
    shutdown_after_stop: bool = False

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli_callback_group = ReentrantCallbackGroup()
        self.bag_stop_cli = self.create_client(Stop, "rosbag2_recorder/stop", callback_group=self.cli_callback_group)
        self.bag_start_cli = self.create_client(
            Record, "rosbag2_recorder/record", callback_group=self.cli_callback_group
        )
        self.bag_is_recording_cli = self.create_client(
            IsRecording, "rosbag_creator/is_recording", callback_group=self.cli_callback_group
        )
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
        while not self.bag_is_recording_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'rosbag_creator/is_recording' not available, waiting...")
        self.pending_stop_check = True
        self._query_is_recording()

    def trigger_shutdown(self):
        self.get_logger().error("Exit Node - Shutdown Triggered")
        self.destroy_node()
        rclpy.shutdown()
        exit(1)

    def start_recording(self, target_mission: str):
        while not self.bag_start_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'rosbag2_recorder/record' not available, waiting...")
        while not self.bag_is_recording_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'rosbag_creator/is_recording' not available, waiting...")

        now = datetime.datetime.now()
        name = f'bags/{target_mission}-{now.strftime("%Y-%m-%d-%H-%M-%S")}'
        self.pending_record_uri = name
        self._query_is_recording()

    def _query_is_recording(self):
        if self.is_recording_future is not None and not self.is_recording_future.done():
            self.get_logger().warning("IsRecording request already in progress")
            return
        self.is_recording_future = self.bag_is_recording_cli.call_async(IsRecording.Request())

    def _is_recording_callback(self):
        assert self.is_recording_future is not None
        result = self.is_recording_future.result()
        if result is None:
            self.get_logger().error(
                f"Service ERROR, failed to query recording state: {self.is_recording_future.exception()}"
            )
            self.pending_record_uri = None
            self.pending_stop_check = False
            return

        if self.pending_record_uri is not None:
            if result.recording:
                self.get_logger().warning("Recording already active. Skipping start request.")
            else:
                request = Record.Request()
                request.uri = self.pending_record_uri
                self.record_future = self.bag_start_cli.call_async(request)
                self.get_logger().info("Recording requested")
            self.pending_record_uri = None

        if self.pending_stop_check:
            if result.recording:
                request = Stop.Request()
                self.stop_record_future = self.bag_stop_cli.call_async(request)
                self.shutdown_after_stop = True
            else:
                self.get_logger().info("Recorder already stopped")
                self.trigger_shutdown()
            self.pending_stop_check = False

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
        if self.shutdown_after_stop:
            self.shutdown_after_stop = False
            self.trigger_shutdown()
        self.get_logger().info(f"Returning {success}")
        return success

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.is_recording_future is not None and self.is_recording_future.done():
                self._is_recording_callback()
                self.is_recording_future = None
            if self.record_future is not None and self.record_future.done():
                self.start_record_callback()
                self.record_future = None
            if self.stop_record_future is not None and self.stop_record_future.done():
                self.stop_future_callback()
                self.stop_record_future = None
