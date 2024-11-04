import asyncio
import multiprocessing
import signal
from subprocess import Popen

from launch import LaunchDescription, LaunchService

from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped

from driverless_common.common import QOS_LATEST


class Ros2LaunchParent:
    def start(self, launch_description: LaunchDescription):
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process, args=(self._stop_event, launch_description), daemon=True
        )
        self._process.start()

    def shutdown(self):
        self._stop_event.set()
        self._process.join()

    def _run_process(self, stop_event, launch_description):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)


class ShutdownNode(Node):
    mission_process: Popen = None
    launch_parent: Ros2LaunchParent = None

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(AVStateStamped, "system/av_state", self.av_state_callback, QOS_LATEST)

    def av_state_callback(self, msg: AVStateStamped):
        if msg.state in [AVStateStamped.END]:
            if self.mission_process is not None:
                self.mission_process.send_signal(signal.SIGINT)
                self.get_logger().error("Interrupted process")

                self.mission_process = None
            if self.launch_parent is not None:
                self.launch_parent.shutdown()
                self.get_logger().error("Shutdown Launch Parent")
            self.get_logger().error("Exit Node - Shutdown Triggered")
            self.destroy_node()
            exit(1)
