from subprocess import Popen
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Shutdown, State

from driverless_common.common import QOS_LATEST
from driverless_common.shutdown_node import ShutdownNode


class InspectionHandler(Node):
    mission_started = False
    start_time = 0.0
    process = None

    def __init__(self):
        super().__init__("inspection_logic_node")
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.reset_sub = self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)
        self.shutdown_pub: Publisher = self.create_publisher(Shutdown, "/system/shutdown", 1)

        self.get_logger().info("---Inspection handler node initialised---")

    def state_callback(self, msg: State):
        # super().state_callback(msg)
        if (
            msg.state == State.DRIVING
            and msg.mission == State.INSPECTION
            and msg.navigation_ready
            and not self.mission_started
        ):
            self.mission_started = True
            self.start_time = time.time()
            command = ["stdbuf", "-o", "L", "ros2", "launch", "mission_controller", "inspection.launch.py"]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Trackdrive mission started")

    def timer_callback(self):
        if self.mission_started and time.time() - self.start_time > 30:
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)
            self.get_logger().info("Inspection mission finished")
            self.get_logger().warn("Closing Inspection mission")
            self.process.terminate()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
