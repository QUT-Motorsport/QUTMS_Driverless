import signal
from subprocess import Popen

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import State

from driverless_common.common import QOS_LATEST
from driverless_common.status_constants import INT_MISSION_TYPE


class MissionControl(Node):
    mission_launched: bool = False
    process = None

    def __init__(self):
        super().__init__("mission_control_node")

        self.create_subscription(State, "/system/as_status", self.callback, QOS_LATEST)

        self.get_logger().info("---Mission control node initialised---")

    def callback(self, status: State):
        if status.mission != State.MISSION_NONE and status.state != State.EMERGENCY and not self.mission_launched:
            target_mission = INT_MISSION_TYPE[status.mission].value
            node = target_mission + "_handler_node"
            command = ["stdbuf", "-o", "L", "ros2", "run", "mission_controller", node]

            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Mission started: " + target_mission)
            self.mission_launched = True

        elif (status.mission == State.MISSION_NONE or status.state == State.EMERGENCY) and self.mission_launched:
            self.get_logger().warn("Closing mission")
            self.process.send_signal(signal.SIGINT)
            self.mission_launched = False


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.process.terminate()
    node.destroy_node()
    rclpy.shutdown()
