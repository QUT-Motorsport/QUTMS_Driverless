from subprocess import Popen

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import State

from driverless_common.status_constants import INT_MISSION_TYPE
from driverless_common.common import QOS_LATEST


class MissionControl(Node):
    mission_launched: bool = False
    process = None

    def __init__(self):
        super().__init__("mission_control_node")

        self.create_subscription(State, "/system/as_status", self.callback, QOS_LATEST)

        self.get_logger().info("---Mission control node initialised---")

    def callback(self, status: State):
        if status.mission != State.MISSION_NONE and not self.mission_launched:
            target_mission = INT_MISSION_TYPE[status.mission].value
            launch_file = target_mission + ".launch.py"
            command = ["stdbuf", "-o", "L", "ros2", "launch", "mission_controller", launch_file]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Mission started: " + target_mission)
            self.mission_launched = True


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.process.terminate()
    node.destroy_node()
    rclpy.shutdown()
