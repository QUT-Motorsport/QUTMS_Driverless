from subprocess import Popen

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import State
from std_msgs.msg import UInt8

from driverless_common.status_constants import INT_MISSION_TYPE


class MissionControl(Node):
    mission_launched: bool = False
    process = None

    def __init__(self):
        super().__init__("mission_control_node")

        self.create_subscription(State, "/system/as_status", self.callback, 10)
        self.create_subscription(UInt8, "/system/mission_select", self.select_callback, 10)

        self.get_logger().info("---Mission control node initialised---")

    def callback(self, status: State):
        if status.state == State.SELECT_MISSION:
            self.mission_launched = False
        if status.mission != State.MISSION_NONE and not self.mission_launched:
            self.launch_mission(status.mission)

    def select_callback(self, msg: UInt8):
        if msg.data == State.MISSION_NONE:
            self.mission_launched = False
        if msg.data != State.MISSION_NONE and not self.mission_launched:
            self.launch_mission(msg.data)

    def launch_mission(self, mission_id: int):
        target_mission = INT_MISSION_TYPE[mission_id].value
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
