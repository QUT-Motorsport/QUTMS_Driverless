from ament_index_python import get_package_share_directory
from ros2launch.api.api import launch_a_launch_file

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import State

from driverless_common.status_constants import INT_MISSION_TYPE

mission_pkg = get_package_share_directory("mission_controller")  # path to the mission package


class MissionControl(Node):
    mission_launched: bool = False

    def __init__(self):
        super().__init__("mission_control_node")

        self.create_subscription(State, "/system/as_status", self.callback, 10)

        self.get_logger().info("---Mission control node initialised---")

    def callback(self, status: State):
        if status.state == State.SELECT_MISSION:
            self.mission_launched = False

        if status.mission != State.MISSION_NONE and not self.mission_launched:
            target_mission = INT_MISSION_TYPE[status.mission].value
            self.get_logger().info("Mission started: " + target_mission)
            self.mission_launched = True
            launch_a_launch_file(
                launch_file_path=(mission_pkg + "/" + target_mission + ".launch.py"), launch_file_arguments={}
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
