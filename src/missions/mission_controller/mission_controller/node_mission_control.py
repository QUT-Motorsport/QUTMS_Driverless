import time

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Can, Reset, State

from driverless_msgs.srv import SelectMission

from .mission_constants import CAN_TO_MISSION_TYPE


class MissionControl(Node):
    target_mission: str

    def __init__(self):
        super().__init__("mission_control")

        self.create_subscription(Can, "/can_rosbound", self.callback, 10)

        self.publisher = self.create_publisher(State, "/state", 10)
        self.publisher = self.create_publisher(Reset, "/reset", 10)

        self.create_service(SelectMission, "select_mission", self.gui_srv)

        self.get_logger().info("---Mission Control node initialised---")

    def gui_srv(self, request, response):  # service callback from terminal selection
        self.get_logger().info("Selected mission: " + request.mission)
        self.target_mission = request.mission
        print(str(self.target_mission))  # triggers a 'I/O' event in the launch file
        return response

    def callback(self, can_msg: Can):
        # first, listen to CAN steering wheel buttons to select mission from steering wheel display
        # check for ID of steering wheel data
        # save target mission
        if can_msg.data == "mission_selection":  # idk about what can IDs are
            mission: int = can_msg.data  # extract msg data
            if mission in CAN_TO_MISSION_TYPE:  # check if its an actual value
                self.target_mission = CAN_TO_MISSION_TYPE[mission]
                print(str(self.target_mission))  # triggers a 'I/O' event in the launch file
                # WE WANT TO CHANGE THIS TO A SUBPROCESS?

        if can_msg.data == "ready_to_drive":
            self.get_logger().info("Ready to drive")
            self.publisher.publish(State(r2d=True))
            self.publisher.publish(Reset(reset=True))
            # START TRACTIVE SYSTEM CONTROLLER AS SUBPROCESS?


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
