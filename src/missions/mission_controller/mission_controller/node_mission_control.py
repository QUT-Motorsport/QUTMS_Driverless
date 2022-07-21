import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Can

from driverless_msgs.srv import SelectMission, StartControl

from typing import List

missions: List[str] = ["manual_driving", "inspection", "ebs_test", "trackdrive"]


class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.create_subscription(Can, "/can_rosbound", self.callback, 10)
        self.create_service(SelectMission, "select_mission", self.gui_srv)

        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

        self.target_mission: str = None
        self.mission_start: bool = False
        self.r2d: bool = False

        self.client = self.create_client(StartControl, "start_control")

        self.send_srv = StartControl.Request()
        self.future = None

        self.get_logger().info("---Mission Control node initialised---")

    def gui_srv(self, request, response):  # service callback from terminal selection
        self.get_logger().info("Selected mission: " + request.mission)
        self.target_mission = request.mission
        return response

    def callback(self, can_msg: Can):
        # first, listen to CAN steering wheel buttons to select mission from steering wheel display
        # check for ID of steering wheel data
        # save target mission
        if can_msg.id == "mission_selection":  # idk about what can IDs are
            mission: int = can_msg.data  # extract msg data
            if mission >= 0 and mission < 4:  # check if its an actual value
                self.target_mission = missions[mission]

        # next, listen to the RES 'start' button CAN msg.
        # send can msg to EBS VCU
        # VCU side: go thru ebs checks
        if can_msg.id == 600 and self.target_mission is not None:
            self.get_logger().info("Received can msg for start")
            if can_msg.data[0]:
                self.get_logger().info("Start confirmed")
                self.mission_start = True
                # out_can = Can()
                # out_can.id = "check_ebs"
                # out_can.data = 1
                # # any other CAN data??
                # self.publisher.publish(out_can)
                print(str(self.target_mission))  # triggers a 'I/O' event in the launch file

        # then, listen to EBS check success
        # else its bad, dont continue
        # delay 5s
        # send 'go' ros message to control and perception nodes
        if can_msg.id == 601 and self.target_mission is not None and self.mission_start:
            self.get_logger().info("Received can msg for ebs ready")
            if can_msg.data[0]:
                self.get_logger().info("EBS ready confirmed")
                self.r2d = True
                # out_can = Can()
                # out_can.id = "r2d"
                # out_can.data = 1
                # # any other CAN data??
                # self.publisher.publish(out_can)
                # this should send srv to start controlling

                self.send_srv.start = True
                self.future = self.client.call_async(self.send_srv)

                rclpy.spin_until_future_complete(self, self.future)
                response = self.future.result()


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = MissionControl()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
