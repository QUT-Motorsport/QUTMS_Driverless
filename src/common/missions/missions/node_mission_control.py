
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from driverless_msgs.msg import Can
from driverless_msgs.srv import SelectMission

import time
from typing import List, Tuple

missions: List[str] = ["manual_driving", "inspection", "ebs_test", "trackdrive"]

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.create_subscription(Can, "/can_rosbound", self.callback, 10)

        self.srv = self.create_service(SelectMission, 'select_mission', self.terminal_srv)


        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

        self.get_logger().info("---Mission Control node initialised---")

        self.target_mission: str = None
        self.mission_start: bool = False
        self.r2d: bool = False


    def terminal_srv(self, request, response): # service callback from terminal selection
        self.get_logger().info('Incoming request\n%d', request)
        self.target_mission = request

        response.confirmation = "Mission confirmed: " + str(self.target_mission)
        return response


    def callback(self, can_msg: Can):
        # first, listen to CAN steering wheel buttons to select mission from steering wheel display
        # check for ID of steering wheel data
        # save target mission
        if can_msg.id == "mission_selection": # idk about what can IDs are
            mission: int = can_msg.data # extract msg data
            if mission >= 0 and mission < 4: # check if its an actual value
                self.target_mission = missions[mission]
        
        # next, listen to the RES 'start' button CAN msg. 
        # send can msg to EBS VCU
            # VCU side: go thru ebs checks
        if can_msg.id == "res_start":
            start_confirm: bool = can_msg.data
            if start_confirm: 
                self.mission_start = True
                out_can = Can()
                out_can.id = "check_ebs"
                out_can.data = 1
                # any other CAN data??
                self.publisher.publish(out_can)

        # then, listen to EBS check success
            # else its bad, dont continue
        # delay 5s
        # send 'go' ros message to control and perception nodes
        if can_msg.id == "ebs_ready":
            ebs_ready: bool = can_msg.data
            if ebs_ready: 
                time.sleep(5) # probably a better way to pause
                self.r2d = True
                out_can = Can()
                out_can.id = "r2d"
                out_can.data = 1
                self.publisher.publish(out_can)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = MissionControl()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()