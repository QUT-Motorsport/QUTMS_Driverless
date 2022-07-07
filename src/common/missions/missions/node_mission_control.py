
# import custom message libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
# other python modules
import time
# import custom message libraries
from driverless_msgs.msg import Can, SelectMission

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")

        self.create_subscription(Can, "/can_rosbound", self.callback, 10)

        self.srv = self.create_service(SelectMission, 'select_mission', self.mission_callback)


        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

        self.get_logger().info("---Mission Control node initialised---")

        self.target_mission = None


    def mission_callback(self, request, response):
        self.get_logger().info('Incoming request\n%d', request)
        self.target_mission = request

        response.confirmation = "Mission confirmed: " + str(self.target_mission)

        return response


    def callback(self, can_msg: Can):
        None
        if can_msg.id == 'mission_selection':
            mission = can_msg.data
            if mission == 0: #1, 2, 3
                self.target_mission = 'manual_driving'
        # first, listen to CAN steering wheel buttons to select mission from steering wheel display
        # check for ID of steering wheel data
        # save target mission

        # next, listen to the RES 'start' button CAN msg. 
        # send can msg to EBS VCU
            # VCU side: go thru ebs checks
        
        # then, listen to EBS check success
            # else its bad, dont continue

        # delay 5s
        # send 'go' ros message to control and perception nodes


def main(args=None):
    # begin ros node
    rclpy.init(args=args)

    node = MissionControl()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()