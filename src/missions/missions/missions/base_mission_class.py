from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Can


class BaseMission(Node):
    start_pressed: bool = False
    ebs_ready: bool = False

    def __init__(self):
        super().__init__("mission")

        self.create_subscription(Can, "/can_rosbound", self.can_callback, 10)

        self.publisher: Publisher = self.create_publisher(Can, "/can_carbound", 10)

    def can_callback(self, can_msg: Can):
        # listen to EBS check success
        # else its bad, dont continue
        # delay 5s
        # send 'go' ros message to control and perception nodes
        if can_msg.id == 600:
            self.get_logger().info("HELLOW???")
            if can_msg.data[0]:
                self.get_logger().info("EBS is ready")
                self.ebs_ready = True
                # out_can = Can()
                # out_can.id = "r2d"
                # out_can.data = 1
                # # any other CAN data??
                # self.publisher.publish(out_can)
                # this should send srv to start controlling

        # next, listen to the RES 'start' button CAN msg.
        # send can msg to EBS VCU
        # VCU side: go thru ebs checks
        if can_msg.id == 601 and self.ebs_ready and not self.start_pressed:
            if can_msg.data[0]:
                self.get_logger().info("Start pressed")
                self.start_pressed = True
                # out_can = Can()
                # out_can.id = "check_ebs"
                # out_can.data = 1
                # # any other CAN data??
                # self.publisher.publish(out_can)
