import rclpy
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive

from .base_mission_class import BaseMission


class EBSTestMission(BaseMission):
    def __init__(self):
        super().__init__()

        # self.create_subscription(Can, "/can_rosbound", self.can_callback, 10)
        self.create_subscription(AckermannDrive, "/reactive_driving_command", self.reactive_callback, 1)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

    def reactive_callback(self, reactive_control_msg: AckermannDrive):
        if self.ebs_ready:
            self.drive_publisher.publish(reactive_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EBSTestMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
