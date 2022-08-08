import rclpy
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive

from .base_mission_class import BaseMission


class InspectionMission(BaseMission):
    def __init__(self):
        super().__init__()

        self.create_subscription(AckermannDrive, "/sine_driving_command", self.sine_callback, 1)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

    def sine_callback(self, inspection_control_msg: AckermannDrive):
        if self.r2d:
            self.drive_publisher.publish(inspection_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
