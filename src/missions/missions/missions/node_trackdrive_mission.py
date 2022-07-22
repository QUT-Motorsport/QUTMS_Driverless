import rclpy
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive

from .base_mission_class import BaseMission


class TrackdriveMission(BaseMission):
    loop_closed: bool = False

    def __init__(self):
        super().__init__()

        self.create_subscription(AckermannDrive, "/reactive_driving_command", self.reactive_callback, 1)
        self.create_subscription(AckermannDrive, "/following_driving_command", self.following_callback, 1)

        # maybe a loop closure service call? or topic like reset

        self.drive_publisher: Publisher = self.create_publisher(AckermannDrive, "/driving_command", 1)

    # logic for switching reactive and following drive commands needed
    def reactive_callback(self, reactive_control_msg: AckermannDrive):
        if self.ebs_ready and not self.loop_closed:
            self.drive_publisher.publish(reactive_control_msg)

    def following_callback(self, following_control_msg: AckermannDrive):
        if self.ebs_ready and self.loop_closed:
            self.drive_publisher.publish(following_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
