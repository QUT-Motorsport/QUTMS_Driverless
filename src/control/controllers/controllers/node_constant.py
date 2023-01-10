import rclpy
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped

from driverless_common.shutdown_node import ShutdownNode

TARGET_STEERING = "target_steering"
TARGET_ACCELERATION = "target_acceleration"


class ConstantController(ShutdownNode):
    pub_interval: float = 0.2

    def __init__(self):
        super().__init__("constant_controller")

        self.declare_parameter(TARGET_STEERING, 0.0)
        self.declare_parameter(TARGET_ACCELERATION, 0.0)

        # timed callback
        self.create_timer(self.pub_interval, self.pub_callback)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "driving_command", 1)
        self.accel_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "accel_command", 1)

    def pub_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.drive.steering_angle = self.get_parameter(TARGET_STEERING).get_parameter_value().double_value
        control_msg.drive.acceleration = self.get_parameter(TARGET_ACCELERATION).get_parameter_value().double_value
        self.drive_publisher.publish(control_msg)
        self.accel_publisher.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConstantController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
