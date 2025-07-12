import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class TwistPublisher(Node):
    def __init__(self):
        super().__init__("twist_publisher")
        self.publisher_ = self.create_publisher(TwistStamped, "/bicycle_steering_controller/reference", 10)
        self.timer = self.create_timer(1 / 30, self.publish_twist)  # 30 Hz
        self.counter = 0

    def publish_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"

        # Example: Modifying the values dynamically
        twist_msg.twist.linear.x = 1.0  # Constant forward speed
        twist_msg.twist.angular.z = 0.1 if self.counter % 2 == 0 else -0.1  # Change direction every cycle

        self.publisher_.publish(twist_msg)
        self.get_logger().info(
            f"Publishing: Linear X = {twist_msg.twist.linear.x}, Angular Z = {twist_msg.twist.angular.z}"
        )

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
