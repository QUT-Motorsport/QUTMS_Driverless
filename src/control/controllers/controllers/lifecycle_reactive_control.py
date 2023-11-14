import rclpy
from rclpy.lifecycle import LifecycleNodeMixin, LifecyclePublisher, LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.subscription import Subscription

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import ConeDetectionStamped

from controllers.node_reactive_control import ReactiveController
from driverless_common.common import QOS_ALL, fast_dist, midpoint

from typing import Tuple

Colour = Tuple[int, int, int]

ORIGIN = [0, 0]


class ReactiveControllerLifecycle(ReactiveController, LifecycleNodeMixin):
    cone_sub: Subscription
    control_pub: LifecyclePublisher

    def __init__(self):
        """
        Overrides __init__ in ReactiveController for lifecycle based pub/sub configuration
        """

        Node.__init__(self, node_name="reactive_controller_lifecycle")
        LifecycleNodeMixin.__init__(self, enable_communication_interface=True)

        self.initialise_params()

        self.get_logger().info("---Reactive controller node initialised---")

    def can_drive(self):
        # override this function to always return true as conditions are lifecycle managed
        self.get_logger().info("Can drive", once=True)
        return True

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.control_pub = self.create_lifecycle_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        topic = "/lidar/cone_detection" if self.ebs_test else "/slam/local_map"
        self.cone_sub = self.create_subscription(ConeDetectionStamped, topic, self.callback, QOS_ALL)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.cone_sub)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.cone_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.cone_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.cone_sub)
        self.destroy_lifecycle_publisher(self.control_pub)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveControllerLifecycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
