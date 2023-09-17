import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecyclePublisher, LifecycleState, LifecycleNodeMixin, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from driverless_msgs.msg import PathStamped

from driverless_common.common import QOS_LATEST
from pure_pursuit.node_pure_pursuit import PurePursuit


class PurePursuitLifecycle(PurePursuit, LifecycleNodeMixin):
    """
    
    """

    path_sub: Subscription
    control_pub: LifecyclePublisher
    debug_pub: LifecyclePublisher
    timer: Timer

    def __init__(self, node_name="pure_pursuit_lifecycle"):
        """
        Overrides __init__ in PurePursuit for lifecycle based pub/sub configuration
        """
        # initialise base node but not pure pursuit - so we dont get its params and pubs/subs
        Node.__init__(self, node_name)
        LifecycleNodeMixin.__init__(self, enable_communication_interface=True)

        self.initialise_params()

        if node_name == "pure_pursuit_lifecycle":
            self.get_logger().info("---Lifecycle Pure pursuit follower initialised---")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.control_pub = self.create_lifecycle_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        if self.DEBUG_IMG:
            self.debug_pub = self.create_lifecycle_publisher(Image, "/debug_imgs/pursuit_img", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        self.path_sub = self.create_subscription(PathStamped, "/planner/path", self.path_callback, QOS_LATEST)
        self.timer = self.create_timer((1 / 50), self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuitLifecycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
