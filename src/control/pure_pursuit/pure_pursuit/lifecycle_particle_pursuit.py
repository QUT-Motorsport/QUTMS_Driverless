import rclpy
from rclpy.lifecycle import LifecycleNodeMixin, LifecyclePublisher, LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import ConeDetectionStamped, PathStamped
from sensor_msgs.msg import Image

from driverless_common.common import QOS_LATEST
from pure_pursuit.node_particle_pursuit import ParticlePursuit


class ParticlePursuitLifecycle(ParticlePursuit, LifecycleNodeMixin):
    """ """

    cone_sub: Subscription
    # copied from base class
    path_sub: Subscription
    control_pub: LifecyclePublisher
    debug_pub: LifecyclePublisher
    timer: Timer

    def __init__(self):
        """
        Overrides __init__ in ParticlePursuit for lifecycle based pub/sub configuration
        """
        # initialise base node but not pure pursuit - so we dont get its params and pubs/subs
        Node.__init__(self, node_name="particle_pursuit_lifecycle")
        LifecycleNodeMixin.__init__(self, enable_communication_interface=True)

        self.initialise_params()

        self.get_logger().info("---Lifecycle Pure pursuit follower initialised---")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.control_pub = self.create_lifecycle_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        if self.DEBUG_IMG:
            self.debug_pub = self.create_lifecycle_publisher(Image, "/debug_imgs/pursuit_img", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        self.cone_sub = self.create_subscription(
            ConeDetectionStamped, "/planner/interpolated_map", self.interp_track_callback, 10
        )
        # copied from base class
        self.path_sub = self.create_subscription(PathStamped, "/planner/path", self.path_callback, QOS_LATEST)
        self.timer = self.create_timer((1 / 50), self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.cone_sub)
        # copied from base class
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.cone_sub)
        # copied from base class
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.cone_sub)
        # copied from base class
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.cone_sub)
        # copied from base class
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = ParticlePursuitLifecycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
