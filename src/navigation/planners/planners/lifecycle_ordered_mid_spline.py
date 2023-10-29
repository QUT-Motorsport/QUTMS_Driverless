import rclpy
from rclpy.lifecycle import LifecycleNodeMixin, LifecycleState, Publisher, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from driverless_msgs.msg import ConeDetectionStamped
from driverless_msgs.msg import PathStamped as QUTMSPathStamped
from nav_msgs.msg import Path

from driverless_common.common import QOS_LATEST
from planners.node_ordered_mid_spline import OrderedMapSpline


class OrderedMidSplineLifecycle(OrderedMapSpline, LifecycleNodeMixin):
    map_sub: Subscription
    planning_timer: Timer
    qutms_path_pub: Publisher
    planned_path_pub: Publisher
    interp_cones_pub: Publisher

    def __init__(self):
        """
        Overrides __init__ in ParticlePursuit for lifecycle based pub/sub configuration
        """
        # initialise base node but not pure pursuit - so we dont get its params and pubs/subs
        Node.__init__(self, node_name="ordered_mid_spline_lifecycle")
        LifecycleNodeMixin.__init__(self, enable_communication_interface=True)

        self.get_logger().info("---Lifecycle Ordered Mid Spline Initialised---")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.qutms_path_pub = self.create_lifecycle_publisher(QUTMSPathStamped, "/planner/path", 1)
        self.planned_path_pub = self.create_lifecycle_publisher(Path, "/planner/spline_path", 1)
        self.interp_cones_pub = self.create_lifecycle_publisher(ConeDetectionStamped, "/planner/interpolated_map", 1)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        self.map_sub = self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.map_callback, QOS_LATEST)
        self.planning_timer = self.create_timer(0.1, self.planning_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.map_sub)
        self.destroy_timer(self.planning_timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.map_sub)
        self.destroy_timer(self.planning_timer)
        self.destroy_lifecycle_publisher(self.qutms_path_pub)
        self.destroy_lifecycle_publisher(self.planned_path_pub)
        self.destroy_lifecycle_publisher(self.interp_cones_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.map_sub)
        self.destroy_timer(self.planning_timer)
        self.destroy_lifecycle_publisher(self.qutms_path_pub)
        self.destroy_lifecycle_publisher(self.planned_path_pub)
        self.destroy_lifecycle_publisher(self.interp_cones_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.map_sub)
        self.destroy_lifecycle_publisher(self.qutms_path_pub)
        self.destroy_lifecycle_publisher(self.planned_path_pub)
        self.destroy_lifecycle_publisher(self.interp_cones_pub)
        return TransitionCallbackReturn.SUCCES


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = OrderedMidSplineLifecycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
