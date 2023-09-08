import time

from driverless_common.lifecycle_service_client import LifecycleServiceClient
from driverless_msgs.msg import Shutdown, State

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8


class TrackdriveHandler(Node):
    mission_started = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    init_method_called = False

    def __init__(self):
        super().__init__("trackdrive_logic_node")
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.pose_callback, 10)

        self.shutdown_pub = self.create_publisher(Shutdown, "/system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "/system/laps_completed", 1)

        self.pure_pursuit_client = LifecycleServiceClient("pure_pursuit_cpp_node", self)
        self.reactive_controller = LifecycleServiceClient("reactive_controller_node", self)

        self.timer = self.create_timer(1, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.init_method = self.create_timer(1, self.init_method_callback,
                                             callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("---Trackdrive handler node initialised---")

    # Services cannot be activated in the constructor, activating clients that would be activated in the constructor
    # should be done here, this timer's callback is called once and then the timer is destroyed.
    def init_method_callback(self):
        self.get_logger().info("init_method_callback")

        self.reactive_controller.activate()

        self.destroy_timer(self.init_method)
        self.init_method_called = True

    def timer_callback(self):
        if not self.init_method_called:
            return
        # Use to periodically check on lifecycle nodes
        if not self.pure_pursuit_client.is_in_expected_state():
            self.pure_pursuit_client.change_to_expected_state()
        if not self.reactive_controller.is_in_expected_state():
            self.reactive_controller.change_to_expected_state()

    def state_callback(self, msg: State):
        if not self.mission_started and msg.state == State.DRIVING and msg.mission == State.TRACKDRIVE:
            self.mission_started = True
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=self.laps))
            self.get_logger().info("Trackdrive mission started")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        if self.last_x <= 0 < msg.pose.pose.position.x and abs(msg.pose.pose.position.y) < 2:
            self.laps += 1
            self.get_logger().info(f"{time.time() - self.last_lap_time}")
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=self.laps))
            self.get_logger().info(f"Lap {self.laps - 1} completed")

            if self.laps == 2:
                self.reactive_controller.deactivate()
                self.pure_pursuit_client.activate()
            elif self.laps == 11:
                self.get_logger().info("Trackdrive mission complete")
                self.pure_pursuit_client.deactivate()
                self.pure_pursuit_client.shutdown()
                self.reactive_controller.shutdown()

                # currently only works when vehicle supervisor node is running on-car
                # TODO: sort out vehicle states for eventual environment agnostic operation
                shutdown_msg = Shutdown(finished_engage_ebs=True)
                self.shutdown_pub.publish(shutdown_msg)

        self.last_x = msg.pose.pose.position.x


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
