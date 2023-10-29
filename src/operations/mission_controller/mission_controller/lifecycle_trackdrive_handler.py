import time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from driverless_msgs.msg import Shutdown, State
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import UInt8

from driverless_common.lifecycle_service_client import LifecycleServiceClient
from driverless_common.shutdown_node import ShutdownNode


class TrackdriveHandler(ShutdownNode):
    mission_started = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    init_method_called = False
    crossed_start = False
    goal_offet = 5.0

    def __init__(self):
        super().__init__("trackdrive_logic_node")

        self.declare_parameter("start_following", False)

        self.create_timer((1 / 20), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.shutdown_pub = self.create_publisher(Shutdown, "/system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "/system/laps_completed", 1)

        # this changes based on which implementation we're using
        self.pure_pursuit = LifecycleServiceClient("pure_pursuit_lifecycle", self)
        self.reactive_controller = LifecycleServiceClient("reactive_controller_lifecycle", self)
        # self.planner = LifecycleServiceClient("ordered_mid_spline_lifecycle", self)

        if self.get_parameter("start_following").value:
            # start at lap 1
            self.get_logger().warn("---DEBUG MODE ENABLED---")
            self.crossed_start = True

        self.get_logger().info("---Trackdrive handler node initialised---")

    def state_callback(self, msg: State):
        super().state_callback(msg)
        if not self.mission_started and msg.state == State.DRIVING and msg.mission == State.TRACKDRIVE:
            self.mission_started = True
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=self.laps))
            self.reactive_controller.activate()
            self.get_logger().info("Trackdrive mission started")

    def timer_callback(self):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        try:
            track_to_base = self.tf_buffer.lookup_transform(
                "track", "base_footprint", rclpy.time.Time(seconds=0)
            )
        except TransformException as e:
            self.get_logger().debug("Transform exception: " + str(e))
            return

        if not self.mission_started:
            self.last_x = track_to_base.transform.translation.x
            return

        if not abs(track_to_base.transform.translation.y) < 3:
            self.last_x = track_to_base.transform.translation.x
            return

        if self.last_x <= self.goal_offet and track_to_base.transform.translation.x > self.goal_offet:
            if not self.crossed_start:
                self.crossed_start = True
                self.last_lap_time = time.time()
                self.get_logger().info("Crossed start line")
                # will need to go around again to reset last x
                self.last_x = track_to_base.transform.translation.x
                return

            if not (time.time() - self.last_lap_time > 3):  # seconds at least between laps
                # will need to go around again to reset last x
                self.last_x = track_to_base.transform.translation.x
                return

            self.laps += 1
            self.lap_trig_pub.publish(UInt8(data=self.laps))
            self.get_logger().info(f"Lap {self.laps} completed")
            if self.laps == 1:
                self.reactive_controller.deactivate()
                # self.planner.activate()
                self.pure_pursuit.activate()

            self.last_x = track_to_base.transform.translation.x
            self.last_lap_time = time.time()

        if self.laps == 10:
            self.get_logger().info("Trackdrive mission complete")
            self.pure_pursuit.deactivate()
            self.pure_pursuit.shutdown()
            # self.planner.deactivate()
            # self.planner.shutdown()
            self.reactive_controller.shutdown()

            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
