import time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Shutdown, State
from std_msgs.msg import UInt8

from driverless_common.shutdown_node import ShutdownNode


class TrackdriveHandler(ShutdownNode):
    mission_started = False
    crossed_start = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    goal_offet = 5.0

    def __init__(self):
        super().__init__("trackdrive_logic_node")

        self.declare_parameter("start_following", False)

        self.create_subscription(State, "system/as_status", self.state_callback, 1)

        self.create_timer((1 / 20), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.shutdown_pub = self.create_publisher(Shutdown, "system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "system/laps_completed", 1)

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
            self.lap_trig_pub.publish(UInt8(data=0))
            self.get_logger().info("Trackdrive mission started")

    def timer_callback(self):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        try:
            track_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time(seconds=0))
        except TransformException as e:
            self.get_logger().debug("Transform exception: " + str(e))
            return

        if not self.mission_started:
            self.last_x = track_to_base.transform.translation.x
            return

        if not abs(track_to_base.transform.translation.y) < 2:
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

            self.last_x = track_to_base.transform.translation.x
            self.last_lap_time = time.time()

        if self.laps == 10:
            self.get_logger().info("Trackdrive mission complete")
            # currently only works when vehicle supervisor node is running on-car
            # TODO: sort out vehicle states for eventual environment agnostic operation
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
