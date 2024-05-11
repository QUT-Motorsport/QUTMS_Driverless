import os
from subprocess import PIPE, Popen
import time

from nav2_msgs.action import FollowPath
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.action import ActionClient

from driverless_msgs.msg import Shutdown, State
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, UInt8

from std_srvs.srv import Trigger

from driverless_common.shutdown_node import ShutdownNode


class TrackdriveHandler(ShutdownNode):
    mission_started = False
    odom_received = False
    crossed_start = False
    sent_init = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    goal_offet = 0.1
    path = None
    goal_handle = None

    def __init__(self):
        super().__init__("trackdrive_logic_node")

        self.declare_parameter("start_following", False)
        self.declare_parameter("sim", False)

        self.create_subscription(State, "system/as_status", self.state_callback, 1)
        self.create_subscription(Path, "planning/midline_path", self.path_callback, 1)
        self.create_subscription(Odometry, "imu/odometry", self.odom_callback, 1)

        if not self.get_parameter("sim").value:
            # reset odom and pose from camera
            self.reset_odom_client = self.create_client(Trigger, "zed2i/zed_node/reset_odometry")
            self.reset_pose_client = self.create_client(Trigger, "zed2i/zed_node/reset_pos_tracking")

            while not self.reset_odom_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("reset_odom_client service not available, waiting again...")
            while not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("reset_pose_client service not available, waiting again...")

        self.create_timer((1 / 20), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.shutdown_pub = self.create_publisher(Shutdown, "system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "system/laps_completed", 1)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        # actions
        self.nav_through_poses_client = ActionClient(self, FollowPath, "follow_path")

        if self.get_parameter("start_following").value:
            # start at lap 1
            self.get_logger().warn("---DEBUG MODE ENABLED---")
            self.crossed_start = True

        self.get_logger().info("---Trackdrive handler node initialised---")

    def state_callback(self, msg: State):
        super().state_callback(msg)
        if (
            msg.state == State.DRIVING
            and msg.mission == State.TRACKDRIVE
            and msg.navigation_ready
            and not self.mission_started
            and self.odom_received
        ):
            self.mission_started = True
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=0))

            if not self.get_parameter("sim").value:
                # reset odom and pose from camera
                self.reset_odom_client.call_async(Trigger.Request())
                self.reset_pose_client.call_async(Trigger.Request())

            command = ["stdbuf", "-o", "L", "ros2", "launch", "mission_controller", "trackdrive.launch.py"]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Trackdrive mission started")

    def path_callback(self, msg: Path):
        # receive path and convert to numpy array
        # if self.path is not None:
        #     return
        self.get_logger().info(f"Spline Path Recieved - length: {len(msg.poses)}", once=True)
        self.send_path(msg)

    def odom_callback(self, msg: Odometry):
        """Ensure the SBG EKF has settled and we get odom msgs before starting the mission"""

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Odometry received")

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

        if self.last_x <= self.goal_offet and track_to_base.transform.translation.x >= self.goal_offet:
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

            # publish initial pose on first lap
            if self.laps == 1 and not self.sent_init:
                init_pose_msg = PoseWithCovarianceStamped()
                init_pose_msg.header.stamp = track_to_base.header.stamp
                init_pose_msg.header.frame_id = "track"
                # convert translation to pose
                init_pose_msg.pose.pose.position.x = track_to_base.transform.translation.x
                init_pose_msg.pose.pose.position.y = track_to_base.transform.translation.y
                init_pose_msg.pose.pose.orientation = track_to_base.transform.rotation
                # cov diag to square
                self.init_pose_pub.publish(init_pose_msg)
                self.sent_init = True

            self.last_x = track_to_base.transform.translation.x
            self.last_lap_time = time.time()

        if self.laps == 10:
            self.get_logger().info("Trackdrive mission complete")
            # currently only works when vehicle supervisor node is running on-car
            # TODO: sort out vehicle states for eventual environment agnostic operation
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)

    def send_path(self, path: Path):
        # Sends a `NavThroughPoses` action request
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()
