from subprocess import Popen
import time

from nav2_msgs.action import FollowPath
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from driverless_msgs.msg import Shutdown, State
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8

from driverless_common.shutdown_node import ShutdownNode


class EBSTestHandler(ShutdownNode):
    mission_started = False
    odom_received = False
    sent_init = False
    goal_offet = 0.1
    path = None
    goal_handle = None

    def __init__(self):
        super().__init__("ebs_test_logic_node")

        self.create_subscription(State, "system/as_status", self.state_callback, 1)
        self.create_subscription(Path, "planning/midline_path", self.path_callback, 1)
        self.create_subscription(Odometry, "imu/odometry", self.odom_callback, 1)

        self.create_timer((1 / 20), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.shutdown_pub = self.create_publisher(Shutdown, "system/shutdown", 1)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        # actions
        self.nav_through_poses_client = ActionClient(self, FollowPath, "follow_path")

        self.declare_parameter("debug", True)

        if self.get_parameter("debug").value:
            self.debug = True
            self.get_logger().warn("---DEBUG MODE ENABLED---")

            command = ["stdbuf", "-o", "L", "ros2", "launch", "mission_controller", "trackdrive.launch.py"]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Trackdrive mission started")

        self.get_logger().info("---Trackdrive handler node initialised---")

    def state_callback(self, msg: State):
        if self.debug:
            return
        
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

            command = ["stdbuf", "-o", "L", "ros2", "launch", "mission_controller", "ebs_test.launch.py"]
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
            # publish initial pose on first lap
            if not self.sent_init:
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

    def send_path(self, path: Path):
        # Sends a `NavThroughPoses` action request
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        # send controller ID in request
        goal_msg.controller_id = "EBSTestRPP" # nav2_params.yaml, controller_server, controller_plugins: ["TrackdriveRPP", "EBSTestRPP"]

        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EBSTestHandler()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()
