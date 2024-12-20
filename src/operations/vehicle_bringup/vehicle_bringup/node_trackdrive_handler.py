from subprocess import Popen
import time

from nav2_msgs.action import FollowPath
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped, ROSStateStamped, Shutdown
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import UInt8

from std_srvs.srv import SetBool

from vehicle_bringup.shutdown_node_class import ShutdownNode


class TrackdriveHandler(ShutdownNode):
    mission_started = False
    good_to_go = False
    process = None
    debug = False
    record: bool = False

    sent_init = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0
    path = None
    in_box = True

    controller_id = "TrackdriveRPP"

    def __init__(self):
        super().__init__("trackdrive_logic_node")

        # subscribers
        self.create_subscription(AVStateStamped, "system/av_state", self.av_state_callback, 1)
        self.create_subscription(ROSStateStamped, "system/ros_state", self.ros_state_callback, 1)
        self.create_subscription(Path, "planning/midline_path", self.path_callback, 1)

        self.create_timer((1 / 20), self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create recording service
        self.create_service(SetBool, "system/record", self.record_callback)

        # publishers
        self.shutdown_pub = self.create_publisher(Shutdown, "system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "system/laps_completed", 1)

        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        # actions
        self.nav_through_poses_client = ActionClient(self, FollowPath, "follow_path")

        self.declare_parameter("debug", False)

        if self.get_parameter("debug").value:
            self.get_logger().warn("---DEBUG MODE ENABLED---")

            self.mission_started = True
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=0))

            command = [
                "stdbuf",
                "-o",
                "L",
                "ros2",
                "launch",
                "vehicle_bringup",
                "trackdrive.launch.py",
                f"record:={self.record}",
            ]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Trackdrive mission started")

        self.get_logger().info("---Trackdrive handler node initialised---")

    def record_callback(self, request, response):
        self.record = request.data
        response.success = True
        self.get_logger().info(f"Recording set to {self.record}")
        return response

    def av_state_callback(self, msg: AVStateStamped):
        super().av_state_callback(msg)
        if (
            (msg.state == AVStateStamped.START_MISSION or msg.state == AVStateStamped.DRIVING)
            and not self.mission_started
            and self.good_to_go
        ):
            self.mission_started = True
            self.last_lap_time = time.time()
            self.lap_trig_pub.publish(UInt8(data=0))

            command = [
                "stdbuf",
                "-o",
                "L",
                "ros2",
                "launch",
                "vehicle_bringup",
                "trackdrive.launch.py",
                f"record:={self.record}",
            ]
            self.get_logger().info(f"Command: {' '.join(command)}")
            self.process = Popen(command)
            self.get_logger().info("Trackdrive mission started")

    def ros_state_callback(self, msg: ROSStateStamped):
        if msg.good_to_go:
            self.good_to_go = True

    def path_callback(self, msg: Path):
        # receive path and convert to numpy array
        # if self.path is not None:
        #     return
        self.get_logger().info(f"Spline Path Recieved - length: {len(msg.poses)}", once=True)

        # Sends a `NavThroughPoses` action request
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        # send controller ID in request
        goal_msg.controller_id = (
            self.controller_id
        )  # nav2_params.yaml, controller_server, controller_plugins: ["TrackdriveRPP", "EBSTestRPP"]

        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)

    def timer_callback(self):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        if not self.mission_started:
            return

        try:
            track_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time(seconds=0))
        except TransformException as e:
            self.get_logger().debug("Transform exception: " + str(e))
            return

        # publish initial pose
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

        # we start at 0,0
        # once we cross out of x < 2, we can start counting laps
        # if we cross back into x == -2, begin checking for lap completion
        # lap completion is when we cross x == 2 again

        # check if we are within the bounds of the start line width (approx 2m)
        # and we are also within the distance of the finish line
        if abs(track_to_base.transform.translation.x) < 2 and abs(track_to_base.transform.translation.y) < 2:
            self.in_box = True
            self.get_logger().info(f"In the starting box", throttle_duration_sec=1)

        # if we are in box, we need to leave the box before we can start counting laps
        if self.in_box:
            if track_to_base.transform.translation.x < 2:
                self.last_x = track_to_base.transform.translation.x
                return

            # we have left the box
            self.get_logger().info(f"Crossed start line in {time.time() - self.last_lap_time:.2f}s")

            self.in_box = False
            self.last_x = track_to_base.transform.translation.x
            self.last_lap_time = time.time()

            self.laps += 1
            self.lap_trig_pub.publish(UInt8(data=self.laps - 1))
            self.get_logger().info(f"Lap {self.laps} completed")

        # we have finished lap "1"
        if self.laps > 1:
            self.controller_id = "TrackdriveRPPFast"

        # we have finished lap "10"
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
