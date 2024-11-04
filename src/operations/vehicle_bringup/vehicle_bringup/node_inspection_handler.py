import os
from subprocess import Popen
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import AVStateStamped, ROSStateStamped, Shutdown

from vehicle_bringup.ros2_launch_parent_class import Ros2LaunchParent
from vehicle_bringup.shutdown_node_class import ShutdownNode


class InspectionHandler(ShutdownNode):
    mission_started = False
    good_to_go = False
    process = None

    start_time = 0.0

    def __init__(self):
        super().__init__("inspection_logic_node")

        # subscribers
        self.create_subscription(AVStateStamped, "system/av_state", self.av_state_callback, 1)
        self.create_subscription(ROSStateStamped, "system/ros_state", self.ros_state_callback, 1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # publishers
        self.shutdown_pub: Publisher = self.create_publisher(Shutdown, "/system/shutdown", 1)

        self.launch_parent = Ros2LaunchParent()

        self.get_logger().info("---Inspection handler node initialised---")

    def av_state_callback(self, msg: AVStateStamped):
        super().av_state_callback(msg)
        if msg.state == AVStateStamped.DRIVING and not self.mission_started and self.good_to_go:
            self.mission_started = True
            self.start_time = time.time()
            # command = ["stdbuf", "-o", "L", "ros2", "launch", "vehicle_bringup", "inspection.launch.py"]
            # self.get_logger().info(f"Command: {' '.join(command)}")
            # self.process = Popen(command)
            launch_description = LaunchDescription(
                [
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("vehicle_bringup"), "mission_launch", "inspection.launch.py"
                            )
                        )
                    )
                ]
            )
            self.launch_parent.start(launch_description)
            self.get_logger().info("Inspection mission started")

    def ros_state_callback(self, msg: ROSStateStamped):
        if msg.good_to_go:
            self.good_to_go = True

    def timer_callback(self):
        if self.mission_started and time.time() - self.start_time > 30:
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)
            self.get_logger().info("Inspection mission finished")
            self.get_logger().warn("Closing Inspection mission")
            self.launch_parent.shutdown()
            # self.process.terminate()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
