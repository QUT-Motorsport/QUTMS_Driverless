import os
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running, check_if_js_published, check_node_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
import pytest

import rclpy


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("qev3d_ros2_control"),  # Replace with your package name
                "launch/qev3d.launch.py",  # Replace with your launch file
            )
        ),
        launch_arguments={"gui": "false"}.items(),  # Modify launch arguments if needed
    )

    return LaunchDescription([launch_include, ReadyToTest()])


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_node_start(self, proc_output):
        """Test if the main node is running."""
        nodes = ["robot_state_publisher"]  # Replace with your nodes
        check_node_running(self.node, nodes)

    def test_controller_running(self, proc_output):
        """Test if controllers are running."""
        controllers = [
            "bicycle_steering_controller",
            "drive_pid_controller",
            "steering_pid_controller",
            "joint_state_broadcaster",
        ]  # Replace with your controllers
        check_controllers_running(self.node, controllers)

    def test_check_if_msgs_published(self):
        """Test if expected messages are published."""
        check_if_js_published(
            "/joint_states", ["virtual_front_wheel_joint", "virtual_rear_wheel_joint"]
        )  # Replace with your topic and expected data
