import os
import time
import unittest

from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions

import rclpy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


def generate_test_description():
    launch_file_path = os.path.join(
        get_package_share_path("ros2_control_bringup"),
        "launch",
        "test",
        "test_mock.launch.py",
    )
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path)),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMockIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_mock_integration")

    def tearDown(self):
        self.node.destroy_node()

    def test_state_broadcasting_and_inverse_kinematics(self):
        pub = self.node.create_publisher(
            TwistStamped,
            "/ackermann_steering_direct_controller/reference",
            10,
        )

        received_states = []

        def cb(msg):
            received_states.append(msg)

        sub = self.node.create_subscription(JointState, "/joint_states", cb, 10)

        # Command reference values
        # vx = 2.0 m/s
        # wz = 0.5 rad/s
        # Expected outputs:
        # Front steering angle: atan(wz * wheelbase / vx) = atan(0.5 * 1.535 / 2.0) = 0.3665 rad
        # Rear left wheel speed: (vx - wz * wheel_track / 2) / wheel_radius = (2.0 - 0.5 * 1.400 / 2) / 0.2032 = 8.12 rad/s
        # Rear right wheel speed: (vx + wz * wheel_track / 2) / wheel_radius = (2.0 + 0.5 * 1.400 / 2) / 0.2032 = 11.56 rad/s

        msg = TwistStamped()
        msg.twist.linear.x = 2.0
        msg.twist.angular.z = 0.5

        start_time = time.time()
        success = False

        # Periodically publish twist command and spin to check the joint states
        while time.time() - start_time < 15.0:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

            for state in received_states:
                # Zip joint names and values into a dictionary
                joint_map = dict(zip(state.name, zip(state.position, state.velocity)))

                if (
                    "virtual_front_wheel_joint" in joint_map
                    and "rear_left_wheel_joint" in joint_map
                    and "rear_right_wheel_joint" in joint_map
                ):
                    front_pos = joint_map["virtual_front_wheel_joint"][0]
                    left_vel = joint_map["rear_left_wheel_joint"][1]
                    right_vel = joint_map["rear_right_wheel_joint"][1]

                    # Assert values match our expected kinematic updates (with a threshold margin)
                    if abs(front_pos - 0.3665) < 0.05 and abs(left_vel - 11.56) < 0.1 and abs(right_vel - 8.12) < 0.1:
                        success = True
                        break

            if success:
                break
            time.sleep(0.1)

        self.assertTrue(
            success,
            "Failed to receive joint state mirroring the expected inverse kinematics values",
        )
