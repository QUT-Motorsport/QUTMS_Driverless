import curses
import threading

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import State
from std_msgs.msg import UInt8

from driverless_common.common import QOS_LATEST
from driverless_common.status_constants import INT_MISSION_TYPE, INT_STATE_TYPE


class KeyboardControllerNode(Node):
    state = State.START
    mission = State.MISSION_NONE
    laps = 0

    def __init__(self):
        super().__init__("terminal_controller_node")

        self.create_subscription(UInt8, "on_request/lap", self.lap_callback, QOS_LATEST)
        self.create_subscription(UInt8, "on_request/mission", self.mission_callback, QOS_LATEST)
        self.create_subscription(UInt8, "on_request/state", self.state_callback, QOS_LATEST)

        self.state_pub = self.create_publisher(State, "system/as_status", 1)

        # timer for publishing state
        self.create_timer(0.02, self.state_timer_callback)

    def state_timer_callback(self):
        as_state = State()
        as_state.state = self.state
        as_state.mission = self.mission
        as_state.lap_count = self.laps
        self.state_pub.publish(as_state)

    def lap_callback(self, msg: UInt8):
        self.laps = msg.data

    def mission_callback(self, msg: UInt8):
        self.mission = msg.data

    def state_callback(self, msg: UInt8):
        self.state = msg.data


def main():
    rclpy.init()
    node = KeyboardControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
