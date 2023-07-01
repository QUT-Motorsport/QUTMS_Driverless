import curses

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Reset, State
from std_msgs.msg import Bool, UInt8

from driverless_common.common import QOS_LATEST
from driverless_common.status_constants import INT_MISSION_TYPE, INT_STATE_TYPE

SPEED_MIN = 0.0
SPEED_MAX = 4.0
SPEED_INCREMENT = 0.1

STEER_MIN = -85.0
STEER_MAX = 85.0
STEER_INCREMENT = -5

speed: float = 0.0
steering_angle: float = 0.0
state: int = State.START
mission: int = State.MISSION_NONE
actual_mission: int = 0
r2d: bool = False
laps: int = 0


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("terminal_controller_node")

        self.state_subr = self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)

        self.drive_command_publisher: Publisher = self.create_publisher(
            AckermannDriveStamped, "/control/driving_command", 1
        )
        self.mission_pub: Publisher = self.create_publisher(UInt8, "/system/mission_select", 1)
        self.reset_pub: Publisher = self.create_publisher(Reset, "/system/reset", 1)
        self.r2d_pub: Publisher = self.create_publisher(Bool, "/system/r2d", 1)
        self.lap_pub: Publisher = self.create_publisher(UInt8, "/system/laps_completed", 1)

    def publish_drive_command(self, speed: float, steering_angle: float):
        if r2d:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = speed
            msg.drive.steering_angle = steering_angle
            self.drive_command_publisher.publish(msg)

    def state_callback(self, msg: State):
        global state, mission

        state = msg.state


def print_state(stdscr):
    global speed, steering_angle, state, mission

    mission_str: str = INT_MISSION_TYPE[mission].value
    actual_mission_str: str = INT_MISSION_TYPE[actual_mission].value
    state_str: str = INT_STATE_TYPE[state].value

    stdscr.addstr(0, 0, f"State: {state_str} | Selected Mission: {mission_str} | Laps Completed: {laps}")
    stdscr.addstr(1, 0, f"Requested Velocity: {speed} | Requested Steering Angle: {steering_angle}")
    stdscr.addstr(3, 0, f"Select mission to launch ROS nodes: [0] to [4]")
    stdscr.addstr(
        4,
        0,
        f"\t0: None | 1: Manual Driving | 2: Inspection | 3: EBS Test | 4: Trackdrive",
    )
    stdscr.addstr(5, 0, f"Press [m] to confirm mission selection")
    stdscr.addstr(7, 0, "Use the [wasd] keys to control the car")
    stdscr.addstr(8, 0, f"[Space] to zero everything")
    stdscr.addstr(9, 0, f"[r] to reset state")
    stdscr.addstr(10, 0, f"[l] to increment a lap")
    stdscr.addstr(11, 0, f"[Enter] to go R2D")
    stdscr.addstr(12, 0, f"")
    # THIS WILL ERROR IF TERMINAL BY DEFAULT IS TOO SMALL (NOT ENOUGH LINES)
    # INCREASE TERMINAL SIZE TO FIX


def curses_main(stdscr, keyboard_controller_node: KeyboardControllerNode):
    global speed, steering_angle, state, mission, r2d, laps

    stdscr.clear()
    print_state(stdscr)

    while True:
        c = stdscr.getch()
        if c == ord("w"):
            speed += SPEED_INCREMENT
        if c == ord("s"):
            speed -= SPEED_INCREMENT
        if c == ord("a"):
            steering_angle -= STEER_INCREMENT
        if c == ord("d"):
            steering_angle += STEER_INCREMENT
        if c == ord(" "):
            speed = 0.0
            steering_angle = 0.0
        if c == ord("0"):
            mission = State.MISSION_NONE
        if c == ord("1"):
            mission = State.MANUAL_DRIVING
        if c == ord("2"):
            mission = State.INSPECTION
        if c == ord("3"):
            mission = State.EBS_TEST
        if c == ord("4"):
            mission = State.TRACKDRIVE
        if c == ord("m"):
            keyboard_controller_node.mission_pub.publish(UInt8(data=mission))
        if c == ord("\n"):
            r2d = True
            state = State.READY
            keyboard_controller_node.r2d_pub.publish(Bool(data=r2d))
        if c == ord("r"):
            speed = 0.0
            steering_angle = 0.0
            mission = State.MISSION_NONE
            state = State.START
            r2d = False
            laps = 0
            keyboard_controller_node.reset_pub.publish(Reset(reset=True))
            keyboard_controller_node.r2d_pub.publish(Bool(data=r2d))
            keyboard_controller_node.lap_pub.publish(UInt8(data=laps))
        if c == ord("l"):
            laps += 1
            keyboard_controller_node.lap_pub.publish(UInt8(data=laps))

        speed = round(max(min(speed, SPEED_MAX), SPEED_MIN), 2)
        steering_angle = round(max(min(steering_angle, STEER_MAX), STEER_MIN), 2)

        print_state(stdscr)
        keyboard_controller_node.publish_drive_command(speed, steering_angle)


def main():
    rclpy.init()
    controller_node = KeyboardControllerNode()
    curses.wrapper(curses_main, controller_node)


if __name__ == "__main__":
    main()
