import curses

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Reset, State
from std_msgs.msg import UInt8

SPEED_MIN = 0.0
SPEED_MAX = 4.0
SPEED_INCREMENT = 0.1

STEER_MIN = -90.0
STEER_MAX = 90.0
STEER_INCREMENT = -5

speed: float = 0.0
steering_angle: float = 0.0
state: str = "idle"
mission: int = 0


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("keyboard_controller_node")

        self.state_sub: Publisher = self.create_subscription(State, "/system/state", self.callback, 1)
        self.drive_command_publisher: Publisher = self.create_publisher(
            AckermannDriveStamped, "/control/driving_command", 1
        )
        self.mission_pub: Publisher = self.create_publisher(UInt8, "/system/mission_select", 1)
        self.reset_pub: Publisher = self.create_publisher(Reset, "/system/reset", 1)

    def publish_drive_command(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.drive_command_publisher.publish(msg)

    def reset(self):
        self.reset_pub.publish(Reset(reset=True))

    def callback(self, msg: State):
        global state, mission

        state = msg.state
        self.get_logger().info(f"State: {msg.state}")


def print_state(stdscr):
    global speed, steering_angle, state, mission

    stdscr.addstr(0, 0, f"State: {state}. Mission: {mission}")
    stdscr.addstr(2, 0, "Use the WASD keys to control the car:")
    stdscr.addstr(3, 0, f"Torque: {speed}")
    stdscr.addstr(4, 0, f"Steering Angle: {steering_angle}")
    stdscr.addstr(6, 0, f"[Space] to zero torque")
    stdscr.addstr(7, 0, f"[g] to zero steering")
    stdscr.addstr(8, 0, f"[Enter] to zero everything")
    stdscr.addstr(10, 0, f"Select mission: [0] to [4]")
    stdscr.addstr(
        11,
        0,
        f"Mission 0: None\t Mission 1: Manual Driving\t Mission 2: Inspection\t Mission 3: EBS Test\t Mission 4: Trackdrive",
    )
    stdscr.addstr(13, 0, f"")


def curses_main(stdscr, keyboard_controller_node: KeyboardControllerNode):
    global speed, steering_angle, state, mission

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
        if c == ord("g"):
            steering_angle = 0.0
        if c == ord("0"):
            mission = 0
        if c == ord("1"):
            mission = 1
        if c == ord("2"):
            mission = 2
        if c == ord("3"):
            mission = 3
        if c == ord("4"):
            mission = 4
        if c == ord("\n"):
            speed = 0.0
            steering_angle = 0.0
            keyboard_controller_node.reset()

        speed = round(max(min(speed, SPEED_MAX), SPEED_MIN), 2)
        steering_angle = round(max(min(steering_angle, STEER_MAX), STEER_MIN), 2)

        if speed > 0.0:
            state = "driving"
        elif speed == 0.0:
            state = "idle"

        print_state(stdscr)
        keyboard_controller_node.publish_drive_command(speed, steering_angle)
        keyboard_controller_node.mission_pub.publish(UInt8(data=mission))


def main():
    rclpy.init()
    controller_node = KeyboardControllerNode()
    curses.wrapper(curses_main, controller_node)


if __name__ == "__main__":
    main()
