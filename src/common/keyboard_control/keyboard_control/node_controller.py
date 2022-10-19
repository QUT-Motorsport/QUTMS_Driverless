import curses

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive

TORQUE_MIN = 0.0
TORQUE_MAX = 1.0
TORQUE_INCREMENT = 0.1

STEER_MIN = -3.1
STEER_MAX = 3.1
STEER_INCREMENT = 0.1

torque: float = 0.0
steering_angle: float = 0.0


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("keyboard_controller")

        self.drive_command_publisher: Publisher = self.create_publisher(AckermannDrive, "drive_command", 1)

    def publish_drive_command(self, torque: float, steering_angle: float):
        self.drive_command_publisher.publish(AckermannDrive(acceleration=torque, steering_angle=steering_angle))


def print_state(stdscr):
    global torque
    global steering_angle

    stdscr.addstr(0, 0, f"Torque: {torque}           ")
    stdscr.addstr(1, 0, f"Steering Angle: {steering_angle}     ")
    stdscr.addstr(3, 0, f"[Space] to zero torque")
    stdscr.addstr(4, 0, f"[Backspace] to zero steering")
    stdscr.addstr(5, 0, f"[Enter] to zero everything")


def curses_main(stdscr, keyboard_controller_node: KeyboardControllerNode):
    global torque
    global steering_angle

    stdscr.clear()
    print_state(stdscr)

    while True:
        c = stdscr.getch()
        if c == ord("w"):
            torque += TORQUE_INCREMENT
        if c == ord("s"):
            torque -= TORQUE_INCREMENT
        if c == ord("a"):
            steering_angle += STEER_INCREMENT
        if c == ord("d"):
            steering_angle -= STEER_INCREMENT
        if c == ord(" "):
            torque = 0.0
        if c == curses.KEY_BACKSPACE:
            steering_angle = 0.0
        if c == ord("\n"):
            torque = 0.0
            steering_angle = 0.0

        torque = round(max(min(torque, TORQUE_MAX), TORQUE_MIN), 1)
        steering_angle = round(max(min(steering_angle, STEER_MAX), STEER_MIN), 1)

        print_state(stdscr)
        keyboard_controller_node.publish_drive_command(torque, steering_angle)


def main():
    rclpy.init()
    controller_node = KeyboardControllerNode()
    curses.wrapper(curses_main, controller_node)


if __name__ == "__main__":
    main()
