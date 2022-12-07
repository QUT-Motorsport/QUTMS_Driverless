import curses

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import Reset

SPEED_MIN = 0.0
SPEED_MAX = 4.0
SPEED_INCREMENT = 0.1

STEER_MIN = -90.0
STEER_MAX = 90.0
STEER_INCREMENT = -5

speed: float = 0.0
steering_angle: float = 0.0


class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__("keyboard_controller")

        self.drive_command_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "driving_command", 1)
        self.reset_pub: Publisher = self.create_publisher(Reset, "/reset", 1)

    def publish_drive_command(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.drive_command_publisher.publish(msg)

    def reset(self):
        self.reset_pub.publish(Reset(reset=True))


def print_state(stdscr):
    global speed
    global steering_angle

    stdscr.addstr(0, 0, f"Torque: {speed}           ")
    stdscr.addstr(1, 0, f"Steering Angle: {steering_angle}     ")
    stdscr.addstr(3, 0, f"[Space] to zero torque")
    stdscr.addstr(4, 0, f"[g] to zero steering")
    stdscr.addstr(5, 0, f"[Enter] to zero everything")


def curses_main(stdscr, keyboard_controller_node: KeyboardControllerNode):
    global speed
    global steering_angle

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
        if c == ord("\n"):
            speed = 0.0
            steering_angle = 0.0
            keyboard_controller_node.reset()

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
