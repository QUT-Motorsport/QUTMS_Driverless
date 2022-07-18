# attempted realistic driving controls with WASD
import curses
import time

import rclpy
from rclpy.node import Node

from fs_msgs.msg import ControlCommand

from .draw_gui import Gui


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")
        # self.get_logger().info("GUI Node Initalised")
        self.keypress_publisher = self.create_publisher(
            ControlCommand, "/control_command", 10
        )  # sending steering values to sim, control command are the messages being sent to the topic

    def publish_command(self, control_message: ControlCommand):
        self.keypress_publisher.publish(control_message)


def gui_main(stdscr, gui_node: GUINode):
    steering_min = -50
    steering_max = 50
    steering_count = 0
    throttle = 0
    brake = 0

    curses.noecho()
    # curses.cbreak()
    curses.curs_set(0)
    stdscr.nodelay(True)  # no need to wait for input

    while True:
        active_steering = 0
        active_throttle = 0
        active_brake = 0
        c = stdscr.getch()
        curses.flushinp()
        stdscr.clear()

        # input for control_command
        if c == ord("a"):
            steering_count -= 10  # original value -5
            active_steering = steering_count
            if steering_count < steering_min:
                steering_count = steering_min  # max steering angle is 25 degrees
                active_steering = steering_count

        if c == ord("d"):
            steering_count += 10  # original value 5
            active_steering = steering_count
            if steering_count > steering_max:
                steering_count = steering_max
                active_steering = steering_count

        if c == ord("w"):
            throttle = throttle + 20  # orginal value 10
            active_throttle = throttle
            if throttle > 100:
                throttle = 100
            active_throttle = throttle

        if c == ord("s"):
            brake = brake + 25  # orginal value 15
            active_brake = brake
            if brake > 100:
                brake = 100
            active_brake = brake

        # decrement steering_count to 0 when no key is pressed
        if c == -1 and steering_count > 0:
            steering_count -= 5  # original 2
            active_steering = steering_count
            if steering_count < 0:
                steering_count = 0
                active_steering = steering_count

        if c == -1 and steering_count < 0:
            steering_count += 5  # original 2
            active_steering = steering_count
            if steering_count > 0:
                steering_count = 0
                active_steering = steering_count

        # reduce throttle and brake to 0 when no key is pressed
        if c == -1 and throttle > 0:
            throttle = 0

        if c == -1 and brake > 0:
            brake = 0

        if c == ord("k"):
            break

        Gui(stdscr).draw_gui(active_steering, active_throttle, active_brake)

        control_msg = ControlCommand()
        control_msg.throttle = float(active_throttle / 100)
        control_msg.steering = float(active_steering / 100)
        control_msg.brake = float(active_brake / 100)
        gui_node.publish_command(control_msg)

        time.sleep(0.05)  # used to calculate speed to max steering, throttle, brake


def main():
    rclpy.init()
    node = GUINode()
    curses.wrapper(gui_main, gui_node=node)  # stdscr = curses.initscr() intialised with curses.wrapper in gui_main
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
