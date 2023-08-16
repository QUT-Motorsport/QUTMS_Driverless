import curses
from curses.textpad import Textbox, rectangle

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

class KeyboardControllerNode(Node):
    desired_velocity: float = 0.0
    desired_steering: float = 0.0
    state: int = State.START
    mission: int = State.MISSION_NONE
    laps: int = 0

    def __init__(self):
        super().__init__("terminal_controller_node")

        self.state_pub = self.create_publisher(UInt8, "/system/as_status", 1)

        self.drive_command_publisher: Publisher = self.create_publisher(
            AckermannDriveStamped, "/control/driving_command", 1
        )

        # timer for publishing state
        self.create_timer(0.02, self.state_timer_callback)


    def publish_drive_command(self):
        if self.state == State.DRIVING and self.mission != State.MISSION_NONE:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = self.desired_velocity
            msg.drive.steering_angle = self.desired_steering
            self.drive_command_publisher.publish(msg)

    def state_timer_callback(self):
        as_state = State()
        as_state.state = self.state
        as_state.mission = self.mission
        as_state.lap_count = self.laps

def reset_controller(stdscr, node: KeyboardControllerNode):
    # clear screen
    stdscr.clear()

    node.desired_velocity = 0.0
    node.desired_steering = 0.0
    node.state = State.START
    node.mission = State.MISSION_NONE
    node.laps = 0

def default_info(stdscr, node: KeyboardControllerNode):
    # clear screen
    stdscr.clear()

    mission_str = INT_MISSION_TYPE[node.mission].value
    state_str = INT_STATE_TYPE[node.state].value

    # display mission prompt and mission enum options
    stdscr.addstr(0, 0, f"State: {state_str} | Selected Mission: {mission_str} | Laps Completed: {node.laps}")
    stdscr.addstr(1, 0, f"Requested Velocity: {node.desired_velocity} | Requested Steering Angle: {node.desired_steering}")
    stdscr.addstr(2, 0, f"")
    stdscr.addstr(3, 0, f"Press 'x' to return to main menu")
    stdscr.addstr(4, 0, f"Press 'r' to reset")
    stdscr.addstr(5, 0, f"")

    return 5

def main_menu(stdscr, node: KeyboardControllerNode):
    # clear screen
    stdscr.clear()

    mission_str = INT_MISSION_TYPE[node.mission].value
    state_str = INT_STATE_TYPE[node.state].value

    stdscr.addstr(0, 0, f"State: {state_str} | Selected Mission: {mission_str} | Laps Completed: {node.laps}")
    stdscr.addstr(1, 0, f"Requested Velocity: {node.desired_velocity} | Requested Steering Angle: {node.desired_steering}")
    stdscr.addstr(3, 0, f"Press 'c' to use the [wasd] keys to control the car")
    stdscr.addstr(4, 0, f"Press 'l' to type in laps completed")
    stdscr.addstr(5, 0, f"Press 'm' to type in mission enum")
    stdscr.addstr(6, 0, f"Press 'e' to type in state enum")
    stdscr.addstr(7, 0, f"Press 'x' to return to this main menu")
    stdscr.addstr(8, 0, f"Press 'r' to reset state in any menu")
    stdscr.addstr(9, 0, f"")

    main_menu_input = True
    while main_menu_input:
        c = stdscr.getch()
        if c == ord("c"):
            main_menu_input = False
            control_menu(stdscr, node)
        if c == ord("l"):
            main_menu_input = False
            laps_menu(stdscr, node)
        if c == ord("m"):
            main_menu_input = False
            mission_menu(stdscr, node)
        if c == ord("e"):
            main_menu_input = False
            state_menu(stdscr, node)
        if c == ord("r"):
            main_menu_input = False
            reset_controller(stdscr, node)    

def mission_menu(stdscr, node: KeyboardControllerNode):
    default_info(stdscr, node)

    stdscr.addstr(6, 0, f"Select mission number: ")
    for i, mission in enumerate(INT_MISSION_TYPE):
        stdscr.addstr(7 + i, 0, f"{i}: {INT_MISSION_TYPE[mission].value}")
    offset = 7 + len(INT_MISSION_TYPE)
    stdscr.addstr(offset, 0, f"")

    # get user input
    mission_menu_input = True
    confirm_input = False
    while mission_menu_input:
        c = stdscr.getch()
        if c == ord("x"):
            mission_menu_input = False
        if c == ord("r"):
            reset_controller(stdscr, node)
            mission_menu_input = False
        if c in range(48, 48 + len(INT_MISSION_TYPE)):
            sel_mission = c - 48
            # display selected mission
            # clear line
            stdscr.addstr(offset + 1, 0, f"{' ' * 100}")
            stdscr.addstr(offset + 1, 0, f"Selected Mission: {INT_MISSION_TYPE[sel_mission].value}")
            stdscr.addstr(offset + 2, 0, f"Press ENTER to confirm or type another number")
            # get user input
            confirm_input = True
        if c == ord("\n") and confirm_input:
            node.mission = sel_mission
            mission_menu_input = False

def control_menu(stdscr, node: KeyboardControllerNode):
    offset = default_info(stdscr, node)

    # ascii art for wasd keys
    stdscr.addstr(offset + 1, 0, f"    [w]    ")
    stdscr.addstr(offset + 2, 0, f"[a] [s] [d]")
    stdscr.addstr(offset + 3, 0, f"")

    # get user input
    control_menu_input = True
    while control_menu_input:
        c = stdscr.getch()
        if c == ord("x"):
            control_menu_input = False
        if c == ord("r"):
            reset_controller(stdscr, node)
            control_menu_input = False

def laps_menu(stdscr, node: KeyboardControllerNode):
    offset = default_info(stdscr, node)

    stdscr.addstr(offset + 1, 0, f"Enter laps completed: ")
    stdscr.addstr(offset + 2, 0, f"")

    # get user input
    laps_menu_input = True
    confirm_input = False
    while laps_menu_input:
        c = stdscr.getch()
        if c == ord("x"):
            laps_menu_input = False
        if c == ord("r"):
            reset_controller(stdscr, node)
            laps_menu_input = False
        if c in range(48, 58):
            stdscr.addstr(offset + 2, 0, f"{chr(c)}")
            sel_laps = int(chr(c))
            stdscr.addstr(offset + 3, 0, f"Press ENTER to confirm or type another number")
            confirm_input = True
        if c == ord("\n") and confirm_input:
            laps_menu_input = False
            node.laps = sel_laps

def state_menu(stdscr, node: KeyboardControllerNode):
    default_info(stdscr, node)

def curses_main(stdscr, node: KeyboardControllerNode):

    while True:
        main_menu(stdscr, node)

def main():
    rclpy.init()
    controller_node = KeyboardControllerNode()
    curses.wrapper(curses_main, controller_node)


if __name__ == "__main__":
    main()
