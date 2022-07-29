import curses

import rclpy
from rclpy.node import Node

from driverless_msgs.srv import SelectMission

from .mission_constants import MissionType


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")

        self.client = self.create_client(SelectMission, "select_mission")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.send_srv = SelectMission.Request()
        self.future = None

        self.get_logger().info("---GUI Node Initalised---")


def print_menu(stdscr, selected_row_idx: int, options: list):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    for idx, row in enumerate(options):
        x: int = w // 2 - len(row) // 2
        y: int = h // 2 - len(MissionType) // 2 + idx
        if idx == selected_row_idx:
            stdscr.attron(curses.color_pair(1))
            stdscr.addstr(y, x, row)
            stdscr.attroff(curses.color_pair(1))
        else:
            stdscr.addstr(y, x, row)
    stdscr.refresh()


def print_center(stdscr, text: str):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    x: int = w // 2 - len(text) // 2
    y: int = h // 2
    stdscr.addstr(y, x, text)
    stdscr.refresh()


def gui_main(stdscr, gui_node: GUINode):
    # turn off cursor blinking
    curses.curs_set(0)
    # color scheme for selected row
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
    # specify the current selected row
    current_row: int = 0

    options = [element.value for element in MissionType]

    # print the menu
    print_menu(stdscr, current_row, options)

    while 1:
        key = stdscr.getch()
        if key == curses.KEY_UP and current_row > 0:
            current_row -= 1
        elif key == curses.KEY_DOWN and current_row < len(MissionType) - 1:
            current_row += 1
        elif key == curses.KEY_ENTER or key in [10, 13]:
            print_center(stdscr, "You selected '{}'".format(options[current_row]))
            stdscr.getch()

            gui_node.send_srv.mission = options[current_row]
            gui_node.future = gui_node.client.call_async(gui_node.send_srv)

            rclpy.spin_until_future_complete(gui_node, gui_node.future)
            response = gui_node.future.result()

            break

        print_menu(stdscr, current_row, options)


def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    curses.wrapper(gui_main, gui_node=node)
    node.destroy_node()
    rclpy.shutdown()
