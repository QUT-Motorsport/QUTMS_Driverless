import curses

import rclpy
from rclpy.node import Node

from driverless_msgs.srv import SelectMission

from typing import List


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")

        self.client = self.create_client(SelectMission, "select_mission")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.send_srv = SelectMission.Request()
        self.future = None

        self.get_logger().info("---GUI Node Initalised---")


menu: List[str] = ["manual_driving", "inspection", "ebs_test", "trackdrive"]


def print_menu(stdscr, selected_row_idx):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    for idx, row in enumerate(menu):
        x = w // 2 - len(row) // 2
        y = h // 2 - len(menu) // 2 + idx
        if idx == selected_row_idx:
            stdscr.attron(curses.color_pair(1))
            stdscr.addstr(y, x, row)
            stdscr.attroff(curses.color_pair(1))
        else:
            stdscr.addstr(y, x, row)
    stdscr.refresh()


def print_center(stdscr, text):
    stdscr.clear()
    h, w = stdscr.getmaxyx()
    x = w // 2 - len(text) // 2
    y = h // 2
    stdscr.addstr(y, x, text)
    stdscr.refresh()


def gui_main(stdscr, gui_node: GUINode):
    # turn off cursor blinking
    curses.curs_set(0)
    # color scheme for selected row
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
    # specify the current selected row
    current_row = 0
    # print the menu
    print_menu(stdscr, current_row)

    while 1:
        key = stdscr.getch()
        if key == curses.KEY_UP and current_row > 0:
            current_row -= 1
        elif key == curses.KEY_DOWN and current_row < len(menu) - 1:
            current_row += 1
        elif key == curses.KEY_ENTER or key in [10, 13]:
            print_center(stdscr, "You selected '{}'".format(menu[current_row]))
            stdscr.getch()

            gui_node.send_srv.mission = menu[current_row]
            gui_node.future = gui_node.client.call_async(gui_node.send_srv)

            rclpy.spin_until_future_complete(gui_node, gui_node.future)
            response = gui_node.future.result()

            break

        # ADD A CONFIRMATION SCREEN
        # ADD AN END PROGRAM OPTION AFTER SUCCESSFUL START

        print_menu(stdscr, current_row)


def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    curses.wrapper(gui_main, gui_node=node)
    node.destroy_node()
    rclpy.shutdown()
