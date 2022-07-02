import curses

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from .curses_slider import CursesSlider
from .threshold import Threshold


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")

        self.threshold_publisher: Publisher = self.create_publisher(String, "hsv_thresholder/threshold", 1)

        self.get_logger().info("GUI Node Initalised")

    def publish_threshold(self, threshold: Threshold):
        self.threshold_publisher.publish(String(data=threshold.to_json()))


def gui_main(stdscr, gui_node: GUINode):
    lower_H_slider = CursesSlider("H", 0, 255, row=1, col=3, scalar=5, initial_val=0)
    lower_S_slider = CursesSlider("S", 0, 255, row=3, col=3, scalar=5, initial_val=0)
    lower_V_slider = CursesSlider("V", 0, 255, row=5, col=3, scalar=5, initial_val=0)

    upper_H_slider = CursesSlider("H", 0, 255, row=8, col=3, scalar=5, initial_val=255)
    upper_S_slider = CursesSlider("S", 0, 255, row=10, col=3, scalar=5, initial_val=255)
    upper_V_slider = CursesSlider("V", 0, 255, row=12, col=3, scalar=5, initial_val=255)

    sliders = [
        lower_H_slider,
        lower_S_slider,
        lower_V_slider,
        upper_H_slider,
        upper_S_slider,
        upper_V_slider,
    ]
    active_idx = 0

    curses.init_pair(1, curses.COLOR_BLUE, curses.COLOR_BLACK)

    while True:
        stdscr.clear()

        # headings
        stdscr.addstr(0, 0, "Lower", curses.A_BOLD)
        stdscr.addstr(7, 0, "Upper", curses.A_BOLD)

        stdscr.addstr(sliders[active_idx].row, 1, ">", curses.color_pair(1))
        for i, slider in enumerate(sliders):
            slider.draw(stdscr, colour_pair=1 if i == active_idx else 0)

        key = stdscr.getch()

        # slider selection
        if key == curses.KEY_DOWN:
            active_idx += 1
            if active_idx > (len(sliders) - 1):
                active_idx = len(sliders) - 1
        if key == curses.KEY_UP:
            active_idx -= 1
            if active_idx < 0:
                active_idx = 0

        # slider value
        if key == curses.KEY_RIGHT:
            sliders[active_idx] += 1
        if key == curses.KEY_LEFT:
            sliders[active_idx] -= 1
        if key == curses.KEY_PPAGE:
            sliders[active_idx] += 10
        if key == curses.KEY_NPAGE:
            sliders[active_idx] -= 10

        # update publish new threshold value
        gui_node.publish_threshold(
            Threshold(
                lower=[
                    lower_H_slider.val,
                    lower_S_slider.val,
                    lower_V_slider.val,
                ],
                upper=[
                    upper_H_slider.val,
                    upper_S_slider.val,
                    upper_V_slider.val,
                ],
            )
        )


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    curses.wrapper(gui_main, gui_node=gui_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
