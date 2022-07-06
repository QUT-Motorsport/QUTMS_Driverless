from math import floor, ceil
import curses
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from driverless_msgs.msg import GuiKeyboard


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")
        self.get_logger().info("GUI Node Initalised")\
        self.publisher = self.create_publisher(GuiKeyboard, "", 1)
        

    def publish_threshold(self, threshold: Threshold):
        self.threshold_publisher.publish(String(data=threshold.to_json()))


stdscr = curses.initscr()
def gui_main(stdscr, gui_node: GUINode):
    scalar = 2  # for bar smoothing
    min = 0
    max = 100
    steering_min = -100
    steering_max = 100
    steering_count = steering_max/2
    throttle = 0
    brake = 0
    
    curses.noecho()
    curses.cbreak()
    curses.curs_set(0)
    # Make stdscr.getch non-blocking
    stdscr.nodelay(True)
    stdscr.keypad(True)
    
    while True:
        active_throttle = 0
        active_brake = 0
        c = stdscr.getch()
        # Clear out anything else the user has typed in
        curses.flushinp()
        stdscr.clear()

        # Steering control
        if c == curses.KEY_LEFT:  
            steering_count -= 2 
            if steering_count < min: steering_count = steering_min
        if c == curses.KEY_RIGHT: 
            steering_count += 2
            if steering_count > max: steering_count = steering_max

        # Throttle and brake control
        if c == curses.KEY_UP: 
            throttle = throttle + 5 # 2 secs to reach max  
            active_throttle = throttle
            if throttle > 100: throttle = 100
            active_throttle = throttle
        if c == curses.KEY_DOWN:
            brake = brake + 10 # 1 sec to reach max
            active_brake = brake
            if brake > 100: brake = 100
            active_brake = brake

        if c == ord('k'):
            break 

        # Draw bar
        right = floor((steering_count - steering_min)/scalar)
        left = ceil((steering_max - steering_count)/scalar)
        brake_filled = ceil((active_brake - min)/scalar)
        brake_unfilled = floor((max-active_brake)/scalar)
        throttle_filled = ceil((active_throttle - min)/scalar)
        throttle_unfilled = floor((max - active_throttle)/scalar)

        #Send steering count, active_brake and throttle to subscriber node

        #Draw border around bars
        stdscr.addstr(0,0,f' {"░"*right}{"┃"}{"░"*left} \n\t\t   steering: {(float(steering_count/100))}')            # \tthrottle: {round(float(active_throttle),2)}\tbrake: {round((float(active_brake)),2)}\n\n')
        stdscr.addstr(3,1,f"{'█'*throttle_filled}{'░'*throttle_unfilled} throttle: {active_throttle/100}")
        stdscr.addstr(5,1,f"{'█'*brake_filled}{'░'*brake_unfilled} brake: {active_brake/100}\n\n")
        stdscr.refresh()
            
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)

    curses.wrapper(gui_main, gui_node=GUINode())
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()