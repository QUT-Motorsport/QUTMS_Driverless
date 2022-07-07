# car control no auto decrementation of throttle, brake or steering
from math import floor, ceil
import curses
import time
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand


class GUINode(Node):
    def __init__(self):
        super().__init__("gui")
        self.get_logger().info("GUI Node Initalised")
        self.keypress_publisher = self.create_publisher(ControlCommand, "/control_command", 10) #sending steering values to sim, control command are the messages being sent to the topic
        
    def publish_controls(self, control_message):
        self.keypress_publisher.publish(control_message)


def gui_main(stdscr, gui_node: GUINode):
    scalar = 5
    _scalar = 10   # bar of throttle and brake half the size of steering bar
    min = 0        # throttle & brake
    max = 100
    steering_min = -50
    steering_max = 50
    steering_count = 0
    brake = 0
    throttle = 0
    active_throttle = 0
    active_brake = 0
    steering_count = 0

    curses.noecho() 
    curses.cbreak() # no need to press enter to get input
    curses.curs_set(0) 
    stdscr.nodelay(True) # no need to wait for input
    stdscr.keypad(True) 
    


    while True:
        control_msg = ControlCommand()
        c = stdscr.getch()
        #curses.flushinp()
        stdscr.clear() # clear the screen

        # Steering control
        if c == curses.KEY_LEFT:
            steering_count -= 10
            if steering_count < steering_min: steering_count = steering_min

        if c == curses.KEY_RIGHT:
            steering_count += 10
            if steering_count > steering_max: steering_count = steering_max

        if c == curses.KEY_UP:
            throttle = throttle + 10 # 2 secs to reach max/min 
            active_throttle = throttle
            if throttle > 100: throttle = 100
            active_throttle = throttle

        # Throttle and brake control
        if c == curses.KEY_DOWN:
            brake = brake + 15 # 1 sec to reach max
            active_brake = brake
            if brake > 100: brake = 100
            active_brake = brake

        if c == ord('r'):
            active_throttle = 0
            active_brake = 0
            steering_count = 0
            throttle = 0
            brake = 0
        
        if c == ord('k'):
            break
            
        right = floor((steering_count - steering_min)/scalar)
        left = ceil((steering_max - steering_count)/scalar)
        brake_filled = ceil((active_brake - min)/_scalar)
        brake_unfilled = floor((max-active_brake)/_scalar)
        throttle_filled = ceil((active_throttle - min)/_scalar)
        throttle_unfilled = floor((max - active_throttle)/_scalar)

        # boxed controls
        stdscr.addstr(0,1,"+-------------------------+")
        stdscr.addstr(1,1,"|                         |")
        stdscr.addstr(2,1,"|        STEERING         |")
        stdscr.addstr(3,1,f'{"|"}  {"░"*right}{"┃"}{"░"*left}  {"|"}')
        stdscr.addstr(4,1,f'|           {(float(steering_count/100))}\t   |')
        stdscr.addstr(5,1,"|                         |")
        stdscr.addstr(6,1,f'{"|"}   T:  {"█"*throttle_filled}{"░"*throttle_unfilled} {active_throttle/100}\t   {"|"}')
        stdscr.addstr(7,1,"|                         |")
        stdscr.addstr(8,1,f'{"|"}   B:  {"█"*brake_filled}{"░"*brake_unfilled} {round(active_brake/100,1)}\t   {"|"}')
        stdscr.addstr(9,1,"|        r: reset         |")
        stdscr.addstr(10,1,"+-------------------------+")
        stdscr.refresh() 
        
        control_msg.throttle = float(active_throttle/100)
        control_msg.steering = float(steering_count/100)
        control_msg.brake = float(active_brake/100) 
        gui_node.publish_controls(control_msg)

        time.sleep(0.05)


def main():
    rclpy.init()
    node = GUINode()
    curses.wrapper(gui_main, gui_node=node)  #stdscr = curses.initscr() intialised with curses.wrapper in gui_main
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()