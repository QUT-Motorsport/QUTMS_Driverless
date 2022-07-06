from math import floor, ceil
import curses
import time
from typing import Optional

class MoveSlider:
    def __init__() -> None:
        pass

def draw_bars(stdcr,steering_count):
    pass 

def main():
    scalar = 2
    min = 0
    max = 100
    steering_count = max/2
    throttle = 0
    brake = 0
    stdscr = curses.initscr()
    # Make stdscr.getch non-blocking
    stdscr.nodelay(True)
    stdscr.clear()
    curses.noecho()
    stdscr.keypad(True)
    while True:
        c = stdscr.getch()
        # Clear out anything else the user has typed in
        curses.flushinp()
        stdscr.clear()
        # If the user presses left increment bar left
        if c == curses.KEY_LEFT and c == curses.KEY_UP:  
            steering_count -= 1
            throttle += 1
            if steering_count < min: steering_count = min
            if throttle > 100 : throttle = 100
        if c == curses.KEY_RIGHT and c == curses.KEY_UP:
            steering_count += 1
            throttle += 1
            if steering_count < min: steering_count = min
            if throttle > 100 : throttle = 100

        if c == curses.KEY_LEFT and c == curses.KEY_DOWN:  
            steering_count -= 1
            brake += 1
            if steering_count < min: steering_count = min
            if brake > 100 : brake = 100

        if c == curses.KEY_RIGHT and c == curses.KEY_DOWN:
            steering_count += 1
            brake += 1
            if steering_count < min: steering_count = min
            if brake > 100 : brake = 100

        # steering control
        if c == curses.KEY_LEFT:  
            steering_count -= 1
            if steering_count < min: steering_count = min
        if c == curses.KEY_RIGHT: 
            steering_count += + 1
            if steering_count > max: steering_count = max

        # throttle and brake control
        if c == curses.KEY_UP: 
            throttle += 1       
            if throttle > 100 : throttle = 100
        if c == curses.KEY_DOWN:
            brake += 1
            if brake > 100: brake = 100
        
        if c == ord('k'):
            break 

        right = floor((steering_count - min)/scalar)
        left = ceil((max - steering_count)/scalar)
        # Draw bar
        stdscr.addstr(3,4,f'{"░"*right}{"┃"}{"░"*left} \n\tsteering: {float(steering_count/100)} \tthrottle: {float(throttle/100)}\tbrake: {float(brake/100)}')  # 3,4 is location of the bar
        time.sleep(0.1)


if __name__ == "__main__":
    main()