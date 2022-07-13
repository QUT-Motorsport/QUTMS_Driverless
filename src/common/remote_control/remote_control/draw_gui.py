from math import ceil, floor

# draw the gui
class Gui:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.scalar = 5  # bar sizes (larger = smaller bar)
        self._scalar = 10 # T & B
        self.min = 0   # throttle & brake
        self.max = 100
        self.steering_min = -50
        self.steering_max = 50

    def draw_gui(self, steering_count, active_throttle, active_brake):
        self.steering_count = steering_count
        self.active_throttle = active_throttle
        self.active_brake = active_brake

        right = floor((self.steering_count - self.steering_min)/self.scalar)
        left = ceil((self.steering_max - self.steering_count)/self.scalar)
        brake_filled = ceil((self.active_brake - self.min)/self._scalar)
        brake_unfilled = floor((self.max-self.active_brake)/self._scalar)
        throttle_filled = ceil((self.active_throttle - self.min)/self._scalar)
        throttle_unfilled = floor((self.max - self.active_throttle)/self._scalar)

        #draw steering gui
        self.stdscr.addstr(0,1,"+-------------------------+")
        self.stdscr.addstr(1,1,"|                         |")
        self.stdscr.addstr(2,1,"|        STEERING         |")
        self.stdscr.addstr(3,1,f'{"|"}  {"░"*right}{"┃"}{"░"*left}  {"|"}')
        self.stdscr.addstr(4,1,f'|           {(float(self.steering_count/100))}\t   |')
        self.stdscr.addstr(5,1,"|                         |")
        self.stdscr.addstr(6,1,f'{"|"}   T:  {"█"*throttle_filled}{"░"*throttle_unfilled} {self.active_throttle/100}\t   {"|"}')
        self.stdscr.addstr(7,1,"|                         |")
        self.stdscr.addstr(8,1,f'{"|"}   B:  {"█"*brake_filled}{"░"*brake_unfilled} {round(self.active_brake/100,1)}\t   {"|"}')
        self.stdscr.addstr(9,1,"|                         |")
        self.stdscr.addstr(10,1,"+-------------------------+")  
        self.stdscr.refresh()