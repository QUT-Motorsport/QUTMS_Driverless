from math import ceil, floor


# draw the gui
class Gui:
    steering_scalar = 5  # bar sizes (larger = smaller bar)
    tb_scalar = 10  # throttle & brake scalar
    tb_min = 0  # throttle & brake min
    tb_max = 100
    steering_min = -50
    steering_max = 50

    def __init__(self, stdscr):
        self.stdscr = stdscr

    def draw_gui(self, steering_count: float, active_throttle: float, active_brake: float):

        right = floor((steering_count - self.steering_min) / self.steering_scalar)
        left = ceil((self.steering_max - steering_count) / self.steering_scalar)
        brake_filled = ceil((active_brake - min) / self.tb_scalar)
        brake_unfilled = floor((max - active_brake) / self.tb_scalar)
        throttle_filled = ceil((active_throttle - min) / self.tb_scalar)
        throttle_unfilled = floor((max - active_throttle) / self.tb_scalar)

        # draw steering gui
        self.stdscr.addstr(0, 1, "+-------------------------+")
        self.stdscr.addstr(1, 1, "|                         |")
        self.stdscr.addstr(2, 1, "|        STEERING         |")
        self.stdscr.addstr(3, 1, f'{"|"}  {"░"*right}{"┃"}{"░"*left}  {"|"}')
        self.stdscr.addstr(4, 1, f"|           {(float(steering_count/100))}\t   |")
        self.stdscr.addstr(5, 1, "|                         |")
        self.stdscr.addstr(
            6, 1, f'{"|"}   T:  {"█"*throttle_filled}{"░"*throttle_unfilled} {active_throttle/100}\t   {"|"}'
        )
        self.stdscr.addstr(7, 1, "|                         |")
        self.stdscr.addstr(
            8, 1, f'{"|"}   B:  {"█"*brake_filled}{"░"*brake_unfilled} {round(active_brake/100,1)}\t   {"|"}'
        )
        self.stdscr.addstr(9, 1, "|                         |")
        self.stdscr.addstr(10, 1, "+-------------------------+")
        self.stdscr.refresh()
