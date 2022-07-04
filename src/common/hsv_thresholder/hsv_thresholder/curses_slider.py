import curses
from math import ceil, floor

from typing import Optional


class CursesSlider:
    label: str
    scalar: str
    min_val: int
    max_val: int
    row: int
    col: int

    val: int

    def __init__(
        self,
        label: str,
        min_val: int,
        max_val: int,
        row: int,
        col: int,
        initial_val: Optional[int] = None,
        scalar: int = 1,
    ) -> None:
        self.label = label
        self.scalar = scalar
        self.min_val = min_val
        self.max_val = max_val
        self.row = row
        self.col = col
        self.val = initial_val if initial_val is not None else min_val

    def __iadd__(self, i: int):
        self.val += i
        if self.val > self.max_val:
            self.val = self.max_val
        return self

    def __isub__(self, i: int):
        self.val -= i
        if self.val < self.min_val:
            self.val = self.min_val
        return self

    def draw(self, stdscr, colour_pair: int):
        filled = floor((self.val - self.min_val) / self.scalar)
        unfilled = ceil((self.max_val - self.val) / self.scalar)

        stdscr.addstr(
            self.row, self.col, f"{self.label}: {'█'*filled}{'░'*unfilled} {self.val}", curses.color_pair(colour_pair)
        )
