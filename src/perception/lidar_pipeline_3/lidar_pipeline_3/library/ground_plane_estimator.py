import math
import multiprocessing as mp

import numpy as np

from .. import constants as const
from .cy_library import total_least_squares as tls

# Returns bin idx of a point from its norm
def get_bin(norm, BIN_SIZE):
    return math.floor(norm / BIN_SIZE)

# Returns the RMSE of a line fit to a set of points
def fit_error(m, b, points):
    num_points = len(points)

    sse = 0
    for i in range(num_points):
        x = points[i][0]
        best_fit = m * x + b

        observed = points[i][1]
        sse += (best_fit - observed) ** 2

    # root mean square return
    return math.sqrt(sse / num_points)
