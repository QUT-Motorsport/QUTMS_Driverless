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

# The Incremental Algorithm
def get_ground_lines(proto_seg_points):
    estimated_lines = []
    new_line_points = []
    lines_created = 0

    idx = 0
    while idx < len(proto_seg_points):
        m_new = None
        b_new = None

        new_point = proto_seg_points[idx]
        if len(new_line_points) >= 2:
            new_line_points.append(new_point)

            [m_new, b_new] = tls.fit_line(new_line_points)

            m_b_check = abs(m_new) <= const.T_M and (abs(m_new) > const.T_M_SMALL or abs(b_new) <= const.T_B)
            if not (m_b_check and fit_error(m_new, b_new, new_line_points) <= const.T_RMSE):
                new_line_points.pop()  # Remove the point we just added

                [m_new, b_new] = tls.fit_line(new_line_points)

                m_b_check = abs(m_new) <= const.T_M and (abs(m_new) > const.T_M_SMALL or abs(b_new) <= const.T_B)
                if m_b_check and fit_error(m_new, b_new, new_line_points) <= const.T_RMSE:
                    estimated_lines.append(
                        (
                            m_new,
                            b_new,
                            new_line_points[0],
                            new_line_points[-1],
                            get_bin(new_line_points[0][0], const.BIN_SIZE),
                        )
                    )
                    lines_created += 1

                new_line_points = []

                if const.REGRESS_BETWEEN_BINS:
                    idx -= 2
                else:
                    idx -= 1

        else:
            if (
                len(new_line_points) == 0
                or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0]))
                <= const.T_M
            ):
                new_line_points.append(new_point)

        idx += 1

    if len(new_line_points) > 1 and m_new != None and b_new != None:
        estimated_lines.append(
            (m_new, b_new, new_line_points[0], new_line_points[-1], get_bin(new_line_points[0][0], const.BIN_SIZE))
        )

    # If no ground lines were identified in segment, return 0
    if len(estimated_lines) > 0:
        return estimated_lines
    else:
        return 0


def get_ground_plane_single_core(proto_segs_arr, proto_segs):
    # Computing the ground plane
    ground_plane = np.zeros(const.SEGMENT_COUNT, dtype=object)
    for segment_counter in range(len(proto_segs_arr)):
        proto_seg_points = proto_segs_arr[segment_counter].tolist()
        ground_plane[proto_segs[segment_counter]] = get_ground_lines(proto_seg_points)

    return ground_plane
