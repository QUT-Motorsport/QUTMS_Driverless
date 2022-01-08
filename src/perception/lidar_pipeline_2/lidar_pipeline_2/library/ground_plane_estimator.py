# Import Custom Modules
from . import total_least_squares as tls

# Python Modules
import numpy as np
import math
import copy


def fit_error(m, b, points):
    num_points = len(points)

    sse = 0
    for i in range(num_points):
        x = points[i][0]
        best_fit = m*x + b

        observed = points[i][1]
        sse += (best_fit - observed)**2

    # root mean square return
    return math.sqrt(sse / num_points)


# The Incremental Algorithm
def get_ground_lines_2(seg_proto_points, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS):
    proto_count = len(seg_proto_points)

    estimated_lines = []
    new_line_points = []
    lines_created = 0

    m_new = None
    b_new = None

    idx = 0
    while idx < proto_count:

        new_point = seg_proto_points[idx]
        if len(new_point) == 2:
            m_new = None
            b_new = None

            if (len(new_line_points) >= 2):
                new_line_points_copy = copy.deepcopy(new_line_points)
                new_line_points_copy.append(new_point)

                [m_new, b_new] = tls.fit_line(new_line_points_copy)

                if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, new_line_points_copy) <= T_RMSE):
                    new_line_points.append(new_point)
                    new_line_points_copy = []
                else:
                    [m_new, b_new] = tls.fit_line(new_line_points)

                    if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
                        estimated_lines.append((m_new, b_new, new_line_points[0][0], new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))
                        lines_created += 1

                    new_line_points = []

                    if REGRESS_BETWEEN_BINS:
                        idx -= 2
                    else:
                        idx -= 1

            else:
                if len(new_line_points) == 0 or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0])) <= T_M:
                    new_line_points.append(new_point)

        idx += 1

    if len(new_line_points) > 1 and m_new != None and b_new != None:
        estimated_lines.append((m_new, b_new, new_line_points[0][0],new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))

    return estimated_lines


def get_ground_plane_3(prototype_points, SEGMENT_COUNT, BIN_COUNT, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS):
    # A numpy array of zeros / lists that contain ground lines for each segment
    ground_plane = np.zeros(SEGMENT_COUNT, dtype=object)

    # For every segment
    for segment in prototype_points:
        ground_plane[int(segment[0])] = get_ground_lines_2(segment[1:], T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS)

    return ground_plane
# Instead of packing the start and end points as [], unpack them and just make the overall array bigger
# Means you can also get rid of the dtype=object and just make it floats or something - should be faster
# Notes:
# 1. Ground plane could be a numpy array that is created to be of size
#    SEGMENT_COUNT. Then each entry to be return of get_ground_lines. Then
#    this array can be used in label_points function which is currently the
#    slowest function.
# 2. Changed the fit error points array in the second if statement check
#    MUST investigate this.
# 3. Idk if I use the len(new_line_points) in estimted lines i.e. line[4] ?
