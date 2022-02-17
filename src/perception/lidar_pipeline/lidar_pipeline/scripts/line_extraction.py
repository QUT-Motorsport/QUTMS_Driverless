# Modules
import math
import copy
from typing import List

RUN_ROS = True
if RUN_ROS:
    from . import total_least_squares
else:
    import total_least_squares


# Constants
T_M = 2*math.pi / 152 # Max angle that will be considered for ground lines
T_M_SMALL = 0         # Angle considered to be a small slope 
T_B = 0.1            # Max y-intercept for a ground plane line 
T_RMSE = 0.2          # Threshold of the Root Mean Square Error of the fit (Recommended: 0.2 - 0.5)

X = 0
Y = 1
Z = 2


def fit_error(m, b, points) -> float:
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
def get_ground_lines(segment, num_bins) -> List[List]:
    lines: List[List] = []
    new_line_points = []
    lines_created = 0
    m_new = None
    b_new = None

    i: int = 0
    while i < num_bins:
        if len(segment[i]) == 2:
            m_new = None
            b_new = None

            new_point = segment[i]
            if len(new_line_points) >= 2:
                temp_line_points = copy.deepcopy(new_line_points)
                temp_line_points.append(new_point)

                [m_new, b_new] = total_least_squares.fit_line(temp_line_points)

                if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, temp_line_points) <= T_RMSE):
                    new_line_points.append(new_point)
                    temp_line_points = []
                else:
                    [m_new, b_new] = total_least_squares.fit_line(new_line_points)

                    if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, temp_line_points) <= T_RMSE):
                        lines.append([m_new, b_new, new_line_points[0], new_line_points[len(new_line_points) - 1], len(new_line_points)])
                        lines_created += 1

                    new_line_points = []
                    i = i - 2
            else:
                if len(new_line_points) == 0 or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0])) <= T_M:
                    new_line_points.append(new_point)
                else:
                    # print("no", new_point, i) # whats this for?
                    pass
        elif len(segment[i]) > 2 or len(segment[i]) == 1:
            # This case should not be possible
            raise ValueError("More than one prototype point has been found in a bin!", "i:", i, "len:", len(segment[i]), "segment[i]:", segment[i])

        i += 1
        
    if len(new_line_points) > 1 and m_new != None and b_new != None:
        lines.append([m_new, b_new, new_line_points[0], new_line_points[len(new_line_points) - 1], len(new_line_points)])

    if (m_new == None and b_new != None) or (m_new != None and b_new == None):
        raise ValueError("how the hell did this happen. Like literally how. it wont, this if statement is unnecessary.")

    return lines


def get_ground_lines_2(seg_proto_points):
    proto_count = len(seg_proto_points)

    estimated_lines = []
    new_line_points = []
    lines_created = 0

    m_new = None
    b_new = None

    idx = 0
    while idx < proto_count:
        if len(seg_proto_points[idx]) == 2:
            m_new = None
            b_new = None

            new_point = seg_proto_points[idx]
            if (len(new_line_points) >= 2):
                new_line_points_copy = copy.deepcopy(new_line_points)
                new_line_points_copy.append(new_point)

                [m_new, b_new] = total_least_squares.fit_line(new_line_points_copy)

                if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, new_line_points_copy) <= T_RMSE):
                    new_line_points.append(new_point)
                    new_line_points_copy = []
                else:
                    [m_new, b_new] = total_least_squares.fit_line(new_line_points)

                    if (abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B) and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
                        estimated_lines.append((m_new, b_new, new_line_points[0][0], new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))
                        lines_created += 1

                    new_line_points = []

                    idx -= 2

            else:
                if len(new_line_points) == 0 or math.atan((new_point[1] - new_line_points[-1][1]) / (new_point[0] - new_line_points[-1][0])) <= T_M:
                    new_line_points.append(new_point)

        idx += 1

    if len(new_line_points) > 1 and m_new != None and b_new != None:
        estimated_lines.append((m_new, b_new, new_line_points[0][0],new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))

    return estimated_lines


# Returns the ground lines for all segments
def get_ground_plane(segments_bins_prototype: List[List[List]], num_segments: int, num_bins: int) -> List[List[List]]:
    ground_plane: List[List[List]] = []
    for i in range(num_segments):
        # print(i, segments_bins_prototype[i])
        ground_plane.append(get_ground_lines(segments_bins_prototype[i], num_bins))
        #ground_plane.append(get_ground_lines_2(segments_bins_prototype[i]))

    return ground_plane

# Notes
# 1. Constant T_M_SMALL and T_B: For small slopes m < T_M_SMALL, 
#    the line’s absolute y-intercept b must not exceed a certain 
#    threshold T_B. This way, plateaus are excluded from the 
#    ground plane. 
# 2. Constant T_RMSE: The root mean square error of the fit must 
#    not exceed a certain threshold T_RMSE. I'm unsure what this
#    value should be yet.
# 3. Constant T_D_PREV: The distance of the first point of a line
#    to the line previously fitted must not exceed T_D_PREV, 
#    enforcing smooth transitions between pairs of successive 
#    lines. 
# 4. get_ground_lines(): Only one prototype point is expected 
#    since each bin is reduced to one point with the lowest z value.
# 5. IMPORTANT: Isn't it possible for one segment to have multiple lines?
#    If so, this algorithm does not return the rho (norm) of the line,
#    nor does it indicate which bin the line starts and stops in.
#    Currently the line norm is set to the max lidar range and one line
#    is assumed per segment, starting from the origin. This has been fixed,
#    each segment CAN have multiple lines with different gradients and
#    intercepts - it's beautiful. 
# 6. i = i - 2  under new_line_points = [] can be changed to i = i - 1 
#    to increase speed of the algorithm slightly. However, there will 
#    be breaks in the ground lines which is undesired.
# 7. The original if statement after the else from if len(new_line_points) >= 2 
#    # if lines_created == 0 or len(new_line_points) != 0 or dist_point_line(new_point, lines[lines_created-2][0], lines[lines_created-2][1]) <= T_D_PREV:
#           new_line_points.append(new_point)
# 8. This was removed from the current if statement (the one that replaced the above ^ )
#       or dist_point_line(new_point, lines[lines_created-2][0], lines[lines_created-2][1]) <= T_D_PREV
