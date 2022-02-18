# C imports
cimport libc.math as math
cimport numpy as np

# Python imports
import numpy as np
from .cy_library import total_least_squares as tls


cdef float fit_error(float m, float b, list points):
    cdef int num_points = len(points)

    cdef float sse = 0
    cdef float x, best_fit, observed

    cdef int i
    for i in range(num_points):
        x = points[i][0]
        best_fit = m*x + b

        observed = points[i][1]
        sse += (best_fit - observed)**2

    # root mean square return
    return math.sqrt(sse / num_points)


# The Incremental Algorithm
cdef list get_ground_lines_3(list seg_proto_points, float T_M, float T_M_SMALL, float T_B, float T_RMSE, bint REGRESS_BETWEEN_BINS):
    cdef int point_count = len(seg_proto_points)

    cdef list estimated_lines = []
    cdef list new_line_points = []
    cdef int lines_created = 0

    cdef float m_new, b_new
    cdef list new_point
    cdef bint m_b_check
    cdef int idx = 0
    while idx < point_count:
        m_new = 0
        b_new = 0

        new_point = seg_proto_points[idx]
        if (len(new_line_points) >= 2):
            new_line_points.append(new_point)

            m_new, b_new = tls.fit_line(new_line_points)

            m_b_check = abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B)
            if not (m_b_check and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
                new_line_points.pop() # Remove the point we just added

                m_new, b_new = tls.fit_line(new_line_points)

                if (m_b_check and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
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

    if len(new_line_points) > 1 and m_new != 0 and b_new != 0:
        estimated_lines.append((m_new, b_new, new_line_points[0][0],new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))

    return estimated_lines

cpdef np.ndarray get_ground_plane_7(list split_prototype_segments, np.ndarray prototype_segments, int SEGMENT_COUNT, float T_M, float T_M_SMALL, float T_B, float T_RMSE, bint REGRESS_BETWEEN_BINS):
    # Init ground plane
    cdef np.ndarray ground_plane = np.zeros(SEGMENT_COUNT, dtype=object)

    # Computing the ground plane
    cdef list segment
    cdef int seg_counter
    for seg_counter in range(len(split_prototype_segments)):
        segment = split_prototype_segments[seg_counter].tolist()
        ground_plane[prototype_segments[seg_counter]] = get_ground_lines_3(segment, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS)

    return ground_plane
