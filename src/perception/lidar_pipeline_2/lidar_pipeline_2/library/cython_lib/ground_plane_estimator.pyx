# The Incremental Algorithm
def get_ground_lines_3(seg_proto_points, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS):
    estimated_lines = []
    new_line_points = []
    lines_created = 0

    idx = 0
    while idx < len(seg_proto_points):
        m_new = None
        b_new = None

        new_point = seg_proto_points[idx]
        if (len(new_line_points) >= 2):
            new_line_points.append(new_point)

            [m_new, b_new] = tls.fit_line(new_line_points)

            m_b_check = abs(m_new) <= T_M and (abs(m_new) > T_M_SMALL or abs(b_new) <= T_B)
            if not (m_b_check and fit_error(m_new, b_new, new_line_points) <= T_RMSE):
                new_line_points.pop() # Remove the point we just added

                [m_new, b_new] = tls.fit_line(new_line_points)

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

    if len(new_line_points) > 1 and m_new != None and b_new != None:
        estimated_lines.append((m_new, b_new, new_line_points[0][0],new_line_points[0][1], new_line_points[len(new_line_points) - 1][0], new_line_points[len(new_line_points) - 1][1], len(new_line_points)))

    return estimated_lines