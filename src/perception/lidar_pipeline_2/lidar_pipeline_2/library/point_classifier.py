# Import Python Modules
import numpy as np
import math


# Returns the start and end points (x, y, z) of a line
def line_to_end_points(line, segment_idx, DELTA_ALPHA):
    start = line[2]  # First point in line
    end = line[3]  # Last point in line

    x_1 = start[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)
    x_2 = end[0] * math.cos((segment_idx + 0.5) * DELTA_ALPHA)

    y_1 = start[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)
    y_2 = end[0] * math.sin((segment_idx + 0.5) * DELTA_ALPHA)

    p_1 = [x_1, y_1, start[1]]
    p_2 = [x_2, y_2, end[1]]

    return [p_1, p_2]


# https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
def dist_points_3D(x_0, p_1, p_2):
    # get distance in each dimension from x to point 1
    p_1_dist = [(x_0[0] - p_1[0]), (x_0[1] - p_1[1]), (x_0[2] - p_1[2])]

    # get distance in each dimension from x to point 2
    p_2_dist = [(x_0[0] - p_2[0]), (x_0[1] - p_2[1]), (x_0[2] - p_2[2])]

    # cross product of dimension distances
    dist_cross = [
        p_1_dist[1]*p_2_dist[2] - p_2_dist[1]*p_1_dist[2], 
        -(p_1_dist[0]*p_2_dist[2] - p_2_dist[0]*p_1_dist[2]), 
        p_1_dist[0]*p_2_dist[1] - p_2_dist[0]*p_1_dist[1]
    ]

    # normalise (pythag) each cross
    dist_norm: float = math.sqrt(dist_cross[0]**2 + dist_cross[1]**2 + dist_cross[2]**2)

    # normalise point 1 and 2 distances
    p_norm: float = math.sqrt(
        (p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2 + (p_2[2] - p_1[2])**2
    )

    # return distance
    return dist_norm / p_norm


# Modifies input
# Conservative approach implemented using T_D_MAX parameter
def label_points(segments_bins, ground_lines, DELTA_ALPHA, SEGMENT_COUNT, BIN_COUNT):
    for i in range(SEGMENT_COUNT):
        num_lines = len(ground_lines[i])
        seg_idx = i
        # If there is no ground line in current segment, find the closest one
        if num_lines == 0:
            left_counter = i-1
            right_counter = i+1
            left_idx = (left_counter) % SEGMENT_COUNT
            right_idx = (right_counter) % SEGMENT_COUNT
            while left_idx != right_idx:
                if len(ground_lines[left_idx]) > 0:
                    seg_idx = left_idx
                    break
                elif len(ground_lines[right_idx]) > 0:
                    seg_idx = right_idx
                    break
                left_counter -= 1
                right_counter += 1
                left_idx = (left_counter) % SEGMENT_COUNT
                right_idx = (right_counter) % SEGMENT_COUNT
            if left_idx == right_idx:
                raise AssertionError("No ground lines found")
        ground_line = ground_lines[seg_idx][0]

        for j in range(BIN_COUNT):
            for k in range(len(segments_bins[i][j])):
                point = segments_bins[i][j][k]
                is_ground = False
                line_height = ground_line[0] * (j * BIN_COUNT) + ground_line[1]
                if point[2] < line_height + 0.10: # Make this a constant
                    line = line_to_end_points(ground_line, seg_idx)
                    closest_dist = dist_points_3D(point, line[0], line[1])

                    for m in range(1, num_lines):
                        ground_line = ground_lines[seg_idx][m]
                        line = line_to_end_points(ground_line, seg_idx)

                        dist_to_line = dist_points_3D(point, line[0], line[1])
                        if (dist_to_line < closest_dist):
                            closest_dist = dist_to_line

                    dynamic_T_D_GROUND = 2*((j + 1) * BIN_COUNT)*math.tan(DELTA_ALPHA/2) + BIN_COUNT * 1.3 # Solved for gradient of segment wrt points and distance
                    if (closest_dist < T_D_MAX and closest_dist < dynamic_T_D_GROUND):
                        is_ground = True
                segments_bins[i][j][k].append(is_ground)

    return segments_bins


def get_nearest_line_set(ground_plane):
    # Array of indices indicating lists of ground lines
    line_set_idx = np.argwhere(ground_plane)

    # 2D matrix of distances from each segment to each list of ground lines
    neighbour_distance = np.arange(0, ground_plane.size) - line_set_idx
    np.absolute(neighbour_distance, out=neighbour_distance)

    # Array of indices indicating nearest lists of ground lines for each segment
    nearest_idx = np.where(neighbour_distance == neighbour_distance.min(axis=0))

    # If a segment is equally close to two lists of ground lines, choose the first
    unique, u_idx = np.unique(nearest_idx[1], return_index=True)
    return nearest_idx[0][u_idx], line_set_idx


def get_line_end_points(ground_plane, line_set_idx, DELTA_ALPHA):
    line_end_points = np.empty((line_set_idx.size, 2, 1), dtype=object)

    # Changes from 2D approx to 3D approx
    for i in range(line_set_idx.size):
        line_set = np.array(ground_plane[line_set_idx][i][0])

        segment_idx = line_set_idx[i]

        norm_start_set = line_set[:, 2]
        norm_end_set = line_set[:, 4]

        height_start_set = line_set[:, 3]
        height_end_set = line_set[:, 5]

        x_start_set = norm_start_set * np.cos((segment_idx + 0.5) * DELTA_ALPHA)
        x_end_set = norm_end_set * np.cos((segment_idx + 0.5) * DELTA_ALPHA)

        y_start_set = norm_start_set * np.sin((segment_idx + 0.5) * DELTA_ALPHA)
        y_end_set = norm_end_set * np.sin((segment_idx + 0.5) * DELTA_ALPHA)

        start_point_set = [np.column_stack((x_start_set, y_start_set, height_start_set))]
        end_point_set = [np.column_stack((x_end_set, y_end_set, height_end_set))]

        line_end_points[i][0] = start_point_set
        line_end_points[i][1] = end_point_set

    return line_end_points


def get_point_line_dist(line_end_points, nearest_line_set, split_bin_nrm_z, SEGMENT_COUNT):
    # For each point, find closest line
    point_line_dist = np.empty(SEGMENT_COUNT, dtype=object)
    for point_set in split_bin_nrm_z:
        seg_idx = int(point_set[0, 0])

        nearest_line = nearest_line_set[seg_idx]

        line_start = line_end_points[nearest_line][0][0]
        line_end = line_end_points[nearest_line][1][0]

        end_minus_start = line_end - line_start

        distances = []
        for j in range(line_start.shape[0]):
            distances.append(np.abs(np.cross(end_minus_start[j], line_start[j] - point_set[:, 3:6])) / np.linalg.norm(end_minus_start[j]))

        point_line_dist[seg_idx] = distances

    return point_line_dist


def label_points_2(ground_plane, split_bin_nrm_z, DELTA_ALPHA, SEGMENT_COUNT):
    nearest_line_set, line_set_idx = get_nearest_line_set(ground_plane)
    line_end_points = get_line_end_points(ground_plane, line_set_idx, DELTA_ALPHA)
    point_line_dist = get_point_line_dist(line_end_points, nearest_line_set, split_bin_nrm_z, SEGMENT_COUNT)

    return []


def dist_points_3D(x_0, p_1, p_2):
    # get distance in each dimension from x to point 1
    p_1_dist = [(x_0[0] - p_1[0]), (x_0[1] - p_1[1]), (x_0[2] - p_1[2])]

    # get distance in each dimension from x to point 2
    p_2_dist = [(x_0[0] - p_2[0]), (x_0[1] - p_2[1]), (x_0[2] - p_2[2])]

    # cross product of dimension distances
    dist_cross = [
        p_1_dist[1]*p_2_dist[2] - p_2_dist[1]*p_1_dist[2], 
        -(p_1_dist[0]*p_2_dist[2] - p_2_dist[0]*p_1_dist[2]), 
        p_1_dist[0]*p_2_dist[1] - p_2_dist[0]*p_1_dist[1]
    ]

    # normalise (pythag) each cross
    dist_norm: float = math.sqrt(dist_cross[0]**2 + dist_cross[1]**2 + dist_cross[2]**2)

    # normalise point 1 and 2 distances
    p_norm: float = math.sqrt(
        (p_2[0] - p_1[0])**2 + (p_2[1] - p_1[1])**2 + (p_2[2] - p_1[2])**2
    )

    # return distance
    return dist_norm / p_norm
