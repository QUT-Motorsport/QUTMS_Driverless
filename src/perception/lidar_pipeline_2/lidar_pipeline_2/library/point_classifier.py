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
    point_line_dist = np.zeros(SEGMENT_COUNT, dtype=object)
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

from bisect import bisect_left

# Adapted from www.stackoverflow.com Lauritz V. Thaulow
def take_closest(myList, myNumber):
    """
    Assumes myList is sorted. Returns closest value to myNumber.

    If two numbers are equally close, return the smallest number.
    """
    pos = bisect_left(myList, myNumber)
    if pos == 0:
        return myList[0]
    if pos == len(myList):
        return myList[-1]
    before = myList[pos - 1]
    after = myList[pos]
    if after - myNumber < myNumber - before:
        return after
    else:
        return before


# https://stackoverflow.com/questions/50727961/shortest-distance-between-a-point-and-a-line-in-3-d-space
def point_to_line_dist(point, line_start, line_end):
    def t(line_start, line_end, point):
        x = line_start - line_end
        return np.dot(point - line_end, x) / np.dot(x, x)

    def d(line_start, line_end, point):
        return np.linalg.norm(t(line_start, line_end, point) * (line_start - line_end) + line_end - point)

    return d(line_start, line_end, point)


def map_segments(ground_plane, SEGMENT_COUNT):
    # Indices of segments without ground lines
    empty_segments = [idx for idx, lines in enumerate(ground_plane) if not lines]

    # Indices of segments with ground lines
    non_empty_segments = [idx for idx, lines in enumerate(ground_plane) if lines]

    # [0, 1, ..., 126, 127]
    segments_full = list(range(SEGMENT_COUNT))
    for empty_segment in empty_segments:
        closest_idx = take_closest(non_empty_segments, empty_segment)
        segments_full[empty_segment] = closest_idx
    
    return segments_full


def sort_segments(segments, seg_bin_z_ind):
    segments_sorted = segments[seg_bin_z_ind]

    # Indicies where segments differ
    seg_sorted_diff = np.where(segments_sorted[:-1] != segments_sorted[1:])[0] + 1
    
    # Indicies where segments differ (appending first element at 0)
    seg_sorted_ind = np.empty(seg_sorted_diff.size + 1, dtype=int)
    seg_sorted_ind[0] = 0
    seg_sorted_ind[1:] = seg_sorted_diff
    
    return seg_sorted_ind, segments_sorted


def get_closest_line(ground_set, bin_idx):
    # ground_set[idx][4] denotes starting bin for line
    line_count = len(ground_set)

    idx = 0
    while idx < line_count - 1 and bin_idx < ground_set[idx][4]:
        idx += 1

    return ground_set[idx]


def get_point_line_dist(ground_line, point_norm, point_z):
    gradient = ground_line[0]
    intercept = ground_line[1]
    ground_point = gradient * point_norm + intercept

    return abs(point_z - ground_point)

# [[m b start_x start_y end_x end_y count], [m b start_x start_y end_x end_y count], ...] size is 128 segments
# [[m b start_point end_point count], [m b start_x start_y end_x end_y count], ...] size is 128 segments
def label_points_3(point_cloud, point_norms, seg_bin_z_ind, segments, ground_plane, SEGMENT_COUNT, DELTA_ALPHA, BIN_SIZE, T_D_MAX, point_count, bins):
    # Map segments with no ground lines to the nearest segment with a ground line
    mapped_segments = map_segments(ground_plane, SEGMENT_COUNT)
    # print('mapped_segments', mapped_segments)

    # Get indices where sorted segments differ
    seg_sorted_ind, segments_sorted = sort_segments(segments, seg_bin_z_ind)
    # print('seg_sorted_ind', seg_sorted_ind)

    # Split point_norms into segments
    split_point_norms = np.split(point_norms[seg_bin_z_ind], seg_sorted_ind)
    # print('split_point_norms', split_point_norms)
    
    split_point_z = np.split(point_cloud['z'][seg_bin_z_ind], seg_sorted_ind)
    # print('split_point_z', split_point_z)
    
    split_bins = np.split(bins[seg_bin_z_ind], seg_sorted_ind)
    # print('split_bins', split_bins)
    
    counter = 0
    point_labels = np.empty(point_count)
    for idx, segment_idx in enumerate(segments_sorted[seg_sorted_ind]):
        mapped_seg_idx = mapped_segments[segment_idx]
        bin_idx_set = split_bins[idx]
        point_norm_set = split_point_norms[idx]
        point_z_set = split_point_z[idx]
        ground_set = ground_plane[mapped_seg_idx]
        
        unq_bin_idx = np.unique(bin_idx_set)
        
        ground_line_dict = dict()
        for bin_idx in unq_bin_idx:
            ground_line = get_closest_line(ground_set, bin_idx)
            ground_line_dict[bin_idx] = ground_line

        for jdx, bin_idx in enumerate(bin_idx_set):
            point_norm = point_norm_set[jdx]
            point_z = point_z_set[jdx]
            ground_line = ground_line_dict[bin_idx]
            point_line_dist = get_point_line_dist(ground_line, point_norm, point_z)
            point_line_dist = 0

            is_ground = False
            if (point_line_dist < T_D_MAX):
                is_ground = True

            point_labels[counter] = is_ground
            counter += 1
    
    return point_labels

# If you have computed what ground line is closest for some bin in some set, 
# and multiple points have the same bin, you probably don't need to keep finding
# which line is closest to bin 4 when you've already done it before
# could probably run through all unique bin values and create a dictionary

# Investigate the empty lists([])
