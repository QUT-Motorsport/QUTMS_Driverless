# YOU DONT WANT IT TO BE ABOUT HOW FAR AWAY THE POINT IS
# YOU WANT HOW HIGH ABOVE WHATEVER THE CLOSEST LINE IS

import numpy as np

from .. import constants as const


def get_closest_line(ground_set, bin_idx):
    # ground_set[idx][4] denotes starting bin for line
    line_count = len(ground_set)

    idx = 0
    while idx < line_count - 1 and bin_idx < ground_set[idx][4]:
        idx += 1

    return ground_set[idx]


def get_point_line_dist_2(ground_line, point_norm, point_z):
    gradient = ground_line[0]
    intercept = ground_line[1]
    new_point = gradient * point_norm + intercept

    return abs(point_z - new_point)


def sort_segments(segments, seg_bin_z_ind):
    segments_sorted = segments[seg_bin_z_ind]

    # Indicies where segments differ
    seg_sorted_diff = np.where(segments_sorted[:-1] != segments_sorted[1:])[0] + 1

    # Indicies where segments differ (appending first element at 0)
    seg_sorted_ind = np.empty(seg_sorted_diff.size + 1, dtype=int)
    seg_sorted_ind[0] = 0
    seg_sorted_ind[1:] = seg_sorted_diff

    return seg_sorted_ind, segments_sorted


def bisect_left(a, x, lo=0, hi=None, *, key=None):
    """Return the index where to insert item x in list a, assuming a is sorted.

    The return value i is such that all e in a[:i] have e < x, and all e in
    a[i:] have e >= x.  So if x already appears in the list, a.insert(i, x) will
    insert just before the leftmost x already there.

    Optional args lo (default 0) and hi (default len(a)) bound the
    slice of a to be searched.
    """

    if lo < 0:
        raise ValueError("lo must be non-negative")
    if hi is None:
        hi = len(a)
    # Note, the comparison uses "<" to match the
    # __lt__() logic in list.sort() and in heapq.
    if key is None:
        while lo < hi:
            mid = (lo + hi) // 2
            if a[mid] < x:
                lo = mid + 1
            else:
                hi = mid
    else:
        while lo < hi:
            mid = (lo + hi) // 2
            if key(a[mid]) < x:
                lo = mid + 1
            else:
                hi = mid
    return lo


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


def label_points(point_cloud, point_norms, seg_bin_z_ind, segments, ground_plane, bins):
    # Map segments with no ground lines to the nearest segment with a ground line
    mapped_segments = map_segments(ground_plane, const.SEGMENT_COUNT)

    # Get indices where sorted segments differ
    seg_sorted_ind, segments_sorted = sort_segments(segments, seg_bin_z_ind)

    # Split point_norms, point_z and bins into segments
    split_point_norms = np.split(point_norms[seg_bin_z_ind], seg_sorted_ind)
    split_point_z = np.split(point_cloud["z"][seg_bin_z_ind], seg_sorted_ind)
    split_bins = np.split(bins[seg_bin_z_ind], seg_sorted_ind)

    counter = 0
    point_labels = np.empty(point_cloud.shape[0], dtype=bool)
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
            # point_line_dist = get_point_line_dist_2(ground_line, point_norm, point_z)
            point_line_dist = point_z - (
                ground_line[0] * ((bin_idx - ground_line[4]) * const.BIN_SIZE) + ground_line[1]
            )

            is_non_ground = True
            # this doesn't make sense ( ... and point_line_dist < T_D_MAX)
            if point_line_dist < const.T_D_GROUND:
                is_non_ground = False

            point_labels[counter] = is_non_ground
            counter += 1

    return point_labels

def map_segments_2(ground_plane):
    non_zeros = np.nonzero(ground_plane)[0]

    for i in range(len(ground_plane)):
        distances = np.abs(non_zeros - i)
        closest_idx = np.min(np.where(distances == np.min(distances)))
        ground_plane[i] = ground_plane[closest_idx]

    return ground_plane
