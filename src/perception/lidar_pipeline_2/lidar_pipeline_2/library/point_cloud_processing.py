# Import Helper Modules
import numpy as np


def get_discretised_positions(point_cloud, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud[:, 1] / point_cloud[:, 0]) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the normal of each point
    point_norms = np.linalg.norm([point_cloud[:, 0], point_cloud[:, 1]], axis=0)
    np.nan_to_num(point_norms, copy=False, nan=0)

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and z coords into one array
    return np.column_stack((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms, point_cloud['z']))


def get_prototype_points(seg_bin_nrm, SEGMENT_COUNT, BIN_COUNT, POINT_COUNT):
    # Sorting norms in descending order by segment idx then bin idx
    seg_bin_nrm = seg_bin_nrm[np.lexsort((seg_bin_nrm[:, 1], seg_bin_nrm[:, 0]))]

    # Split seg_bin_nrm into sub arrays for each unique segment
    split_bin_nrm = np.split(seg_bin_nrm, np.where(np.diff(seg_bin_nrm[:, 0]))[0] + 1)

    segments_approx = [[[] for j in range(BIN_COUNT)] for i in range(SEGMENT_COUNT)]
    for i in range(len(split_bin_nrm)):
        split_split_nrm = np.split(split_bin_nrm[i], np.where(np.diff(split_bin_nrm[i][:, 1]))[0] + 1)
        for j in range(len(split_split_nrm)):
            min_height_idx = np.argsort(split_split_nrm[j][:, 3])[0]
            segments_approx[int(split_split_nrm[j][0, 0])][int(split_split_nrm[j][0, 1])] = [split_split_nrm[j][min_height_idx, 2], split_split_nrm[j][min_height_idx, 3]]

    return segments_approx

# Notes
# 1. For get_prototype_points() if needed you can revert back to using a numpy
#    array and then converting to a python list. It's quicker for operations 
#    but slower overall for conversion. Might still be useful.
#    # segments_approx = np.empty((SEGMENT_COUNT, BIN_COUNT, 2))
#    the segments_approx[...][...] = ... 
#    remains the same for both python and numpy lists / arrays
