# Import Helper Modules
import numpy as np


def get_discretised_positions(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and z coords into one array
    return np.column_stack((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms, point_cloud['z']))


def get_prototype_points(seg_bin_nrm_z, SEGMENT_COUNT, BIN_COUNT):
    # Sorting norms in descending order by segment idx then bin idx
    seg_bin_nrm_z = seg_bin_nrm_z[np.lexsort((seg_bin_nrm_z[:, 1], seg_bin_nrm_z[:, 0]))]

    # Split seg_bin_nrm into sub arrays for each unique segment
    split_bin_nrm_z = np.split(seg_bin_nrm_z, np.where(np.diff(seg_bin_nrm_z[:, 0]))[0] + 1)

    # For every bin in every each segment a prototype point may be assigned
    prototype_points = [[[] for j in range(BIN_COUNT)] for i in range(SEGMENT_COUNT)]

    # For each array in split_bin_nrm representing a unique segment
    for i in range(len(split_bin_nrm_z)):
        # Split split_bin_nrm[i] into sub arrays for each unique bin
        split_split_nrm_z = np.split(split_bin_nrm_z[i], np.where(np.diff(split_bin_nrm_z[i][:, 1]))[0] + 1)

        # For each array in split_split_nrm representing a unique bin
        for j in range(len(split_split_nrm_z)):
            # Find index of point with lowest height in current bin and segment
            min_height_idx = np.argsort(split_split_nrm_z[j][:, 3])[0]

            # Set pos corresponding to current segment and bin in nested array
            # to [norm, height] of point corresponding to min_height_idx above
            prototype_points[int(split_split_nrm_z[j][0, 0])][int(split_split_nrm_z[j][0, 1])] = [split_split_nrm_z[j][min_height_idx, 2], split_split_nrm_z[j][min_height_idx, 3]]

    return prototype_points

# Notes
# 1. For get_prototype_points() if needed you can revert back to using a numpy
#    array and then converting to a python list. It's quicker for operations 
#    but slower overall for conversion. Might still be useful.
#    # segments_approx = np.empty((SEGMENT_COUNT, BIN_COUNT, 2))
#    the segments_approx[...][...] = ... 
#    remains the same for both python and numpy lists / arrays
