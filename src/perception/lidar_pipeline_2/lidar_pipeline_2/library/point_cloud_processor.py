# Import Python Modules
import numpy as np


def get_discretised_positions(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE):

    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and z coords into one array
    return np.column_stack((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms, point_cloud['x'], point_cloud['y'], point_cloud['z']))


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
            prototype_points[int(split_split_nrm_z[j][0, 0])][int(split_split_nrm_z[j][0, 1])] = [float(split_split_nrm_z[j][min_height_idx, 2]), float(split_split_nrm_z[j][min_height_idx, 3])]

    return prototype_points


def get_discretised_positions_2(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
    return segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False)


def get_discretised_positions_3(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
    return np.rec.fromarrays((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms, point_cloud['x'], point_cloud['y'], point_cloud['z']), names=('seg_idx', 'bin_idx', 'norms', 'x', 'y', 'z'))


def get_prototype_points_2(seg_bin_nrm_z):
    # Sorting norms in descending order by segment idx then norm
    seg_bin_nrm_z = seg_bin_nrm_z[np.lexsort((seg_bin_nrm_z[:, 2], seg_bin_nrm_z[:, 0]))]

    # Splitting seg_bin_nrm_z into sub arrays for each unique segment
    split_bin_nrm_z = np.split(seg_bin_nrm_z, np.where(np.diff(seg_bin_nrm_z[:, 0]))[0] + 1)

    seg_idx = 0
    prototype_points = []
    for segment_array in split_bin_nrm_z:
        prototype_points.append([segment_array[0][0]])
        split_split_nrm_z = np.split(segment_array, np.where(np.diff(segment_array[:, 1]))[0] + 1)

        for bin_array in split_split_nrm_z:
            prototype_points[seg_idx].append([float(bin_array[0][2]), float(bin_array[0][5])])

        seg_idx += 1

    return prototype_points, split_bin_nrm_z


def get_prototype_points_3(seg_bin_nrm_xyz):
    # Sorting points by segment idx then height (z) in ascending order
    seg_bin_nrm_xyz = seg_bin_nrm_xyz[np.lexsort((np.absolute(seg_bin_nrm_xyz[:, 5]), seg_bin_nrm_xyz[:, 1], seg_bin_nrm_xyz[:, 0]))]
    
    # Splitting seg_bin_nrm_xyz into sub arrays for each unique segment
    split_bin_nrm_xyz = np.split(seg_bin_nrm_xyz, np.where(np.diff(seg_bin_nrm_xyz[:, 0]))[0] + 1)

    prototype_points = []
    for segment in split_bin_nrm_xyz:
        split_split_nrm_xyz = np.split(segment, np.where(np.diff(segment[:, 1]))[0] + 1)
        
        for bin in split_split_nrm_xyz:
            prototype_points.append(bin[0])
    
    return None, None


def get_prototype_points_4(segments, bins, point_norms, z):
    # Indicies sorted by segments, then bins, then absolute z (height)
    seg_bin_z_ind = np.lexsort((np.absolute(z), bins, segments))

    # Indicies where neighbouring bins in array are different
    bin_diff_ind = np.where((bins[seg_bin_z_ind])[:-1] != (bins[seg_bin_z_ind])[1:])[0] + 1

    # Indicies of prototype points
    proto_sorted_ind = np.empty(bin_diff_ind.size + 1, dtype=int)
    proto_sorted_ind[0] = seg_bin_z_ind[0]
    proto_sorted_ind[1:] = seg_bin_z_ind[bin_diff_ind]

    # Prototype points and segment idx corresponding to each
    prototype_points = np.column_stack((point_norms[proto_sorted_ind], z[proto_sorted_ind]))
    prototype_segments = segments[proto_sorted_ind]

    # Indicies where neighbouring prototype_segments value in array are different
    proto_seg_diff = np.where(prototype_segments[:-1] != prototype_segments[1:])[0] + 1
    
    # Prototype points split into subarrays for each segment
    split_prototype_segments = np.split(prototype_points, proto_seg_diff)

    return split_prototype_segments, prototype_segments


# Notes
# 1. For get_prototype_points() if needed you can revert back to using a numpy
#    array and then converting to a python list. It's quicker for operations
#    but slower overall for conversion. Might still be useful.
#    # segments_approx = np.empty((SEGMENT_COUNT, BIN_COUNT, 2))
#    the segments_approx[...][...] = ...
#    remains the same for both python and numpy lists / arrays
