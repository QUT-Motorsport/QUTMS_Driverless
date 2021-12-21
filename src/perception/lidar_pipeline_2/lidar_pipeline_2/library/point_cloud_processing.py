# Import Helper Modules
import numpy as np


def get_discretised_positions(point_cloud, DELTA_ALPHA, BIN_SIZE):
    # Calculating the segment index for each point
    segments_idx = np.arctan(point_cloud['y'] / point_cloud['x']) / DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the normal of each point
    point_norms = np.linalg.norm([point_cloud['x'], point_cloud['y']], axis=0)
    np.nan_to_num(point_norms, copy=False, nan=0)

    # Calculating the bin index for each point
    bins_idx = point_norms / BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, and point_norms into one matrix
    return np.column_stack((segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False), point_norms))


def get_prototype_points(seg_bin_nrm, SEGMENT_COUNT, BIN_COUNT, POINT_COUNT):
    # Sorting norms by segment idx then bin idx in reverse
    seg_bin_nrm = seg_bin_nrm[np.lexsort((seg_bin_nrm[:, 1], seg_bin_nrm[:, 0]))]

    split_bin_nrm = np.split(seg_bin_nrm, np.where(np.diff(seg_bin_nrm[:, 0]))[0] + 1)
    
    print(split_bin_nrm)
    
    split_split_nrm = np.split(split_bin_nrm, np.where(np.diff(split_bin_nrm[:, 1]))[0] + 1)

    return seg_bin_nrm
