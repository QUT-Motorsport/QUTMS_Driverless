import numpy as np

from .. import constants as const


def get_discretised_positions(x, y, point_norms):
    # Calculating the segment index for each point
    segments_idx = np.arctan2(y, x) / const.DELTA_ALPHA
    np.nan_to_num(segments_idx, copy=False, nan=((np.pi / 2) / const.DELTA_ALPHA))  # Limit arctan x->inf = pi/2

    # Calculating the bin index for each point
    bins_idx = point_norms / const.BIN_SIZE

    # Stacking arrays segments_idx, bins_idx, point_norms, and xyz coords into one array
    return segments_idx.astype(int, copy=False), bins_idx.astype(int, copy=False)


# In LPP 2, np.absolute(z) is used. I'm not sure why this was the case.
# If a point is negative, i.e., low, that's still valid.
def get_prototype_points(z, segments, bins, point_norms):
    # Indicies sorted by segments, then bins, then z (height)
    seg_bin_z_ind = np.lexsort((z, bins, segments))

    # Indicies where neighbouring bins in array are different
    bin_diff_ind = np.where((bins[seg_bin_z_ind])[:-1] != (bins[seg_bin_z_ind])[1:])[0] + 1

    # Indicies of prototype points
    proto_sorted_ind = np.empty(bin_diff_ind.size + 1, dtype=int)
    proto_sorted_ind[0] = seg_bin_z_ind[0]
    proto_sorted_ind[1:] = seg_bin_z_ind[bin_diff_ind]

    # Prototype points and segment idx corresponding to each
    proto_points = np.column_stack((point_norms[proto_sorted_ind], z[proto_sorted_ind]))
    proto_segments = segments[proto_sorted_ind]

    # Indicies where neighbouring prototype_segments value in array are different
    proto_segs_diff = np.where(proto_segments[:-1] != proto_segments[1:])[0] + 1

    # Prototype points split into subarrays for each segment
    proto_segs_arr = np.split(proto_points, proto_segs_diff)

    return proto_segs_arr, proto_segments[np.concatenate((np.array([0]), proto_segs_diff))], seg_bin_z_ind
