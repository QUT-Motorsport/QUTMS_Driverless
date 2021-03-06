import logging
import math
import time

import matplotlib.pyplot as plt
import numpy as np

from . import ground_plane_estimator as gpe
from . import object_processor as op
from . import point_classifier as pc
from . import point_cloud_processor as pcp
from . import visualiser as vis
from . import worlds_worst_code as ihateuni

LOGGER = logging.getLogger(__name__)


def detect_cones(
    point_cloud,
    point_norms,
    LIDAR_RANGE,
    DELTA_ALPHA,
    BIN_SIZE,
    T_M,
    T_M_SMALL,
    T_B,
    T_RMSE,
    REGRESS_BETWEEN_BINS,
    T_D_GROUND,
    T_D_MAX,
    point_count,
    create_figures,
    show_figures,
    animate_figures,
    model_car,
    print_logs,
    stdout_handler,
    working_dir,
    timestamp,
):

    # Printing logs to terminal
    if print_logs:
        LOGGER.addHandler(stdout_handler)

    LOGGER.info("Hi from lidar_manager.")

    np.set_printoptions(suppress=True)
    np.set_printoptions(threshold=np.inf)

    # Derived Constants
    SEGMENT_COUNT = math.ceil(2 * math.pi / DELTA_ALPHA)
    BIN_COUNT = math.ceil(LIDAR_RANGE / BIN_SIZE)

    if create_figures:
        vis.plot_point_cloud_2D(point_cloud, point_count, working_dir, timestamp)
        vis.plot_point_cloud_3D(point_cloud, point_count, working_dir, timestamp, animate_figures, model_car)
        pass

    # Discretise point cloud for real-time performance
    start_time = time.perf_counter()
    # segments_bins_nrm_xyz = pcp.get_discretised_positions(point_cloud, point_norms, DELTA_ALPHA, BIN_SIZE)
    segments, bins = pcp.get_discretised_positions_2(
        point_cloud["x"], point_cloud["y"], point_norms, DELTA_ALPHA, BIN_SIZE
    )
    end_time = time.perf_counter()

    LOGGER.info(f"Numpy PointCloud discretised in {end_time - start_time}s")

    if create_figures:
        # vis.plot_segments_2D(point_cloud, segments, working_dir, timestamp)
        # vis.plot_bins_2D(point_cloud, bins, working_dir, timestamp)
        # vis.plot_segments_3D(point_cloud, segments, working_dir, timestamp, animate_figures)
        # vis.plot_bins_3D(point_cloud, bins, working_dir, timestamp, animate_figures)
        pass

    # Calculate prototype point for every bin (if one exists)
    start_time = time.perf_counter()
    # prototype_points, split_bin_nrm_z = pcp.get_prototype_points_2(segments_bins_nrm_xyz)
    split_prototype_segments, prototype_segments, seg_bin_z_ind = pcp.get_prototype_points_4(
        segments, bins, point_norms, point_cloud["z"]
    )
    end_time = time.perf_counter()

    if create_figures:
        # vis.plot_prototype_points_2D(split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp)
        # vis.plot_prototype_points_3D(split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp, animate_figures)
        pass

    LOGGER.info(f"Prototype Points computed in {end_time - start_time}s")

    start_time = time.perf_counter()
    # ground_plane = gpe.get_ground_plane_3(prototype_points, SEGMENT_COUNT, BIN_COUNT, T_M, T_M_SMALL, T_B, T_RMSE, REGRESS_BETWEEN_BINS)
    ground_plane = gpe.get_ground_plane_7(
        split_prototype_segments,
        prototype_segments,
        SEGMENT_COUNT,
        T_M,
        T_M_SMALL,
        T_B,
        T_RMSE,
        REGRESS_BETWEEN_BINS,
        BIN_SIZE,
    )
    end_time = time.perf_counter()

    if create_figures:
        # vis.plot_ground_plane_2D(ground_plane, split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp)
        # vis.plot_ground_plane_3D(ground_plane, split_prototype_segments, prototype_segments, DELTA_ALPHA, working_dir, timestamp, animate_figures)
        pass

    # print(ground_plane)

    LOGGER.info(f"Ground Plane estimated in {end_time - start_time}s")

    start_time = time.perf_counter()
    point_labels = pc.label_points_4(
        point_cloud,
        point_norms,
        seg_bin_z_ind,
        segments,
        ground_plane,
        SEGMENT_COUNT,
        DELTA_ALPHA,
        BIN_SIZE,
        T_D_GROUND,
        T_D_MAX,
        point_count,
        bins,
    )
    # point_labels = pc.label_points_5(point_cloud, point_norms, seg_bin_z_ind, segments, ground_plane, SEGMENT_COUNT, DELTA_ALPHA, BIN_SIZE, T_D_GROUND, T_D_MAX, point_count, bins)
    # point_labels = pc.label_points_3(point_cloud, point_norms, seg_bin_z_ind, segments, ground_plane, SEGMENT_COUNT, DELTA_ALPHA, BIN_SIZE, T_D_GROUND, T_D_MAX, point_count, bins)
    end_time = time.perf_counter()

    if create_figures:
        vis.plot_labelled_points_2D(
            point_cloud[seg_bin_z_ind], point_labels, ground_plane, DELTA_ALPHA, working_dir, timestamp
        )
        # vis.plot_labelled_points_3D(point_cloud[seg_bin_z_ind], point_labels, ground_plane, DELTA_ALPHA, working_dir, timestamp, animate_figures)
        pass

    LOGGER.info(f"Points labelled in {end_time - start_time}s")

    start_time = time.perf_counter()
    object_points = np.column_stack(
        (point_cloud["x"][seg_bin_z_ind][point_labels], point_cloud["y"][seg_bin_z_ind][point_labels])
    )
    end_time = time.perf_counter()

    if create_figures:
        vis.plot_object_points_2D(object_points)

    LOGGER.info(f"Object points extracted and stacked in {end_time - start_time}s")

    start_time = time.perf_counter()
    object_centers = op.group_points(object_points)
    end_time = time.perf_counter()

    if create_figures:
        vis.plot_object_centers_2D(object_points, object_centers)

    LOGGER.info(f"Object centers computed in {end_time - start_time}s")

    start_time = time.perf_counter()
    reconstructed_objects = op.reconstruct_objects(
        np.column_stack((point_cloud["x"], point_cloud["y"], point_cloud["z"])),
        object_centers,
        DELTA_ALPHA,
        0.4,
        BIN_SIZE,
    )
    end_time = time.perf_counter()

    if create_figures:
        vis.plot_reconstructed_objects_2D(reconstructed_objects)

    LOGGER.info(f"Objects reconstructed in {end_time - start_time}s")

    start_time = time.perf_counter()
    # identified_cones = op.cone_filter(reconstructed_objects)
    identified_cones = ihateuni.get_cones(reconstructed_objects)
    end_time = time.perf_counter()

    if create_figures:
        vis.plot_identified_cones_2D(point_cloud, identified_cones)

    LOGGER.info(f"Objects filtered for cones in {end_time - start_time}s")

    if show_figures:
        plt.show()

    return identified_cones


# Look into numpy views and see if they could be used instead of creating
# entirely new variables
# maybe add equal axes to figures
# only import what's necessary for each file

# aight so move timings so they ignore figures
# perhaps even add figure timings

# see if there's anywhere else you can use the multi-access
# structured array indexing point_cloud[['x', 'y', 'z']]

# moar visoolisations
