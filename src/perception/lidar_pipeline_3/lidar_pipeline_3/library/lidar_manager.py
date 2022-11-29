import time
import matplotlib.pyplot as plt
import numpy as np

from . import ground_plane_estimator as gpe
from . import object_processor as op
from . import point_classifier as pc
from . import point_cloud_processor as pcp
from . import visualiser as vis
from . import visualiser_2 as vis2
from .. import constants as const
from ..utils import Config  # For typing


def locate_cones(config, point_cloud, start_time):
    config.logger.info(f"Point Cloud received with {point_cloud.shape[0]} points")

    # Visualise inital point cloud before filtering
    if config.create_figures:
        config.setup_image_dir()
        vis.plot_point_cloud_2D(config, point_cloud, "00_PointCloud_2D")

    # Remove points behind car
    point_cloud = point_cloud[point_cloud["x"] > 0]

    # Compute point normals
    point_norms = np.linalg.norm([point_cloud["x"], point_cloud["y"]], axis=0)

    # Remove points that are outside of range or have a norm of 0
    mask = point_norms <= const.LIDAR_RANGE  # & (point_norms != 0)
    point_norms = point_norms[mask]
    point_cloud = point_cloud[mask]
    config.logger.info(f"{point_cloud.shape[0]} points remain after filtering point cloud")

    segments, bins = pcp.get_discretised_positions(point_cloud["x"], point_cloud["y"], point_norms)
    config.logger.info("DONE: Segments and Bins")

    proto_segs_arr, proto_segs, seg_bin_z_ind = pcp.get_prototype_points(point_cloud["z"], segments, bins, point_norms)
    config.logger.info("DONE: Prototype Points")

    # Multiprocessing Ground Plane Mapping [m b start(x, y) end(x, y) bin]
    # ground_plane = gpe.get_ground_plane_mp(config, proto_segs_arr, proto_segs)
    ground_plane = gpe.get_ground_plane_single_core(proto_segs_arr, proto_segs)
    config.logger.info("DONE: Ground Plane Mapped")

    # point_labels = pc.label_points(point_cloud, point_norms, seg_bin_z_ind, segments, ground_plane, bins)
    # point_labels = pc.label_points_2(point_cloud, point_norms, segments, bins, seg_bin_z_ind, ground_plane)
    # point_labels = pc.label_points_3(point_cloud, segments, bins, seg_bin_z_ind, ground_plane)
    # point_labels = pc.label_points_4(point_cloud, segments, bins, proto_segs, seg_bin_z_ind, ground_plane)
    # point_labels = pc.label_points_5(point_cloud, segments, bins, seg_bin_z_ind, ground_plane)
    point_labels, ground_lines_arr = pc.label_points_6(point_cloud["z"], segments, bins, seg_bin_z_ind, ground_plane)
    config.logger.info("DONE: Points Labelled")

    object_points = point_cloud[point_labels]
    # object_points = np.column_stack((object_points["x"], object_points["y"], object_points["z"]))
    config.logger.info("DONE: Object Points Grouped")

    if object_points.size == 0:
        config.logger.info("No objects points detected")
        return None

    object_centers, objects = op.group_points(object_points)  # maybe improve speed?
    config.logger.info("DONE: Objects Identified")

    ground_points = point_cloud[~point_labels]
    # reconstructed_objects = op.reconstruct_objects(point_cloud, object_centers, objects, const.DELTA_ALPHA, const.CONE_DIAM, const.BIN_SIZE)
    obj_segs, obj_bins, reconstructed_objects, reconstructed_centers = op.reconstruct_objects_2(
        ground_points, segments[~point_labels], bins[~point_labels], object_centers, objects
    )
    config.logger.info("DONE: Objects Reconstructed")

    cone_centers, cone_points = op.cone_filter(
        segments,
        bins,
        ground_lines_arr,
        obj_segs,
        obj_bins,
        object_centers,
        reconstructed_objects,
        reconstructed_centers,
    )
    config.logger.info("DONE: Cones Identified")

    duration = time.perf_counter() - start_time

    # cones = cones.tolist()
    # for cone in cones:
    #     print(cone)
    # print(const.HALF_AREA_CONE_HEIGHT)

    # Investigate turning structured arrays into normal arrays for better indexing and avoiding column stack
    # actually i think this is fine ^ go back to structured to retain intensity
    # Tune group points, 2 min is great for range, but probs noisy, also slower
    # and now that we have ros bags that are more accurate for track, maybe increase epsilon
    # to known min distance between cones

    # what if entire point cloud was just turned into a n*5 array of floats?
    # remove structured array but keep intentity and ring

    # Create visualisations
    if config.create_figures:
        # vis.plot_point_cloud_2D(config, point_cloud, "01_PointCloud_2D")
        # vis.plot_segments_2D(config, point_cloud, segments, "03_PointCloudSegments_2D")
        # vis.plot_bins_2D(config, point_cloud, bins, "05_PointCloudBins_2D")
        # vis.plot_segments_3D(config, point_cloud, segments, "04_PointCloudSegments_3D")
        # vis.plot_bins_3D(config, point_cloud, bins, "06_PointCloudBins_3D")
        # vis.plot_prototype_points_2D(config, proto_segs_arr, proto_segs, "07_PrototypePoints_2D")
        # vis.plot_prototype_points_3D(config, proto_segs_arr, proto_segs, "08_PrototypePoints_3D")
        # vis.plot_ground_plane_2D(config, ground_plane, proto_segs_arr, proto_segs, "09_GroundPlane_2D")
        # vis.plot_ground_plane_3D(config, ground_plane, proto_segs_arr, proto_segs, "10_GroundPlane_3D")
        # vis.plot_labelled_points_2D(config, point_cloud, point_labels, ground_plane, "11_LabelledPoints_2D")
        # vis.plot_labelled_points_3D(
        #   config, point_cloud, point_labels, ground_plane, "12_LabelledPoints_3D"
        # )
        # vis.plot_object_points_2D(config, object_points, "13_Object_Points_2D")
        # vis.plot_object_centers_2D(config, object_points, object_centers, objects, object_line_dists, "14_Objects_2D")
        # vis.plot_reconstructed_objects_2D(config, reconstructed_objects, reconstructed_centers, "14_Reconstructed_Objects")
        # vis2.plot_cones_2D(config, point_cloud, point_labels, cone_centers, cone_points, "15_Cones")
        # vis2.plot_cones_3D(config, point_cloud[point_norms <= 100], point_labels[point_norms <= 100], cones, "16_Cones_3D")
        vis2.plot_detailed_2D(config, point_cloud, segments, bins, ground_plane[np.unique(segments)], point_labels, reconstructed_objects, reconstructed_centers, cone_centers, cone_points, duration, "15_Cones")

        # reintro structured array for lidar colouring

        if config.show_figures:
            plt.show()

    return cone_centers
