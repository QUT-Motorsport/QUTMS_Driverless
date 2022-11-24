import matplotlib.pyplot as plt
import numpy as np

from . import ground_plane_estimator as gpe
from . import point_cloud_processor as pcp
from . import visualiser as vis
from .. import constants as const
from ..utils import Config  # For typing


def locate_cones(config, point_cloud):
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
    config.logger.info(f"DONE: Segments and Bins")

    proto_segs_arr, proto_segs, seg_bin_z_ind = pcp.get_prototype_points(segments, bins, point_norms, point_cloud["z"])
    config.logger.info(f"DONE: Prototype Points")

    # Multiprocessing Ground Plane Mapping [m b start(x, y) end(x, y) bin]
    ground_plane = gpe.get_ground_plane_mp(config, proto_segs_arr, proto_segs)
    config.logger.info(f"DONE: Ground Plane Mapped")

    # Create visualisations
    if config.create_figures:
        vis.plot_point_cloud_2D(config, point_cloud, "01_PointCloud_2D")
        vis.plot_segments_2D(config, point_cloud, segments, "03_PointCloudSegments_2D")
        vis.plot_bins_2D(config, point_cloud, bins, "05_PointCloudBins_2D")
        # vis.plot_segments_3D(config, point_cloud, segments, name)
        # vis.plot_bins_3D(point_cloud, bins, working_dir, timestamp, animate_figures)

        if config.show_figures:
            plt.show()
