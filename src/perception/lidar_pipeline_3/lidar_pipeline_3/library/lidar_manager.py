import matplotlib.pyplot as plt
import numpy as np

from . import visualiser as vis
from .. import constants as const
from ..utils import Config  # For typing


def locate_cones(config, point_cloud):
    config.logger.info(f"Point Cloud received with {point_cloud.shape[0]} points")

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

    # Number of points in point_cloud
    config.logger.info(f"{point_cloud.shape[0]} points remain after filtering point cloud")

    if config.create_figures:
        vis.plot_point_cloud_2D(config, point_cloud, "01_PointCloud_2D")

    if config.show_figures:
        plt.show()
