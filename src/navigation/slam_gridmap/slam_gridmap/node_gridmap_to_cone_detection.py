import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


# node class object that gets created
def cone_msg(x: float, y: float) -> Cone:
    """
    Create a Cone message from x and y coordinates.

    Args:
        x (float): The x coordinate of the cone (LiDAR sensor is origin).
        y (float): The y coordinate of the cone (LiDAR sensor is origin).

    Returns:
        Cone: The cone message.
    """
    location: Point = Point(x=x / 10, y=y / 10, z=0.0)

    # SLAM does not identify cone colour
    return Cone(location=location, color=Cone.UNKNOWN)


class SLAMDetectorNode(Node):
    def __init__(self):
        super().__init__("gridmap_to_cone_detection_node")

        # any subscribers to messages on topics go here
        self.create_subscription(OccupancyGrid, "/slam/occupancy_grid", self.grid_callback, 10)

        # any publishers for messages to topics go here
        self.detection_publisher = self.create_publisher(ConeDetectionStamped, "/slam/cone_detection", 1)

        self.get_logger().info("---Gridmap to Cone Detection node initialised---")

    # function that is called each time the subscriber reads a new message on the topic
    def grid_callback(self, map_message: OccupancyGrid):
        gridmap_info = map_message.info
        map_width = gridmap_info.width
        map_height = gridmap_info.height

        gridmap_origin_x = gridmap_info.origin.position.x
        gridmap_origin_y = gridmap_info.origin.position.y

        gridmap_data = map_message.data
        map_2d = np.array(gridmap_data).reshape((map_height, map_width))

        # point_coords = []
        # for row in range(map_height):
        #     for col in range(map_width):
        #         cell = map_2d[row, col]
        #         if cell == 100:
        #             point_coords.append((col + gridmap_origin_x * 10, (row + gridmap_origin_y * 10)))

        # convert row, col checks to a numpy operation (faster)
        cone_indices = np.where(map_2d == 100) # Get the indices of the cones
        point_coords = np.column_stack((cone_indices[1] + gridmap_origin_x * 10, cone_indices[0] + gridmap_origin_y * 10))

        # check if there are any cones
        if len(point_coords) == 0:
            return

        # check if its actually a 1D array (only one cone), if so, make it 2D
        if point_coords.ndim == 1:
            point_coords = np.expand_dims(point_coords, axis=0)

        # Cluster object points
        if len(point_coords) < 20:  # if we're close to start, we havent seen many cones
            epsilon = 3
            min_points = 1
        else:
            epsilon = 7
            min_points = 3

        clustering = DBSCAN(eps=epsilon, min_samples=min_points).fit(point_coords)
        labels = clustering.labels_

        unq_labels = np.unique(labels)[1:]  # Noise cluster -1 (np.unique sorts)

        objects = np.empty(unq_labels.size, dtype=object)
        object_centers = np.empty((unq_labels.size, 2))
        for idx, label in enumerate(unq_labels):
            objects[idx] = point_coords[np.where(labels == label)]
            if objects[idx].size == 2: # If only one point in cluster
                object_centers[idx] = objects[idx][0]
            else:  # multiple points, get the mean
                object_centers[idx] = np.mean(np.column_stack((objects[idx][0], objects[idx][1])), axis=1)

        if len(object_centers) == 0:
            return

        # Convert cone locations to ConeDetection messages and publish
        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in object_centers]
        detection_msg = ConeDetectionStamped(header=map_message.header, cones=detected_cones)
        self.detection_publisher.publish(detection_msg)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = SLAMDetectorNode()
    rclpy.spin(node)  # starts node operations
    node.destroy_node()
    rclpy.shutdown()
