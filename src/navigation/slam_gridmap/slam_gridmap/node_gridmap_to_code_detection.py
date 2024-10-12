# import ros2 libraries
# other python modules
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

# OccupancyGridUpdate


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

        # any extra initialisation of class variables go here

    # function that is called each time the subscriber reads a new message on the topic
    def grid_callback(self, map_message: OccupancyGrid):
        print("GRID RECIEVED")
        # do stuff
        gridmap_info = map_message.info
        map_width = gridmap_info.width
        map_height = gridmap_info.height

        gridmap_origin_x = gridmap_info.origin.position.x
        gridmap_origin_y = gridmap_info.origin.position.y

        gridmap_data = map_message.data
        map_2d = np.array(gridmap_data).reshape((map_height, map_width))

        point_coords = []
        for row in range(map_height):
            for col in range(map_width):
                cell = map_2d[row, col]
                if cell == 100:
                    point_coords.append((col + gridmap_origin_x * 10, (row + gridmap_origin_y * 10)))
        point_coords = np.array(point_coords)

        # Cluster object points
        self.epsilon = 7
        self.min_points = 4

        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_points).fit(point_coords)
        labels = clustering.labels_

        unq_labels = np.unique(labels)[1:]  # Noise cluster -1 (np.unique sorts)

        objects = np.empty(unq_labels.size, dtype=object)
        object_centers = np.empty((unq_labels.size, 2))
        for idx, label in enumerate(unq_labels):
            objects[idx] = point_coords[np.where(labels == label)]
            object_centers[idx] = np.mean(np.column_stack((objects[idx][0], objects[idx][1])), axis=1)

        if len(object_centers) == 0:
            return

        # Convert cone locations to ConeDetection messages and publish
        detected_cones: list = [cone_msg(cone[0], cone[1]) for cone in object_centers]
        detection_msg = ConeDetectionStamped(header=map_message.header, cones=detected_cones)
        self.detection_publisher.publish(detection_msg)

        print(detection_msg)
        print(gridmap_info)

        # do stuffs
        # publishing_variable = Message_Type2()
        # publishing_variable.property = another_variable
        # self.publisher_name.publish(publishing_variable)
        print("MAP CREATED")


# main run when script is started in the terminal
def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = SLAMDetectorNode()
    rclpy.spin(node)  # starts node operations
    node.destroy_node()
    rclpy.shutdown()
