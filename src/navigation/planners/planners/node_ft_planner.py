from math import atan2, cos, pi, sin, sqrt
import time
import cv2

import numpy as np
import scipy.interpolate as scipy_interpolate
from transforms3d.euler import euler2quat
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from driverless_msgs.msg import Cone, ConeDetectionStamped, State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path

from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, midpoint, wrap_to_pi
from driverless_common.draw import draw_map

from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes

from typing import List, Tuple

# for colour gradient based on intensity
SEARCH_RANGE = 6
SEARCH_ANGLE = pi / 3
SEARCH_RANGE_LENIANCE = 0.3
SEARCH_ANGLE_MIN_LENIANCE = 15 * pi / 180  # 10 degrees

def approximate_b_spline_path(x: list, y: list, n_path_points: int, degree=3, s=0) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximate points with a B-Spline path
    * param x: x position list of approximated points
    * param y: y position list of approximated points
    * param n_path_points: number of path points
    * param degree: (Optional) B Spline curve degree
    * return: x and y position list of the result path
    """

    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]

    spl_i_x = scipy_interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = scipy_interpolate.UnivariateSpline(distances, y, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)

    spline_x = spl_i_x(sampled)
    spline_y = spl_i_y(sampled)

    # curvature
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0 / 3.0)

    return spline_x, spline_y

def make_path_msg(points) -> Path:
    spline_len = len(points)
    poses: List[PoseStamped] = []  # target spline poses
    for i in range(spline_len):
        # get angle between current point and next point
        if i < spline_len - 1:
            th_change = angle(points[i], points[i + 1])
        elif i == spline_len - 1:
            th_change = angle(points[i], points[0])
        # keep between 360
        if th_change > pi:
            th_change = th_change - 2 * pi
        elif th_change < -pi:
            th_change = th_change + 2 * pi

        pose = PoseStamped()
        pose.header.frame_id = "track"
        pose.pose.position.x = points[i][0]
        pose.pose.position.y = points[i][1]
        pose.pose.position.z = 0.0
        quat = euler2quat(0.0, 0.0, th_change)
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        poses.append(pose)

    # Add the first path point to the end of the list to complete the loop
    # poses.append(poses[0])
    path_msg = Path()
    path_msg.header.frame_id = "track"
    path_msg.poses = poses
    return path_msg

def get_occupancy_grid(blue_points, yellow_points, header):
    # turn spline boundary points into an occupancy grid map
    map = OccupancyGrid()
    map.header = header
    map.info.resolution = 0.1
    map.info.map_load_time = header.stamp

    # we have blue and yellow interpolated points.
    # turn them into an occupancy grid map with the specified resolution
    # set origin which has an origin at 0,0 in world coords
    # find the min and max x and y values
    bounds = blue_points + yellow_points
    bounds = np.array(bounds)
    bounds = bounds / map.info.resolution
    bounds = bounds.astype(int)

    # find the min and max x and y values
    min_x = int(np.min(bounds[:, 0]))
    max_x = int(np.max(bounds[:, 0]))
    min_y = int(np.min(bounds[:, 1]))
    max_y = int(np.max(bounds[:, 1]))

    # set map size
    if min_x > 0:
        min_x = 0

    # set origin to be the 0,0 point
    map.info.origin.position.x = min_x * map.info.resolution
    map.info.origin.position.y = min_y * map.info.resolution

    map.info.width = max_x - min_x + 1
    map.info.height = max_y - min_y + 1

    # shift all points to be relative to the origin
    bounds = bounds - [min_x, min_y]

    # turn points into a 2D np array
    grid = np.zeros((map.info.height, map.info.width), dtype=np.int8)
    for x, y in bounds:
        grid[y, x] = 100
    map.data = grid.ravel().tolist()
    return map

class FaSTTUBeBoundaryExtractor(Node):
    following = False
    current_track = None
    spline_const = 10  # number of points per cone

    path_planner = PathPlanner(MissionTypes.trackdrive)

    def __init__(self):
        super().__init__("ordered_map_spline_node")

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.map_callback, QOS_LATEST)
        self.create_timer(1 / 20, self.planning_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.blue_bound_pub = self.create_publisher(Path, "/planning/blue_bounds", 1)
        self.yellow_bound_pub = self.create_publisher(Path, "/planning/yellow_bounds", 1)
        self.planned_path_pub = self.create_publisher(Path, "/planning/midline_path", 1)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.map_meta_pub = self.create_publisher(MapMetaData, "/map_metadata", map_qos)

        self.declare_parameter("start_following", False)
        if self.get_parameter("start_following").value:
            self.following = True
            self.get_logger().warn("---DEBUG MODE ENABLED---")

        self.get_logger().info("---Ordered path planner node initalised---")

    def map_callback(self, track_msg: ConeDetectionStamped):
        self.get_logger().debug("Received map")
        self.current_track = track_msg

    def planning_callback(self):
        # skip if we haven't completed a lap yet
        if self.current_track is None or len(self.current_track.cones) == 0:
            self.get_logger().warn("No track data received", throttle_duration_sec=1)
            return

        try:
            # TODO: parameterise these frames?
            map_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e), throttle_duration_sec=1)
            return

        car_position = np.array([map_to_base.transform.translation.x, map_to_base.transform.translation.y])
        car_direction = quat2euler(
            [
                map_to_base.transform.rotation.w,
                map_to_base.transform.rotation.x,
                map_to_base.transform.rotation.y,
                map_to_base.transform.rotation.z,
            ]
        )[2]

        # split track into conetypes
        unknown_cones = np.array([])
        yellow_cones = np.array([])
        blue_cones = np.array([])
        orange_small_cones = np.array([])
        orange_big_cones = np.array([])
        for cone in self.current_track.cones:
            # stack the cones into arrays Mx2
            cone_pos = np.array([cone.location.x, cone.location.y])
            if cone.color == Cone.UNKNOWN:
                unknown_cones = np.vstack([unknown_cones, cone_pos]) if unknown_cones.size else cone_pos
            elif cone.color == Cone.YELLOW:
                yellow_cones = np.vstack([yellow_cones, cone_pos]) if yellow_cones.size else cone_pos
            elif cone.color == Cone.BLUE:
                blue_cones = np.vstack([blue_cones, cone_pos]) if blue_cones.size else cone_pos
            elif cone.color == Cone.ORANGE_SMALL:
                orange_small_cones = np.vstack([orange_small_cones, cone_pos]) if orange_small_cones.size else cone_pos
            elif cone.color == Cone.ORANGE_BIG:
                orange_big_cones = np.vstack([orange_big_cones, cone_pos]) if orange_big_cones.size else cone_pos

        global_cones = [unknown_cones, yellow_cones, blue_cones, orange_small_cones, orange_big_cones]

        path, ordered_blues, ordered_yellows, virt_blues, virt_yellows, _, _ = self.path_planner.calculate_path_in_global_frame(
            global_cones, car_position, car_direction, return_intermediate_results=True
        )

        # use_virt = True
        # if use_virt:
        #     ordered_blues = virt_blues
        #     ordered_yellows = virt_yellows
        
        if len(ordered_blues) == 0 or len(ordered_yellows) == 0:
            self.get_logger().warn("No cones found", throttle_duration_sec=1)
            return

        # Spline smoothing
        # make number of pts based on length of path
        spline_len = self.spline_const * len(ordered_blues)

        # specify degree of spline if less than 3 cones
        blue_degree = len(ordered_blues) - 1 if len(ordered_blues) <= 3 else 3
        yellow_degree = len(ordered_yellows) - 1 if len(ordered_yellows) <= 3 else 3

        yx, yy = approximate_b_spline_path(
            [cone[0] for cone in ordered_yellows],
            [cone[1] for cone in ordered_yellows],
            spline_len, yellow_degree, 0.01,
        )
        bx, by = approximate_b_spline_path(
            [cone[0] for cone in ordered_blues], 
            [cone[1] for cone in ordered_blues], 
            spline_len, blue_degree, 0.01
        )
        # turn individual x,y lists into points lists
        blue_points = []
        yellow_points = []
        mid_points = []
        for i in range(spline_len):
            blue_points.append([bx[i], by[i]])
            yellow_points.append([yx[i], yy[i]])
            mid_points.append(midpoint([yx[i], yy[i]], [bx[i], by[i]]))

        # publish bounds
        blue_bound_msg = make_path_msg(blue_points)
        self.blue_bound_pub.publish(blue_bound_msg)

        yellow_bound_msg = make_path_msg(yellow_points)
        self.yellow_bound_pub.publish(yellow_bound_msg)

        # publish midpoints
        mid_bound_msg = make_path_msg(mid_points)
        # self.planned_path_pub.publish(mid_bound_msg)
        self.planned_path_pub.publish(make_path_msg(path[:, 1:3]))

        ## Create occupancy grid of interpolated bounds
        map = get_occupancy_grid(blue_points, yellow_points, self.current_track.header)
        self.current_map = map
        self.map_pub.publish(self.current_map)
        self.map_meta_pub.publish(self.current_map.info)

def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = FaSTTUBeBoundaryExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
