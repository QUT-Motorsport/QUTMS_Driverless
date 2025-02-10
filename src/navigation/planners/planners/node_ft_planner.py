# Path planner documentation below:
# ft-fsd-path-planning - Simple Application (https://github.com/QUT-Motorsport/ft-fsd-path-planning/blob/main/fsd_path_planning/demo/simple_application.ipynb)

from math import pi
import time

import diagnostic_updater
from fsd_path_planning import ConeTypes, MissionTypes, PathPlanner
import numpy as np
import scipy.interpolate as scipy_interpolate
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import euler2quat, quat2euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from std_msgs.msg import Bool

from driverless_common.common import QOS_LATEST, angle, midpoint

from typing import List, Tuple


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


def make_path_msg(points, frame) -> Path:
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
        pose.header.frame_id = frame
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
    path_msg.header.frame_id = frame
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
    current_track = None
    spline_const = 10  # number of points per cone
    initial_planning = True

    def __init__(self):
        super().__init__("ft_planner_node")

        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_footprint")
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "slam/cone_detection", self.detection_callback, QOS_LATEST)
        self.create_timer(1 / 5, self.planning_callback)  # change back to 1/10 (1Hz)

        # Create subscriber for cars pose, when using the navigation simulator
        self.create_subscription(PoseStamped, "car/pose", self.car_pose_sim, 10)

        # Initialise nav_sim as false (will be set to true if dummy track publisher sets to true)
        self.create_subscription(Bool, "nav_sim", self.nav_sim_callback, 10)
        self.nav_sim = False  # intiialise as false

        # Initialise car_position & car_direction
        self.car_position = [0.0]
        self.car_direction = [0.0]

        # Initialise Initial_planning as None
        self.initial_planning = None  # Change???

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path Planning Publishers
        self.blue_bound_pub = self.create_publisher(Path, "/planning/blue_bounds", 1)
        self.yellow_bound_pub = self.create_publisher(Path, "/planning/yellow_bounds", 1)
        self.planned_path_pub = self.create_publisher(Path, "/planning/midline_path", 1)

        # map_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        # )
        # self.map_pub = self.create_publisher(OccupancyGrid, "/planning/boundary_grid", map_qos)
        # self.map_meta_pub = self.create_publisher(MapMetaData, "/planning/boundary_grid_metadata", map_qos)

        # Create PathPlanner object
        self.path_planner = PathPlanner(**self.get_planner_cfg())

        self.diagnostic_updater = diagnostic_updater.Updater(self, 1)
        self.diagnostic_updater.setHardwareID("Planner")
        self.diagnostic_pub = diagnostic_updater.TopicDiagnostic(
            "/planning/midline_path",
            self.diagnostic_updater,
            diagnostic_updater.FrequencyStatusParam({"min": 5, "max": 10}, 1, 10),
            diagnostic_updater.TimeStampStatusParam(),
        )

        self.get_logger().info("---Planner node initalised---")

    def get_planner_cfg(self):
        self.declare_parameter("mission", MissionTypes.trackdrive)

        # cone sorting
        self.declare_parameter("max_n_neighbors", 5)
        self.declare_parameter("max_dist", 6.5)
        self.declare_parameter("max_dist_to_first", 6.0)
        self.declare_parameter("max_length", 12)
        self.declare_parameter("threshold_directional_angle", 30)
        self.declare_parameter("threshold_absolute_angle", 55)
        self.declare_parameter("use_unknown_cones", True)

        cone_sorting_kwargs = {
            "max_n_neighbors": self.get_parameter("max_n_neighbors").value,
            "max_dist": self.get_parameter("max_dist").value,
            "max_dist_to_first": self.get_parameter("max_dist_to_first").value,
            "max_length": self.get_parameter("max_length").value,
            "threshold_directional_angle": np.deg2rad(self.get_parameter("threshold_directional_angle").value),
            "threshold_absolute_angle": np.deg2rad(self.get_parameter("threshold_absolute_angle").value),
            "use_unknown_cones": self.get_parameter("use_unknown_cones").value,
        }

        # cone fitting
        self.declare_parameter("smoothing", 0.2)
        self.declare_parameter("predict_every", 0.1)
        self.declare_parameter("max_deg", 3)

        cone_fitting_kwargs = {
            "smoothing": self.get_parameter("smoothing").value,
            "predict_every": self.get_parameter("predict_every").value,
            "max_deg": self.get_parameter("max_deg").value,
        }

        # path calculation
        self.declare_parameter("maximal_distance_for_valid_path", 5)
        self.declare_parameter("mpc_path_length", 20)
        self.declare_parameter("mpc_prediction_horizon", 40)

        path_calculation_kwargs = {
            "maximal_distance_for_valid_path": self.get_parameter("maximal_distance_for_valid_path").value,
            "mpc_path_length": self.get_parameter("mpc_path_length").value,
            "mpc_prediction_horizon": self.get_parameter("mpc_prediction_horizon").value,
        }

        # cone matching
        self.declare_parameter("min_track_width", 3.5)
        self.declare_parameter("max_search_range", 5)
        self.declare_parameter("max_search_angle", 50)
        self.declare_parameter("matches_should_be_monotonic", True)

        cone_matching_kwargs = {
            "min_track_width": self.get_parameter("min_track_width").value,
            "max_search_range": self.get_parameter("max_search_range").value,
            "max_search_angle": np.deg2rad(self.get_parameter("max_search_angle").value),
            "matches_should_be_monotonic": self.get_parameter("matches_should_be_monotonic").value,
        }

        return {
            "mission": self.get_parameter("mission").value,
            "cone_sorting_kwargs": cone_sorting_kwargs,
            "cone_fitting_kwargs": cone_fitting_kwargs,
            "path_calculation_kwargs": path_calculation_kwargs,
            "cone_matching_kwargs": cone_matching_kwargs,
        }

    # Navigation simulation boolean callback (to announce if using navigation simulation)
    def nav_sim_callback(self, msg: Bool):
        self.nav_sim = msg.data

    # Cone detection callback, save detected cones as self.current_track variable/list
    def detection_callback(self, track_msg: ConeDetectionStamped):
        self.get_logger().info("Received detections")
        if not self.initial_planning:
            self.current_track = track_msg
            return
        if self.nav_sim:
            self.current_track = track_msg
            return

    # Car pose callback (for navigation simulation)
    def car_pose_sim(self, car_pose: PoseStamped):
        self.car_position = np.array([car_pose.pose.position.x, car_pose.pose.position.y])
        # Extract the car's orientation (yaw angle)
        self.car_direction = quat2euler(
            [
                car_pose.pose.orientation.w,
                car_pose.pose.orientation.x,
                car_pose.pose.orientation.y,
                car_pose.pose.orientation.z,
            ]
        )[2]

    def planning_callback(self):

        # if not running nav_sim get car position and car_direction from transforms
        if not self.nav_sim:
            try:
                map_to_base = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            except TransformException as e:
                self.get_logger().warn("Transform exception: " + str(e), throttle_duration_sec=1)
                return
            self.car_position = np.array([map_to_base.transform.translation.x, map_to_base.transform.translation.y])
            car_direction = quat2euler(
                [
                    map_to_base.transform.rotation.w,
                    map_to_base.transform.rotation.x,
                    map_to_base.transform.rotation.y,
                    map_to_base.transform.rotation.z,
                ]
            )[2]

        # If car position is close to start & initial planning is true (initial planning is true for setup lap)
        # & not running the navigation simulation (nav_sim), set points to pre-track list
        if self.car_position[0] < 0.5 and self.initial_planning and not self.nav_sim:
            # make a cone detection stamped msg from pre-track list
            points = [
                [12.062088012695312, 1.751],
                [15.124588012695314, 1.753],
                [3.812088012695313, 1.754],
                [18.499588012695312, 1.755],
                [9.624588012695312, 1.756],
                [12.499588012695314, -1.757],
                [14.999588012695314, -1.758],
                [19.124588012695316, -1.759],
                [4.187088012695313, -1.751],
                [9.374588012695312, -1.752],
                [6.187088012695313, 1.753],
                [6.3745880126953125, -1.754],
                [5.6245880126953125, 1.755],
                [5.9370880126953125, -1.756],
            ]
            # Set list of points equal to cone locations (spoof cone locations with pre-track list)
            self.current_track = ConeDetectionStamped()
            for point in points:
                cone = Cone()
                cone.location.x = point[0]
                cone.location.y = point[1]
                cone.color = Cone.UNKNOWN
                self.current_track.cones.append(cone)
        else:  # initial planning false when car is moving &
            self.initial_planning = False

        # Redundancy in case self.current_track variable is None
        if self.current_track is None or len(self.current_track.cones) == 0:
            self.get_logger().warn("No track data received, Path Planning Aborted!", throttle_duration_sec=1)
            return

        # Split current_track (recieved from cone/points detection callback) into cone types, organised by color (2D numpy array)
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
                self.get_logger().info(f"Found Unknown Cones:\n{unknown_cones}")
            elif cone.color == Cone.YELLOW:
                yellow_cones = np.vstack([yellow_cones, cone_pos]) if yellow_cones.size else cone_pos
                self.get_logger().info(f"Found Yellow Cones:\n{yellow_cones}")
            elif cone.color == Cone.BLUE:
                blue_cones = np.vstack([blue_cones, cone_pos]) if blue_cones.size else cone_pos
                self.get_logger().info(f"Found Blue Cones:\n{blue_cones}")
            elif cone.color == Cone.ORANGE_SMALL:
                orange_small_cones = np.vstack([orange_small_cones, cone_pos]) if orange_small_cones.size else cone_pos
                self.get_logger().info(f"Found Orange Small Cones:\n{orange_small_cones}")
            elif cone.color == Cone.ORANGE_BIG:
                orange_big_cones = np.vstack([orange_big_cones, cone_pos]) if orange_big_cones.size else cone_pos
                self.get_logger().info(f"Found Orange Big Cones:\n{orange_big_cones}")

        global_cones = [unknown_cones, yellow_cones, blue_cones, orange_small_cones, orange_big_cones]
        # self.get_logger().info(f"global_cones: {global_cones}")

        try:
            (
                path,
                ordered_blues,
                ordered_yellows,
                virt_blues,
                virt_yellows,
                _,
                _,
            ) = self.path_planner.calculate_path_in_global_frame(
                global_cones, self.car_position, self.car_direction, return_intermediate_results=True
            )
        except Exception as e:
            self.get_logger().warn("Cant plan, error" + str(e), throttle_duration_sec=1)
            return

        # self.get_logger().info(f"Found Path!:\n {path}")

        self.get_logger().info(f"ordered_blues: {ordered_blues}\n virt_blues: {virt_blues}")
        self.get_logger().info(f"ordered_yellows: {ordered_yellows}\n virt_yellows: {virt_yellows}")

        use_virt = True
        if use_virt:
            ordered_blues = virt_blues
            ordered_yellows = virt_yellows

        # If not planning properly, return out of path_planning function **need to keep this uncommented, to validate planning working properly
        # NOTE: ft-fsd-path-planner documentation states that only need one side of cones to path plan,
        # NOTE: could use "and" condition here instead of "or"
        # if len(ordered_blues) == 0 or len(ordered_yellows) == 0:
        #    self.get_logger().warn("No cones found!", throttle_duration_sec=1)
        #    return
        # else:
        #    self.get_logger().info(f"Found {len(ordered_blues)} blue cones and {len(ordered_yellows)} yellow cones")

        # Spline smoothing
        # make number of pts based on length of path
        spline_len = self.spline_const * len(ordered_blues)

        # specify degree of spline if less than 3 cones
        blue_degree = len(ordered_blues) - 1 if len(ordered_blues) <= 3 else 3
        yellow_degree = len(ordered_yellows) - 1 if len(ordered_yellows) <= 3 else 3

        try:
            yx, yy = approximate_b_spline_path(
                [cone[0] for cone in ordered_yellows],
                [cone[1] for cone in ordered_yellows],
                spline_len,
                yellow_degree,
                0.01,
            )
            bx, by = approximate_b_spline_path(
                [cone[0] for cone in ordered_blues], [cone[1] for cone in ordered_blues], spline_len, blue_degree, 0.01
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
            blue_bound_msg = make_path_msg(blue_points, self.map_frame)
            self.get_logger().info("Publish blue line")
            self.blue_bound_pub.publish(blue_bound_msg)

            yellow_bound_msg = make_path_msg(yellow_points, self.map_frame)
            self.get_logger().info("Publish yellow line")
            self.yellow_bound_pub.publish(yellow_bound_msg)

        except Exception as e:
            self.get_logger().warn("Cant calculate bounds, error" + str(e), throttle_duration_sec=1)

        # publish midpoints
        midline_msg = make_path_msg(path[:, 1:3], self.map_frame)
        self.get_logger().info(f"Published Mid-Point Path as msg!")
        self.planned_path_pub.publish(midline_msg)

        ## Create occupancy grid of interpolated bounds
        # map = get_occupancy_grid(blue_points, yellow_points, self.current_track.header)
        # self.current_map = map
        # self.map_pub.publish(self.current_map)
        # self.map_meta_pub.publish(self.current_map.info)

        # convert current time to nanosecs
        stamp_float = time.time()
        self.diagnostic_pub.tick(stamp_float)


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = FaSTTUBeBoundaryExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
