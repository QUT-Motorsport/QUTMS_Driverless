from math import cos, pi, sin, sqrt

import numpy as np
import scipy.interpolate as scipy_interpolate
from transforms3d.euler import euler2quat

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped, PathPoint, State
from driverless_msgs.msg import PathStamped as QUTMSPathStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from driverless_common.common import QOS_LATEST, angle, dist, midpoint

from typing import List, Tuple

# for colour gradient based on intensity
MAX_ANGLE = 0.15


def approximate_b_spline_path(x: list, y: list, n_path_points: int, degree: int = 3) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py \n
    Approximate points with a B-Spline path
    * param x: x position list of approximated points
    * param y: y position list of approximated points
    * param n_path_points: number of path points
    * param degree: (Optional) B Spline curve degree
    * return: x and y position list of the result path
    """

    t: int = range(len(x))
    # interpolate for the length of the input cone list
    x_list = list(scipy_interpolate.splrep(t, x, k=degree))
    y_list = list(scipy_interpolate.splrep(t, y, k=degree))

    # add 4 'zero' components to align matrices
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    spline_x = scipy_interpolate.splev(ipl_t, x_list)
    spline_y = scipy_interpolate.splev(ipl_t, y_list)

    return spline_x, spline_y


def sort_cones(cones, start_index=None, end_index=None):
    """
    This is a function that calculates the nearest-neighbor path between a start and end index for a list of cones.
    The cones are represented as 2D points in the form of (x, y).

    The function takes 3 parameters:
    cones: a list of the cones to be sorted.
    start_index: the index of the starting cone.
    end_index: the index of the ending cone.

    The function uses a matrix "mat" to store the Euclidean distances between each pair of cones, and uses these
    distances to calculate the nearest-neighbor path between the start and end indices.

    The path is found by starting at the start_index, finding the index of the closest cone that hasn't already
    been visited, and adding that index to the "order" list. This process is repeated until the end_index is reached.

    The final path is returned as the "order" list, which contains the indices of the cones in the order they should
    be visited.
    """
    if start_index is None:
        start_index = 0
    if end_index is None:
        end_index = len(cones) - 1

    cone_count = len(cones)
    mat = [[0] * cone_count for i in range(cone_count)]
    for i, a in enumerate(cones):
        for j, b in enumerate(cones):
            if i == j:
                continue

            # Ensures we sort in the direction that the car is pointing
            # If the current cone is the far orange cone, we ignore all cones where the x coord is less than the
            # selected cone's x coord.
            # I can't image a track that this would cause problems on.
            if i == start_index and b[0] < a[0]:
                continue

            # Prevent unnecessary square root calculations. We don't need to
            # calculate distance if the distance is going to be greater than 5m (sqrt(25)).
            # So don't square root, or store the distance in the mat.
            # Note: Doubled the accepted distance to account for human error and camera noise.
            diff = [a[0] - b[0], a[1] - b[1]]
            dist = diff[0] * diff[0] + diff[1] * diff[1]
            if dist <= 50:
                mat[i][j] = sqrt(dist)

    order = [start_index]
    # Loop for each cone that needs to be ordered.
    # -2 accounts for the start and end indices that are manually handled.
    for unused in range(len(mat) - 2):
        min_index = -1
        min_value = 10000
        # Get the latest ordered cone's index from the 'order' list, and loop through that cone's row in 'mat'.
        # That cone's row in 'mat' contains the distance between the cone, and every other cone on the track.
        # We can then easily find the closest unused cone and append it to the 'order' list.
        for i, dist in enumerate(mat[order[-1]]):
            if dist >= min_value or dist == 0 or i == end_index or i in order:
                continue

            min_index = i
            min_value = dist
        order.append(min_index)
    order.append(end_index)

    # Return the ordered cones.
    return [cones[order[i]] for i in range(len(order))]


def new_coordinates(x, y, angle_rads, distance):
    """
    Calculate new coordinates after moving from an initial point in a specified direction and distance.

    * param x: float, x-coordinate of the initial point
    * param y: float, y-coordinate of the initial point
    * param angle_rads: float, angle in radians to move from initial point
    * param distance: float, distance to move from the initial point
    * return: tuple (x_new, y_new, 0), new coordinates after moving
    """
    x_new = x + distance * cos(angle_rads)
    y_new = y + distance * sin(angle_rads)
    return (x_new, y_new, 0)


def parse_orange_cones(node_logger, orange_cones: List[List[float]]) -> List[List[float]]:
    """
    Breaks the big orange starting cones into their position relative to the other blue/yellow cones.
    Returns format close_blue, far_blue, close_yellow, far_yellow.
    """
    if len(orange_cones) < 4:
        node_logger.fatal("parse_orange_cones called with less than 4 visible cones. Requires 4 cones.")
        return []

    blue_cones = [cone for cone in orange_cones if cone[1] > 0]
    yellow_cones = [cone for cone in orange_cones if cone[1] < 0]

    if blue_cones[0][0] > blue_cones[1][0]:
        blue_cones[0], blue_cones[1] = blue_cones[1], blue_cones[0]

    if yellow_cones[0][0] > yellow_cones[1][0]:
        yellow_cones[0], yellow_cones[1] = yellow_cones[1], yellow_cones[0]

    return [blue_cones[0], blue_cones[1], yellow_cones[0], yellow_cones[1]]


class OrderedMapSpline(Node):
    spline_const = 10  # number of points per cone
    segment = int(spline_const * 0.1)  # percentage of PPC
    planning = False
    current_track = None
    interp_cone_num = 3  # number of points interpolated between each pair of cones
    final_path_published = False

    def __init__(self):
        super().__init__("ordered_map_spline_node")
        # sub to track for all cone locations relative to car start point
        self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.map_callback, QOS_LATEST)
        self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)
        self.create_timer(0.05, self.planning_callback)

        # publishers
        self.qutms_path_pub = self.create_publisher(QUTMSPathStamped, "/planner/path", 1)
        self.planned_path_pub = self.create_publisher(Path, "/planner/global_path", 1)
        self.interp_cones_pub = self.create_publisher(ConeDetectionStamped, "/planner/interpolated_map", 1)

        self.get_logger().info("---Ordered path planner node initialised---")

    def map_callback(self, track_msg: ConeDetectionStamped):
        if self.final_path_published:
            return

        self.get_logger().info("Received map", once=True)
        self.current_track = track_msg

    def planning_callback(self):
        if self.current_track is None:
            return

        # skip if we haven't completed a lap yet
        self.get_logger().debug("Planning")

        # extract data out of message
        cones = self.current_track.cones

        yellows: List[List[float]] = []
        blues: List[List[float]] = []
        oranges: List[List[float]] = []

        for cone in cones:
            if cone.color == Cone.YELLOW:
                yellows.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                blues.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.ORANGE_BIG:
                oranges.append([cone.location.x, cone.location.y])

        # place orange cones on their respective sides of the track
        parsed_orange_cones = parse_orange_cones(self.get_logger(), oranges)
        if len(parsed_orange_cones) == 0:
            return
        blues.insert(0, parsed_orange_cones[1])
        blues.append(parsed_orange_cones[0])
        yellows.insert(0, parsed_orange_cones[3])
        yellows.append(parsed_orange_cones[2])

        # Sort the blue and yellow cones starting from the far orange cone, and ending at the close orange cone.
        ordered_blues = sort_cones(blues)
        ordered_yellows = sort_cones(yellows)

        ## Spline smoothing
        # make number of pts based on length of path
        spline_len = self.spline_const * len(ordered_blues)
        # retrieves spline lists (x,y)
        yx, yy = approximate_b_spline_path(
            [cone[0] for cone in ordered_yellows], [cone[1] for cone in ordered_yellows], spline_len
        )
        bx, by = approximate_b_spline_path(
            [cone[0] for cone in ordered_blues], [cone[1] for cone in ordered_blues], spline_len
        )

        tx: List[float] = []  # target spline x coords
        ty: List[float] = []  # target spline y coords
        th: List[float] = []  # target spline angles
        poses: List[PoseStamped] = []  # target spline poses
        # find midpoint between splines at each point to make target path
        for i in range(spline_len):
            mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
            tx.append(mid_x)
            ty.append(mid_y)
            # for old QUTMS path curve intensity calcs
            th.append(angle([bx[i], by[i]], [yx[i], yy[i]]))  # angle of tangent at midpoint

        for i in range(spline_len):
            # get angle between current point and next point
            if i < spline_len - 1:
                th_change = angle([tx[i], ty[i]], [tx[i + 1], ty[i + 1]])
            elif i == spline_len - 1:
                th_change = angle([tx[i], ty[i]], [tx[0], ty[0]])
            # keep between 360
            if th_change > pi:
                th_change = th_change - 2 * pi
            elif th_change < -pi:
                th_change = th_change + 2 * pi

            pose = PoseStamped()
            pose.pose.position.x = tx[i]
            pose.pose.position.y = ty[i]
            quat = euler2quat(0.0, 0.0, th_change)
            pose.pose.orientation.w = quat[0]
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            poses.append(pose)

        # Add the first path point to the end of the list to complete the loop
        poses.append(poses[0])
        spline_path_msg = Path()
        spline_path_msg.header.frame_id = "track"
        spline_path_msg.poses = poses
        self.planned_path_pub.publish(spline_path_msg)

        # curvature segments
        qutms_path: list[PathPoint] = []
        for i in range(0, spline_len - self.segment, self.segment):
            # check angle between current and 10th spline point ahead
            th_change = th[i + self.segment] - th[i]
            # keep between 360
            if th_change > pi:
                th_change = th_change - 2 * pi
            elif th_change < -pi:
                th_change = th_change + 2 * pi

            # angle relative to max angle on track
            change_pc = abs(th_change) / MAX_ANGLE * 100
            for j in range(self.segment):
                path_point = PathPoint()
                path_point.location.x = tx[i + j]
                path_point.location.y = ty[i + j]
                path_point.turn_intensity = change_pc
                qutms_path.append(path_point)

        # Add the first path point to the end of the list to complete the loop
        qutms_path.append(qutms_path[0])
        qutms_path_msg = QUTMSPathStamped(path=qutms_path)
        qutms_path_msg.header.frame_id = "track"
        self.qutms_path_pub.publish(qutms_path_msg)

        # interpolate boundary points between cones
        interpolated_blues = self.interpolate_boundary(ordered_blues, Cone.BLUE)
        interpolated_yellows = self.interpolate_boundary(ordered_yellows, Cone.YELLOW)
        # Publish list of ordered and interpolated cones
        interpolated_cones = interpolated_blues + interpolated_yellows
        interpolated_cones_msg = ConeDetectionStamped(cones=interpolated_cones)
        interpolated_cones_msg.header = self.current_track.header
        self.interp_cones_pub.publish(interpolated_cones_msg)

        # once we have received the track once, we don't need to keep receiving it
        self.final_path_published = True

    def interpolate_boundary(self, cones, colour):
        """
        Interpolate boundary points between cones at a given distance
        """

        # interpolate between each pair of cones to create a list of Cones
        # use new coordinates function to calculate new points between cones
        interpolated_cones: List[Cone] = []
        for i in range(len(cones)):
            current_cone = cones[i]
            next_cone = cones[(i + 1) % len(cones)]  # wrap around to first cone

            # get angle between current cone and next cone
            angle_rads = angle(current_cone, next_cone)

            # keep between 360
            if angle_rads > pi:
                angle_rads = angle_rads - 2 * pi
            elif angle_rads < -pi:
                angle_rads = angle_rads + 2 * pi

            # calculate distance between cones
            interp_sub_dist = dist(current_cone, next_cone) / self.interp_cone_num

            # interpolate between cones
            for j in range(self.interp_cone_num):
                # distance increases by the sub distance each time
                cone_location = new_coordinates(
                    current_cone[0],
                    current_cone[1],
                    angle_rads,
                    interp_sub_dist * (j + 1),
                )
                # create new cone msg
                cone = Cone()
                cone.location.x = cone_location[0]
                cone.location.y = cone_location[1]
                cone.location.z = 0.0
                cone.color = colour
                interpolated_cones.append(cone)

        return interpolated_cones


def main(args=None):
    # begin ros node
    rclpy.init(args=args)
    node = OrderedMapSpline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
