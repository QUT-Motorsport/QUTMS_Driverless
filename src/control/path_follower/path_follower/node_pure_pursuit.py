from math import cos, sin
import time

import cv2
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sklearn.neighbors import KDTree
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped, State
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image

from driverless_common.common import QOS_LATEST, angle, dist, fast_dist, wrap_to_pi

from typing import List

cv_bridge = CvBridge()
WIDTH = 1000
HEIGHT = 1000


def get_wheel_position(pos_cog: List[float], heading: float) -> List[float]:
    """
    Gets the position of the steering axle from the car's center of gravity and heading
    * param pos_cog: [x,y] coords of the car's center of gravity
    * param heading: car's heading in rads
    * return: [x,y] position of steering axle
    """
    cog2axle = 0.5  # m
    x_axle = pos_cog[0] + cos(heading) * cog2axle
    y_axle = pos_cog[1] + sin(heading) * cog2axle

    return [x_axle, y_axle, heading]


class PurePursuit(Node):
    path = np.array([])
    count = 0
    img_initialised = False
    scale = 1
    x_offset = 0
    y_offset = 0
    following = False
    driving = False
    fallback_path_points_offset = 0

    def __init__(self, node_name="pure_pursuit_node"):
        super().__init__(node_name)

        # subscribers
        self.create_subscription(State, "/system/as_status", self.state_callback, QOS_LATEST)
        self.create_subscription(PathStamped, "/planner/path", self.path_callback, QOS_LATEST)
        # sync subscribers pose + velocity
        # self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, QOS_LATEST)
        self.create_timer((1 / 50), self.timer_callback)  # 50hz state 'prediction'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        self.debug_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/pursuit_img", 1)

        # parameters
        self.Kp_ang = self.declare_parameter("Kp_ang", -3.0).value
        self.lookahead = self.declare_parameter("lookahead", 6.0).value
        self.vel_max = self.declare_parameter("vel_max", 10.0).value
        self.vel_min = self.declare_parameter("vel_min", 4.0).value
        self.DEBUG_IMG = self.declare_parameter("debug_img", True).value

        self.get_logger().info("---Pure pursuit follower initalised---")

    def state_callback(self, msg: State):
        # change to driving state
        if msg.state == State.DRIVING and not self.driving:
            self.driving = True
            self.get_logger().info("Ready to drive")
        if msg.lap_count > 0 and not self.following:
            self.following = True
            self.get_logger().info("Lap completed, following commencing")

    def get_rvwp(self, car_pos: List[float]):
        """
        Retrieve angle between two points
        * param car_pos: [x,y,theta] pose of the car
        * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
        * param rvwp_lookahead: distance to look ahead for the RVWP
        * return: RVWP position as [x,y,i]
        """
        path_xy = [[p[0], p[1]] for p in self.path]

        # find the closest point on the path to the car
        kdtree = KDTree(path_xy)
        close_index = kdtree.query([[car_pos[0], car_pos[1]]], return_distance=False)[0][0]
        close = path_xy[close_index] if close_index is not None else car_pos
        if close_index is None:
            self.get_logger().warn("Could not find closest point, have used car's axle pos")

        # find the first point on the path that is further than the lookahead distance
        # Tragically, there is no way to set a minimum search distance, so I'm giving it the lookahead doubled, we'll
        # receive everything under that distance and filter out the items under the lookahead distance below.
        # Problem there is if there are no points under the double lookahead distance, we're in trouble.
        indexes_raw, distances_raw = kdtree.query_radius(
            [close], r=self.lookahead * 2, return_distance=True, sort_results=True
        )
        indexes_raw, distances_raw = indexes_raw[0], distances_raw[0]

        indexes, distances = [], []
        for i in range(len(indexes_raw)):
            if distances_raw[i] < self.lookahead:
                continue
            # Distances are sorted, so once we get here just grab everything.
            indexes = indexes_raw[i:]
            distances = distances_raw[i:]
            break

        rvwp_dist = float("inf")
        rvwp_index = None
        for i in range(len(indexes)):
            index = indexes[i]
            p = path_xy[index]
            d = distances[i]

            # get angle to check if the point is in front of the car
            ang = angle(close, p)
            error = wrap_to_pi(car_pos[2] - ang)
            if np.pi / 2 > error > -np.pi / 2 and d < rvwp_dist:
                rvwp_dist = d
                rvwp_index = index

        if rvwp_index is None or rvwp_index == close_index:
            self.get_logger().warn("No valid RVWP found, using fallback point")
            path_points_count = len(self.path) - 1
            fallback_point = close_index + self.fallback_path_points_offset
            if fallback_point > path_points_count:
                rvwp_index = abs(path_points_count - fallback_point)
            else:
                rvwp_index = fallback_point

        return self.path[rvwp_index]

    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        self.get_logger().debug(f"Spline Path Recieved - length: {len(spline_path_msg.path)}")
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        distance = 0
        for i in range(len(self.path) - 1):
            prev = self.path[i]
            curr = self.path[i + 1]
            distance += dist(prev, curr)
        # Calculate the number of path points to skip when finding a fallback RVWP.
        # Formula is the number of path points divided by the calculated distance in metres, giving the number of
        # points per metre, which is then multiplied by the lookahead value (also in metres) giving the number of
        # path points that should be skipped.
        self.fallback_path_points_offset = int(round(len(self.path) / distance * self.lookahead))

        if not self.img_initialised:
            # get dimensions of the path
            path_x_min = np.min(self.path[:, 0])
            path_x_max = np.max(self.path[:, 0])
            path_y_min = np.min(self.path[:, 1])
            path_y_max = np.max(self.path[:, 1])

            # scale the path to fit in a 1000x1000 image, pixels per meter
            self.scale = WIDTH / max(path_x_max - path_x_min, path_y_max - path_y_min)

            # add a border around the path for all elements
            self.scale *= 0.90

            # set offsets to the corner of the image
            self.x_offset = -path_x_min * self.scale
            self.x_offset += (WIDTH - (path_x_max - path_x_min) * self.scale) / 2
            self.y_offset = -path_y_min * self.scale
            self.y_offset += (HEIGHT - (path_y_max - path_y_min) * self.scale) / 2

            self.img_initialised = True

    # def callback(self, msg: PoseWithCovarianceStamped):
    def timer_callback(self):
        """
        Listens to odom->base_footprint transform and updates the state.
        Solution is to get the delta and add it to the previous state
        and subtract the delta from the previous map->odom transform.
        """
        # Only start once the path has been recieved, it's a following lap, and we are ready to drive
        if not self.following or not self.driving or self.path.size == 0:
            self.started = False
            return

        if not self.started:
            self.started = True
            self.get_logger().info("Starting pure pursuit")

        try:
            odom_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e))
            return

        start_time = time.perf_counter()

        # i, j, k angles in rad
        theta = quat2euler(
            [
                odom_to_base.transform.rotation.w,
                odom_to_base.transform.rotation.x,
                odom_to_base.transform.rotation.y,
                odom_to_base.transform.rotation.z,
            ]
        )[2]
        # get the position of the center of gravity
        position_cog: List[float] = [odom_to_base.transform.translation.x, odom_to_base.transform.translation.y]
        position: List[float] = get_wheel_position(position_cog, theta)

        # rvwp control
        rvwp: List[float] = self.get_rvwp(position)

        des_heading_ang = angle(position, [rvwp[0], rvwp[1]])
        error = wrap_to_pi(theta - des_heading_ang)
        steering_angle = np.rad2deg(error) * self.Kp_ang

        # velocity control based on steering angle
        des_vel2 = self.vel_min + max((self.vel_max - self.vel_min) * (1 - (abs(steering_angle) / 90) ** 2), 0)

        if self.DEBUG_IMG:
            debug_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
            for i in range(0, len(self.path) - 1):
                cv2.line(
                    debug_img,
                    (
                        int(self.path[i, 0] * self.scale + self.x_offset),
                        int(self.path[i, 1] * self.scale + self.y_offset),
                    ),
                    (
                        int(self.path[i + 1, 0] * self.scale + self.x_offset),
                        int(self.path[i + 1, 1] * self.scale + self.y_offset),
                    ),
                    (255, 0, 0),
                    thickness=5,
                )

            cv2.drawMarker(
                debug_img,
                (int(rvwp[0] * self.scale + self.x_offset), int(rvwp[1] * self.scale + self.y_offset)),
                (0, 255, 0),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=10,
                thickness=4,
            )

            cv2.drawMarker(
                debug_img,
                (int(position[0] * self.scale + self.x_offset), int(position[1] * self.scale + self.y_offset)),
                (0, 0, 255),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=10,
                thickness=4,
            )
            cv2.flip(debug_img, 1, debug_img)

            text_angle = "Steering: " + str(round(steering_angle, 2))
            cv2.putText(debug_img, text_angle, (10, HEIGHT - 75), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
            text_vel = "Velocity: " + str(round(des_vel2, 2))
            cv2.putText(debug_img, text_vel, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
            self.debug_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = des_vel2
        self.control_publisher.publish(control_msg)

        self.count += 1
        if self.count == 50:
            self.count = 0
            self.get_logger().debug(f"{(time.perf_counter() - start_time) * 1000}")


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
