from math import cos, sin
import time

import cv2
import numpy as np
from sklearn.neighbors import KDTree
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecyclePublisher, LifecycleState, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped
from sensor_msgs.msg import Image

from driverless_common.common import QOS_LATEST, angle, dist, wrap_to_pi

from typing import List, Tuple

cv_bridge = CvBridge()
WIDTH = 1000
HEIGHT = 1000


class PurePursuit(LifecycleNode):
    path = None
    count = 0
    img_initialised = False
    scale = 1
    x_offset = 0
    y_offset = 0
    fallback_path_points_offset = 0
    cog2axle = 0.5  # could be a declared parameter
    path_sub: Subscription
    control_pub: LifecyclePublisher
    debug_pub: LifecyclePublisher
    timer: Timer

    def __init__(self, node_name="pure_pursuit_node"):
        super().__init__(node_name)

        # transform listening
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # parameters
        self.Kp_ang = self.declare_parameter("Kp_ang", -3.0).value
        self.lookahead = self.declare_parameter("lookahead", 3.0).value
        self.vel_max = self.declare_parameter("vel_max", 10.0).value
        self.vel_min = self.declare_parameter("vel_min", 4.0).value
        self.DEBUG_IMG = self.declare_parameter("debug_img", False).value

        if node_name == "pure_pursuit_node":
            self.get_logger().info("---Pure pursuit follower initialised---")

    def path_callback(self, spline_path_msg: PathStamped):
        """
        Callback for the path subscriber. Stores the path and calculates the number of path points to skip when
        finding a fallback RVWP. Also initialises the image scaling and offset values.
        """

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

    def can_drive(self):
        if self.path is None:
            return False
        return True

    def timer_callback(self):
        """
        Listens to odom->base_footprint transform and updates the state.
        Solution is to get the delta and add it to the previous state
        and subtract the delta from the previous map->odom transform.
        """

        if not self.can_drive():
            return

        try:
            # TODO: parameterise these frames?
            map_to_base = self.tf_buffer.lookup_transform("track", "base_footprint", rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn("Transform exception: " + str(e))
            return

        start_time = time.perf_counter()

        # i, j, k angles in rad
        theta = quat2euler(
            [
                map_to_base.transform.rotation.w,
                map_to_base.transform.rotation.x,
                map_to_base.transform.rotation.y,
                map_to_base.transform.rotation.z,
            ]
        )[2]
        # get the position of the center of gravity
        position_cog: List[float] = [map_to_base.transform.translation.x, map_to_base.transform.translation.y]
        position: List[float] = self.get_wheel_position(position_cog, theta)

        # rvwp lookup
        rvwp: List[float] = self.get_rvwp(position)

        # steering control
        desired_steering = self.calc_steering(position, rvwp)

        # velocity control based on steering angle
        desired_velocity = self.calc_velocity(desired_steering)

        if self.DEBUG_IMG:
            self.publish_debug_image(desired_steering, desired_velocity, position, rvwp)

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = desired_steering
        control_msg.drive.speed = desired_velocity
        self.control_pub.publish(control_msg)

        self.count += 1
        if self.count == 50:
            self.count = 0
            self.get_logger().debug(f"{(time.perf_counter() - start_time) * 1000}")

    def calc_steering(self, pose: List[float], rvwp: List[float]) -> float:
        """
        Calculates the steering angle based on the pose of the car and the RVWP.
        Gets the angle between the car and the RVWP and calculates the error between the desired heading and the
        current heading. The steering angle is then calculated using the error and the proportional gain.
        """

        des_heading_ang = angle(pose[:2], [rvwp[0], rvwp[1]])
        error = wrap_to_pi(pose[2] - des_heading_ang)
        steering = np.rad2deg(error) * self.Kp_ang
        return steering

    def calc_velocity(self, desired_steering: float) -> float:
        """
        Calculates the velocity based on the steering angle.
        Reduces the velocity as the steering angle increases
        """

        vel = self.vel_min + max((self.vel_max - self.vel_min) * (1 - (abs(desired_steering) / 90) ** 2), 0)
        return vel

    def get_wheel_position(self, pos_cog: List[float], heading: float) -> List[float]:
        """
        Gets the position of the steering axle from the car's center of gravity and heading
        * param pos_cog: [x,y] coords of the car's center of gravity
        * param heading: car's heading in rads
        * return: [x,y] position of steering axle
        """

        x_axle = pos_cog[0] + cos(heading) * self.cog2axle
        y_axle = pos_cog[1] + sin(heading) * self.cog2axle

        return [x_axle, y_axle, heading]

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

    def get_img_params(self) -> Tuple[float, float, float]:
        # get dimensions of the path
        path_x_min = np.min(self.path[:, 0])
        path_x_max = np.max(self.path[:, 0])
        path_y_min = np.min(self.path[:, 1])
        path_y_max = np.max(self.path[:, 1])

        # scale the path to fit in a 1000x1000 image, pixels per meter
        scale = WIDTH / max(path_x_max - path_x_min, path_y_max - path_y_min)

        # add a border around the path for all elements
        scale *= 0.90

        # set offsets to the corner of the image
        x_offset = -path_x_min * scale
        x_offset += (WIDTH - (path_x_max - path_x_min) * scale) / 2
        y_offset = -path_y_min * scale
        y_offset += (HEIGHT - (path_y_max - path_y_min) * scale) / 2

        return scale, x_offset, y_offset

    def draw_rvwp(self, img_params, rvwp: List[float], position: List[float]) -> np.ndarray:
        scale = img_params[0]
        x_offset = img_params[1]
        y_offset = img_params[2]

        debug_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        for i in range(0, len(self.path) - 1):
            cv2.line(
                debug_img,
                (
                    int(self.path[i, 0] * scale + x_offset),
                    int(self.path[i, 1] * scale + y_offset),
                ),
                (
                    int(self.path[i + 1, 0] * scale + x_offset),
                    int(self.path[i + 1, 1] * scale + y_offset),
                ),
                (255, 0, 0),
                thickness=5,
            )

        cv2.drawMarker(
            debug_img,
            (int(rvwp[0] * scale + x_offset), int(rvwp[1] * scale + y_offset)),
            (0, 255, 0),
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=10,
            thickness=4,
        )

        cv2.drawMarker(
            debug_img,
            (int(position[0] * scale + x_offset), int(position[1] * scale + y_offset)),
            (0, 0, 255),
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=10,
            thickness=4,
        )

        return debug_img

    def add_data_text(self, debug_img, steering_angle: float, velocity: float) -> np.ndarray:
        cv2.flip(debug_img, 1, debug_img)

        text_angle = "Steering: " + str(round(steering_angle, 2))
        cv2.putText(debug_img, text_angle, (10, HEIGHT - 75), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)
        text_vel = "Velocity: " + str(round(velocity, 2))
        cv2.putText(debug_img, text_vel, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4)

        return debug_img

    def publish_debug_image(self, steering_angle: float, velocity: float, rvwp: List[float], position: List[float]):
        img_params = self.get_img_params()
        debug_img = self.draw_rvwp(img_params, rvwp, position)
        debug_img = self.add_data_text(debug_img, steering_angle, velocity)

        self.debug_pub.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure")
        self.control_pub = self.create_lifecycle_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        if self.DEBUG_IMG:
            self.debug_pub = self.create_lifecycle_publisher(Image, "/debug_imgs/pursuit_img", QOS_LATEST)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate")
        self.path_sub = self.create_subscription(PathStamped, "/planner/path", self.path_callback, QOS_LATEST)
        self.timer = self.create_timer((1 / 50), self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_error")
        self.destroy_subscription(self.path_sub)
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.control_pub)
        self.destroy_lifecycle_publisher(self.debug_pub)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
