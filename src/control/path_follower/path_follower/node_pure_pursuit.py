from math import atan2, cos, sin, sqrt
import time

import cv2
import numpy as np
from transforms3d.euler import quat2euler

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image

from driverless_common.common import angle, dist, wrap_to_pi
from driverless_common.draw import loc_to_img_pt
from driverless_common.shutdown_node import ShutdownNode

from typing import List, Tuple

LOOKAHEAD = 4
DEBUG = True
SCALE = 80
WIDTH = 20 * SCALE  # 15m either side
HEIGHT = 20 * SCALE  # 30m forward
OFFSET_X = 15
OFFSET_Y = 30

cv_bridge = CvBridge()


def get_wheel_position(pos_cog: List[float], heading: float) -> List[float]:
    """
    Gets the position of the steering axle from the car's center of gravity and heading
    * param pos_cog: [x,y] coords of the car's center of gravity
    * param heading: car's heading in rads
    * return: [x,y] position of steering axle
    """
    # https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/vehicle_model/
    cog2axle = 0.4  # m
    x_axle = pos_cog[0] + cos(heading) * cog2axle
    y_axle = pos_cog[1] + sin(heading) * cog2axle

    return [x_axle, y_axle, heading]


def get_RVWP(car_pos: List[float], path: np.ndarray, lookahead: float) -> np.ndarray:
    """
    Retrieve angle between two points
    * param car_pos: [x,y,theta] pose of the car
    * param path: [[x0,y0,i0],[x1,y1,i1],...,[xn-1,yn-1,in-1]] path points
    * param rvwp_lookahead: distance to look ahead for the RVWP
    * return: RVWP position as [x,y,i]
    """
    # find the closest point on the path to the car
    close_dist: float = 100.0
    for i, p in enumerate(path):
        if dist(p, car_pos) < close_dist:
            close_dist = dist(p, car_pos)
            min_index = i
    close = path[min_index]

    # find the first point on the path that is further than the lookahead distance
    rvwp_dist: float = 100.0
    for i, p in enumerate(path):
        current_dist = dist(p, close)
        if current_dist > lookahead and current_dist < rvwp_dist:
            # get angle to check if the point is in front of the car
            ang = angle(close, p)
            if (car_pos[2] - ang) < np.pi / 2 and (car_pos[2] - ang) > -np.pi / 2:
                rvwp_dist = current_dist
                rvwp = path[i]
    return rvwp


class PurePursuit(Node):
    path = np.array([])
    Kp_ang: float = -5.0
    Kp_vel: float = 0.08
    vel_max: float = 2.0  # m/s
    vel_min: float = 2.0  # m/s
    throttle_max: float = 0.2
    brake_max: float = 0.12
    Kp_brake: float = 0.0
    last_time = time.time()
    count = 0

    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.create_subscription(PathStamped, "/planner/path", self.path_callback, 10)
        # sync subscribers pose + velocity
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.callback, 10)

        # publishers
        self.control_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 10)
        self.debug_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/pursuit_img", 1)

        self.get_logger().info("---Path Follower Node Initalised---")

    def path_callback(self, spline_path_msg: PathStamped):
        # convert List[PathPoint] to 2D numpy array
        self.path = np.array([[p.location.x, p.location.y, p.turn_intensity] for p in spline_path_msg.path])
        self.get_logger().debug(f"Spline Path Recieved - length: {len(self.path)}")

    def callback(self, msg: PoseWithCovarianceStamped):
        # Only start once the path has been recieved
        if self.path.size == 0:
            return

        # i, j, k angles in rad
        theta = quat2euler(
            [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )[2]
        # get the position of the center of gravity
        position_cog: List[float] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        position: List[float] = get_wheel_position(position_cog, theta)

        # rvwp control
        rvwp: List[float] = get_RVWP(position, self.path, LOOKAHEAD)

        des_heading_ang = angle(position, [rvwp[0], rvwp[1]])
        error = wrap_to_pi(theta - des_heading_ang)
        steering_angle = np.rad2deg(error) * self.Kp_ang

        if DEBUG:
            debug_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
            for i in range(0, len(self.path) - 1):
                cv2.line(
                    debug_img,
                    loc_to_img_pt(self.path[i, 0] - OFFSET_X, self.path[i, 1] - OFFSET_Y).to_tuple(),
                    loc_to_img_pt(self.path[i + 1, 0] - OFFSET_X, self.path[i + 1, 1] - OFFSET_Y).to_tuple(),
                    (255, 0, 0),
                    thickness=3,
                )

            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(rvwp[0] - OFFSET_X, rvwp[1] - OFFSET_Y).to_tuple(),
                (0, 255, 0),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=7,
                thickness=4,
            )

            cv2.drawMarker(
                debug_img,
                loc_to_img_pt(position[0] - OFFSET_X, position[1] - OFFSET_Y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=7,
                thickness=4,
            )
            text_angle = "Steering: " + str(round(steering_angle, 2))
            cv2.putText(debug_img, text_angle, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 4)

            self.debug_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        # publish message
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = steering_angle
        control_msg.drive.speed = self.vel_min
        self.control_publisher.publish(control_msg)

        self.count += 1
        if self.count % 100 == 0:
            print("cars coords", position)
            print("rvwp coords", rvwp)
            print("desired heading", np.rad2deg(des_heading_ang))
            print("current heading", np.rad2deg(theta))
            print("error", np.rad2deg(error))
            print("wheel angles", np.rad2deg(steering_angle) / (90 / 26))
            self.count = 0

            print("Time: ", time.time() - self.last_time, "\n")

        self.last_time = time.time()


def main(args=None):  # begin ros node
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
