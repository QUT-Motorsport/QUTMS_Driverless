from math import atan2, cos, hypot, pi, sin, sqrt
import time

from geodesy.utm import UTMPoint, fromLatLong
import numpy as np
from sklearn.neighbors import KDTree
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat, quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import ConeDetectionStamped, ConeWithCovariance, Reset, TrackDetectionStamped
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped, TwistStamped
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgGpsPos, SbgGpsVel, SbgMag
from sensor_msgs.msg import Imu, NavSatFix

from py_slam.cone_props import ConeProps

from typing import Optional, Tuple

R = np.diag([0.1, 0.001]) ** 2  # motion model
Q_CAM = np.diag([0.5, 0.5]) ** 2  # measurement
Q_LIDAR = np.diag([0.2, 0.2]) ** 2
RADIUS = 1.5  # nn kdtree nearch
LEAF_SIZE = 50  # nodes per tree before it starts brute forcing?
FRAME_COUNT = 20  # minimum frames before confirming cones
FRAME_REM_COUNT = 40  # minimum frames that cones have to be seen in to not be removed
X_RANGE = 15  # max x distance from car
Y_RANGE = 10  # max y distance from car


def wrap_to_pi(angle: float) -> float:  # in rads
    return (angle + pi) % (2 * pi) - pi


class PySlam(Node):
    initial_pos: Optional[Tuple[float, float]] = None
    initial_ang: Optional[float] = None
    state = np.array([0.0, 0.0, 0.0])  # initial pose
    sigma = np.diag([0.5, 0.5, 0.001])
    properties = np.array([])

    last_timestamp: Optional[float] = None

    def __init__(self):
        super().__init__("py_slam")

        # sync subscribers
        ekf_nav_sub = message_filters.Subscriber(self, SbgEkfNav, "/sbg/ekf_nav")
        ekf_euler_sub = message_filters.Subscriber(self, SbgEkfEuler, "/sbg/ekf_euler")
        sbg_synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[ekf_nav_sub, ekf_euler_sub], queue_size=20, slop=0.2
        )
        sbg_synchronizer.registerCallback(self.nav_callback)

        # gps_sub = message_filters.Subscriber(self, NavSatFix, "/imu/nav_sat_fix")
        # imu_sub = message_filters.Subscriber(self, Imu, "/imu/data")
        # imu_synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[gps_sub, imu_sub], queue_size=20, slop=0.2)
        # imu_synchronizer.registerCallback(self.imu_callback)

        # self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.callback, 1)
        # self.create_subscription(ConeDetectionStamped, "/vision/cone_detection2", self.callback, 1)
        self.create_subscription(Reset, "/system/reset", self.reset_callback, 10)

        # ekf_nav data visualisation publishers
        self.ekf_nav_coords_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/ekf_nav_coords_point", 1
        )
        self.ekf_nav_lat_long_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/ekf_nav_lat_long_point", 1
        )
        self.ekf_nav_lat_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/ekf_nav_lat_time_point", 1
        )
        self.ekf_nav_long_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/ekf_nav_long_time_point", 1
        )
        self.ekf_nav_altitude_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/ekf_nav_altitude_time_point", 1
        )

        # sbg & imu data visualisation subscribers
        self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.sbg_gps_pos_callback, 1)
        self.create_subscription(SbgGpsVel, "/sbg/gps_vel", self.sbg_gps_vel_callback, 1)
        self.create_subscription(SbgMag, "/sbg/mag", self.sbg_mag_callback, 1)
        self.create_subscription(Imu, "/imu/data", self.imu_data_callback, 1)

        # sbg gps pos visualisation publishers
        self.sbg_gps_pos_coords_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_pos_coords_point", 1
        )
        self.sbg_gps_pos_lat_long_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_pos_lat_long_point", 1
        )
        self.sbg_gps_pos_lat_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_pos_lat_time_point", 1
        )
        self.sbg_gps_pos_long_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_pos_long_time_point", 1
        )
        self.sbg_gps_pos_altitude_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_pos_altitude_time_point", 1
        )

        # sbg gps vel visualisation publishers
        self.sbg_gps_vel_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_vel_x_time_point", 1
        )
        self.sbg_gps_vel_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_vel_y_time_point", 1
        )
        self.sbg_gps_vel_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_gps_vel_z_time_point", 1
        )

        # sbg mag visualisation publishers
        self.sbg_mag_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_x_time_point", 1
        )
        self.sbg_mag_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_y_time_point", 1
        )
        self.sbg_mag_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_z_time_point", 1
        )
        self.sbg_mag_accel_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_accel_x_time_point", 1
        )
        self.sbg_mag_accel_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_accel_y_time_point", 1
        )
        self.sbg_mag_accel_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/sbg_mag_accel_z_time_point", 1
        )

        # imu data visualisation publishers
        self.imu_orientation_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_orientation_x_time_point", 1
        )
        self.imu_orientation_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_orientation_y_time_point", 1
        )
        self.imu_orientation_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_orientation_z_time_point", 1
        )
        self.imu_orientation_w_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_orientation_w_time_point", 1
        )
        self.imu_ang_vel_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_ang_vel_x_time_point", 1
        )
        self.imu_ang_vel_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_ang_vel_y_time_point", 1
        )
        self.imu_ang_vel_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_ang_vel_z_time_point", 1
        )
        self.imu_accel_x_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_accel_x_time_point", 1
        )
        self.imu_accel_y_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_accel_y_time_point", 1
        )
        self.imu_accel_z_time_point_publisher: Publisher = self.create_publisher(
            PointStamped, "/debug/imu_accel_z_time_point", 1
        )

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        self.get_logger().info("---SLAM node initialised---")

    def sbg_gps_pos_callback(self, sbg_gps_pos_msg: SbgGpsPos):
        coords: UTMPoint = fromLatLong(sbg_gps_pos_msg.latitude, sbg_gps_pos_msg.longitude, sbg_gps_pos_msg.altitude)

        # ----------------------------- SBG OUTPUT DEBUGGING (plotting outputs) -----------------------------
        # plotting easting vs northing
        point_coords_msg: PointStamped = PointStamped(
            header=sbg_gps_pos_msg.header, point=Point(x=coords.easting, y=coords.northing, z=0.0)
        )
        self.sbg_gps_pos_coords_point_publisher.publish(point_coords_msg)

        # plotting lat vs long
        point_lat_long_msg: PointStamped = PointStamped(
            header=sbg_gps_pos_msg.header,
            point=Point(x=sbg_gps_pos_msg.latitude * 100000, y=sbg_gps_pos_msg.longitude * 100000, z=0.0),
        )
        self.sbg_gps_pos_lat_long_point_publisher.publish(point_lat_long_msg)

        # Plotting lat, long, and altitude vs time
        timestamp: float = sbg_gps_pos_msg.header.stamp.sec + sbg_gps_pos_msg.header.stamp.nanosec * 10**-9

        point_lat_time_msg: PointStamped = PointStamped(
            header=sbg_gps_pos_msg.header, point=Point(x=timestamp, y=sbg_gps_pos_msg.latitude * 100000, z=0.0)
        )
        self.sbg_gps_pos_lat_time_point_publisher.publish(point_lat_time_msg)
        point_long_time_msg: PointStamped = PointStamped(
            header=sbg_gps_pos_msg.header, point=Point(x=timestamp, y=sbg_gps_pos_msg.longitude * 100000, z=0.0)
        )
        self.sbg_gps_pos_long_time_point_publisher.publish(point_long_time_msg)
        point_altitude_time_msg: PointStamped = PointStamped(
            header=sbg_gps_pos_msg.header, point=Point(x=timestamp, y=sbg_gps_pos_msg.altitude, z=0.0)
        )
        self.sbg_gps_pos_altitude_time_point_publisher.publish(point_altitude_time_msg)
        # ---------------------------------------------------------------------------------------------------

    def sbg_gps_vel_callback(self, sbg_gps_vel_msg: SbgGpsVel):
        # ----------------------------- SBG OUTPUT DEBUGGING (plotting outputs) -----------------------------
        # Plotting the x, y, and z velocities vs time
        timestamp: float = sbg_gps_vel_msg.header.stamp.sec + sbg_gps_vel_msg.header.stamp.nanosec * 10**-9

        point_vel_x_time_msg: PointStamped = PointStamped(
            header=sbg_gps_vel_msg.header, point=Point(x=timestamp, y=sbg_gps_vel_msg.velocity.x, z=0.0)
        )
        self.sbg_gps_vel_x_time_point_publisher.publish(point_vel_x_time_msg)
        point_vel_y_time_msg: PointStamped = PointStamped(
            header=sbg_gps_vel_msg.header, point=Point(x=timestamp, y=sbg_gps_vel_msg.velocity.y, z=0.0)
        )
        self.sbg_gps_vel_y_time_point_publisher.publish(point_vel_y_time_msg)
        point_vel_z_time_msg: PointStamped = PointStamped(
            header=sbg_gps_vel_msg.header, point=Point(x=timestamp, y=sbg_gps_vel_msg.velocity.z, z=0.0)
        )
        self.sbg_gps_vel_z_time_point_publisher.publish(point_vel_z_time_msg)
        # ---------------------------------------------------------------------------------------------------

    def sbg_mag_callback(self, sbg_mag_msg: SbgMag):
        # ----------------------------- SBG OUTPUT DEBUGGING (plotting outputs) -----------------------------
        # Plotting mag x, y, and z vs time, and acceleration x, y, and z vs time
        timestamp: float = sbg_mag_msg.header.stamp.sec + sbg_mag_msg.header.stamp.nanosec * 10**-9

        # mag
        point_mag_x_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.mag.x, z=0.0)
        )
        self.sbg_mag_x_time_point_publisher.publish(point_mag_x_time_msg)
        point_mag_y_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.mag.y, z=0.0)
        )
        self.sbg_mag_y_time_point_publisher.publish(point_mag_y_time_msg)
        point_mag_z_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.mag.z, z=0.0)
        )
        self.sbg_mag_z_time_point_publisher.publish(point_mag_z_time_msg)

        # mag accel
        point_mag_accel_x_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.accel.x, z=0.0)
        )
        self.sbg_mag_accel_x_time_point_publisher.publish(point_mag_accel_x_time_msg)
        point_mag_accel_y_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.accel.y, z=0.0)
        )
        self.sbg_mag_accel_y_time_point_publisher.publish(point_mag_accel_y_time_msg)
        point_mag_accel_z_time_msg: PointStamped = PointStamped(
            header=sbg_mag_msg.header, point=Point(x=timestamp, y=sbg_mag_msg.accel.z, z=0.0)
        )
        self.sbg_mag_accel_z_time_point_publisher.publish(point_mag_accel_z_time_msg)
        # ---------------------------------------------------------------------------------------------------

    def imu_data_callback(self, imu_data_msg: Imu):
        # ----------------------------- SBG OUTPUT DEBUGGING (plotting outputs) -----------------------------
        # Plotting orientation x, y, z, and w vs time, angular velocity x, y, and z vs time, and linear acceleration x, y, and z vs time
        timestamp: float = imu_data_msg.header.stamp.sec + imu_data_msg.header.stamp.nanosec * 10**-9

        # orientation
        point_imu_orientation_x_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.orientation.x * 300, z=0.0)
        )
        self.imu_orientation_x_time_point_publisher.publish(point_imu_orientation_x_time_msg)
        point_imu_orientation_y_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.orientation.y * 300, z=0.0)
        )
        self.imu_orientation_y_time_point_publisher.publish(point_imu_orientation_y_time_msg)
        point_imu_orientation_z_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.orientation.z, z=0.0)
        )
        self.imu_orientation_z_time_point_publisher.publish(point_imu_orientation_z_time_msg)
        point_imu_orientation_w_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.orientation.w, z=0.0)
        )
        self.imu_orientation_w_time_point_publisher.publish(point_imu_orientation_w_time_msg)

        # angular velocity
        point_imu_ang_vel_x_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.angular_velocity.x, z=0.0)
        )
        self.imu_ang_vel_x_time_point_publisher.publish(point_imu_ang_vel_x_time_msg)
        point_imu_ang_vel_y_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.angular_velocity.y, z=0.0)
        )
        self.imu_ang_vel_y_time_point_publisher.publish(point_imu_ang_vel_y_time_msg)
        point_imu_ang_vel_z_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.angular_velocity.z, z=0.0)
        )
        self.imu_ang_vel_z_time_point_publisher.publish(point_imu_ang_vel_z_time_msg)

        # linear acceleration
        point_imu_accel_x_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.linear_acceleration.x, z=0.0)
        )
        self.imu_accel_x_time_point_publisher.publish(point_imu_accel_x_time_msg)
        point_imu_accel_y_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.linear_acceleration.y, z=0.0)
        )
        self.imu_accel_y_time_point_publisher.publish(point_imu_accel_y_time_msg)
        point_imu_accel_z_time_msg: PointStamped = PointStamped(
            header=imu_data_msg.header, point=Point(x=timestamp, y=imu_data_msg.linear_acceleration.z, z=0.0)
        )
        self.imu_accel_z_time_point_publisher.publish(point_imu_accel_z_time_msg)
        # ---------------------------------------------------------------------------------------------------

    def nav_callback(self, ekf_nav_msg: SbgEkfNav, ekf_euler_msg: SbgEkfEuler):
        coords: UTMPoint = fromLatLong(ekf_nav_msg.latitude, ekf_nav_msg.longitude, ekf_nav_msg.altitude)

        # ----------------------------- SBG OUTPUT DEBUGGING (plotting outputs) -----------------------------
        # plotting easting vs northing
        point_coords_msg: PointStamped = PointStamped(
            header=ekf_nav_msg.header, point=Point(x=coords.easting, y=coords.northing, z=0.0)
        )
        self.ekf_nav_coords_point_publisher.publish(point_coords_msg)

        # plotting lat vs long
        point_lat_long_msg: PointStamped = PointStamped(
            header=ekf_nav_msg.header,
            point=Point(x=ekf_nav_msg.latitude * 100000, y=ekf_nav_msg.longitude * 100000, z=0.0),
        )
        self.ekf_nav_lat_long_point_publisher.publish(point_lat_long_msg)

        # Plotting lat, long, and altitude vs time
        timestamp: float = ekf_nav_msg.header.stamp.sec + ekf_nav_msg.header.stamp.nanosec * 10**-9

        point_lat_time_msg: PointStamped = PointStamped(
            header=ekf_nav_msg.header, point=Point(x=timestamp, y=ekf_nav_msg.latitude * 100000, z=0.0)
        )
        self.ekf_nav_lat_time_point_publisher.publish(point_lat_time_msg)
        point_long_time_msg: PointStamped = PointStamped(
            header=ekf_nav_msg.header, point=Point(x=timestamp, y=ekf_nav_msg.longitude * 100000, z=0.0)
        )
        self.ekf_nav_long_time_point_publisher.publish(point_long_time_msg)
        point_altitude_time_msg: PointStamped = PointStamped(
            header=ekf_nav_msg.header, point=Point(x=timestamp, y=ekf_nav_msg.altitude, z=0.0)
        )
        self.ekf_nav_altitude_time_point_publisher.publish(point_altitude_time_msg)
        # ---------------------------------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = PySlam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
