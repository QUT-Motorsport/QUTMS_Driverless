from math import atan2, cos, hypot, pi, sin, sqrt
import time

import numpy as np
from sklearn.neighbors import KDTree
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat, quat2euler

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import ConeDetectionStamped, ConeWithCovariance, Reset, TrackDetectionStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion, TransformStamped, TwistStamped

from driverless_common.shutdown_node import ShutdownNode
from py_slam.cone_props import ConeProps

from typing import Optional

R = np.diag([0.1, 0.001]) ** 2  # motion model
Q_CAM = np.diag([0.5, 0.5]) ** 2  # measurement
Q_LIDAR = np.diag([0.2, 0.2]) ** 2
RADIUS = 1.5  # nn kdtree nearch
LEAF_SIZE = 50  # nodes per tree before it starts brute forcing?
FRAME_COUNT = 20  # minimum frames before confirming cones
FRAME_REM_COUNT = 40  # minimum frames that cones have to be seen in to not be removed
X_RANGE = 15  # max x distance from car
Y_RANGE = 10  # max y distance from car


def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1e9


def wrap_to_pi(angle: float) -> float:  # in rads
    return (angle + pi) % (2 * pi) - pi

# SLAM with IMU velocity and angular velocity
class IMUSlam(Node):
    state = np.array([0.0, 0.0, 0.0])  # initial pose
    sigma = np.diag([0.5, 0.5, 0.001])
    properties = np.array([])

    last_timestamp: Optional[float] = None

    def __init__(self):
        super().__init__("imu_slam_node")

        # sync subscribers
        vel_sub = message_filters.Subscriber(self, TwistStamped, "/imu/velocity")
        lidar_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/lidar/cone_detection")
        lidar_synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[vel_sub, lidar_sub], queue_size=20, slop=0.2
        )
        lidar_synchronizer.registerCallback(self.sync_callback)

        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.callback, 1)
        self.create_subscription(Reset, "/reset", self.reset_callback, 10)

        # slam publisher
        self.slam_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/slam/global_map", 1)
        self.local_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/slam/local_map", 1)
        self.pose_publisher: Publisher = self.create_publisher(PoseWithCovarianceStamped, "/slam/car_pose", 1)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        self.get_logger().info("---SLAM node initialised---")

    def reset_callback(self, msg):
        self.get_logger().info("Resetting Map")
        self.state = np.array([0.0, 0.0, 0.0])
        self.sigma = np.diag([0.5, 0.5, 0.001])
        self.properties = np.array([])

    def sync_callback(self, vel_msg: TwistStamped, detection_msg: ConeDetectionStamped):
        # get velocity timestep
        if self.last_timestamp is None:
            self.last_timestamp = stamp_to_seconds(vel_msg.header.stamp)
            return

        self.dt = stamp_to_seconds(vel_msg.header.stamp) - self.last_timestamp

        # rosbag repeat - reset location
        if self.dt < -1:
            self.last_timestamp = stamp_to_seconds(vel_msg.header.stamp)
            self.state[0:3] = [0.0, 0.0, 0.0]
            return

        self.last_timestamp = stamp_to_seconds(vel_msg.header.stamp)

        # predict car location
        self.predict(vel_msg)
        self.callback(detection_msg)

    def callback(self, msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        start: float = time.perf_counter()

        if msg.header.frame_id == "velodyne":
            Q = Q_LIDAR
        else:
            Q = Q_CAM

        # process detected cones
        track_as_2d = np.array([])
        for detection in msg.cones:
            detection = ConeProps(detection, msg.header.frame_id)  # detection with properties

            # transform detection to map
            rotation_mat = np.array(
                [
                    [cos(self.state[2]), -sin(self.state[2])],
                    [sin(self.state[2]), cos(self.state[2])],
                ]
            )
            map_coords = rotation_mat @ np.array([detection.local_x, detection.local_y]).T + self.state[:2]

            track_as_2d = np.array([])
            for i in range(3, len(self.state), 2):
                track_as_2d = np.append(track_as_2d, [self.state[i], self.state[i + 1]])

            if len(track_as_2d) != 0:
                neighbourhood = KDTree(track_as_2d.reshape(-1, 2), leaf_size=LEAF_SIZE)
                check: np.ndarray = map_coords.reshape(1, -1)  # turn into a 2D row array
                ind: np.ndarray = neighbourhood.query_radius(check, r=RADIUS)  # check neighbours in radius
                close: np.ndarray = ind[0]  # index from the single colour list
                if close.size != 0:
                    # update step
                    prev_mu = self.state[0:3]
                    prev_sigma = self.sigma[0:3, 0:3]
                    updated_detection: ConeProps = self.properties[close[0]]
                    if detection.sensor == "lidar":
                        self.update(close[0], detection, Q)
                        updated_detection.sensor = "lidar"

                    # reset car state if this isn't a confirmed cone
                    # only update states on confirmed cones not noise
                    if not updated_detection.confirmed:
                        self.state[0:3] = prev_mu
                        self.sigma[0:3, 0:3] = prev_sigma

                    idx = close[0] * 2 + 3
                    state = self.state[idx : idx + 2]
                    cov = self.sigma[idx : idx + 2, idx : idx + 2]

                    updated_detection.update(state, cov, detection.colour, FRAME_COUNT)
                    detection = updated_detection

            if not detection.tracked and msg.header.frame_id == "velodyne":
                detection.set_world_coords(map_coords)
                self.properties = np.append(self.properties, detection)
                # initialise new landmark
                self.init_landmark(detection, Q)

        # remove noise
        self.flush_map(track_as_2d.reshape(-1, 2))

        # publish track msg
        track_msg = TrackDetectionStamped()
        track_msg.header.stamp = msg.header.stamp
        track_msg.header.frame_id = "track"
        for detection in self.properties:
            if detection.confirmed:
                track_msg.cones.append(ConeWithCovariance(cone=detection.cone_as_msg, covariance=detection.cov_as_msg))
        self.slam_publisher.publish(track_msg)

        # publish local map msg
        local_map_msg = ConeDetectionStamped()
        local_map_msg.header.stamp = msg.header.stamp
        local_map_msg.header.frame_id = "car"
        for detection in self.get_local_map(track_as_2d.reshape(-1, 2)):
            if detection.confirmed:
                local_map_msg.cones.append(detection.local_cone_as_msg)
        self.local_publisher.publish(local_map_msg)

        # publish localisation msg
        self.publish_localisation(msg)

        self.get_logger().debug(f"Wait time: {str(time.perf_counter()-start)}")  # log time

    def publish_localisation(self, msg):
        # publish pose msg
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "track"
        pose_msg.pose.pose.position = Point(x=self.state[0], y=self.state[1], z=0.2)
        quaternion = euler2quat(0.0, 0.0, self.state[2])
        pose_msg.pose.pose.orientation = Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
        # covariance as a 6x6 matrix
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.sigma[0:2, 0:2]
        cov[0:2, 5] = self.sigma[0:2, 2]
        cov[5, 0:2] = self.sigma[2, 0:2]
        cov[5, 5] = self.sigma[2, 2]
        pose_msg.pose.covariance = cov.flatten().tolist()
        self.pose_publisher.publish(pose_msg)

        # send transformation
        t = TransformStamped()
        # read message content and assign it to corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "track"
        t.child_frame_id = "car"

        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.2
        t.transform.rotation = Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
        self.broadcaster.sendTransform(t)

    def predict(self, vel_msg: TwistStamped):
        """
        Predict step of the EKF
        * param pose: tuple of current (x, y, theta) of the car
        * param cov: current 6x6 covariance matrix of the car
        """

        ddist = (
            sqrt((vel_msg.twist.linear.x / 2) ** 2 + (vel_msg.twist.linear.y / 2) ** 2) * self.dt
        )  # magnitude of distance
        dtheta = -vel_msg.twist.angular.z * self.dt  # change in angle

        Jx = np.array([[1, 0, -ddist * sin(self.state[2])], [0, 1, ddist * cos(self.state[2])], [0, 0, 1]])
        Ju = np.array([[cos(self.state[2]), 0], [sin(self.state[2]), 0], [0, 1]])

        self.state[0] = self.state[0] + ddist * cos(self.state[2])
        self.state[1] = self.state[1] + ddist * sin(self.state[2])
        self.state[2] = wrap_to_pi(self.state[2] + dtheta)

        # uncertainty
        self.sigma[0:3, 0:3] = Jx @ self.sigma[0:3, 0:3] @ Jx.T + Ju @ R @ Ju.T

    def update(self, index: int, detection: ConeProps, Q: np.ndarray):
        """
        Update step of the EKF
        * param index: index of the cone in the map
        * param cone: tuple of (x, y) of the cone
        """

        i = index * 2 + 3  # landmark index, first 3 are vehicle, each landmark has 2 values
        mu_cone = self.state[i : i + 2]  # omit colour from location mean

        r = hypot(self.state[0] - mu_cone[0], self.state[1] - mu_cone[1])  # range to landmark
        b = wrap_to_pi(atan2(mu_cone[1] - self.state[1], mu_cone[0] - self.state[0]) - self.state[2])  # bearing to lm
        h = [r, b]

        sig_len = len(self.sigma)  # length of vehicle+landmarks we've seen so far

        Gt = np.zeros((2, sig_len))
        # vehicle jacobian
        Gt[0:2, 0:3] = [
            [-(mu_cone[0] - self.state[0]) / r, -(mu_cone[1] - self.state[1]) / r, 0],
            [(mu_cone[1] - self.state[1]) / (r**2), -(mu_cone[0] - self.state[0]) / (r**2), 1],
        ]
        # landmark jacobian
        Gt[0:2, i : i + 2] = [
            [(mu_cone[0] - self.state[0]) / r, (mu_cone[1] - self.state[1]) / r],
            [-(mu_cone[1] - self.state[1]) / (r**2), (mu_cone[0] - self.state[0]) / (r**2)],
        ]

        Kt = self.sigma @ Gt.T @ np.linalg.inv(Gt @ self.sigma @ Gt.T + Q)

        # update state, wrap bearing to pi, wrap heading to pi
        self.state = self.state + (Kt @ np.array([detection.range - h[0], wrap_to_pi(detection.bearing - h[1])])).T
        self.state[2] = wrap_to_pi(self.state[2])
        # update cov
        self.sigma = (np.eye(sig_len) - Kt @ Gt) @ self.sigma

    def init_landmark(self, detection: ConeProps, Q: np.ndarray):
        """
        Add new landmark to state
        * param detection: ConeProps object containing the cone properties
        * param Q: measurement noise covariance matrix for this sensor
        """

        self.state = np.append(self.state, detection.map_coords)  # append new landmark

        # landmark Jacobian
        Lz = np.array(
            [
                [cos(self.state[2] + detection.bearing), detection.range * -sin(self.state[2] + detection.bearing)],
                [sin(self.state[2] + detection.bearing), detection.range * cos(self.state[2] + detection.bearing)],
            ]
        )

        sig_len = len(self.sigma)
        new_sig = np.zeros((sig_len + 2, sig_len + 2))  # create zeros
        new_sig[0:sig_len, 0:sig_len] = self.sigma  # top left corner is existing sigma
        new_sig[sig_len : sig_len + 2, sig_len : sig_len + 2] = Lz @ Q @ Lz.T  # bottom right is new lm
        self.sigma = new_sig

    def flush_map(self, track_as_2d: np.ndarray):
        """
        Remove landmarks not seen for a number of frames and only behind the car
        * param track_as_2d: track as a 2d array
        """

        if len(self.properties) == 0:
            return
        heading = np.array([cos(self.state[2]), sin(self.state[2])])
        position = np.array([self.state[0], self.state[1]])

        # get the landmark position vectors
        # if the landmark is behind car, the dot product will be negative
        landmark_position_vectors = track_as_2d - position
        dot_products = np.dot(landmark_position_vectors, heading)
        behind_idxs = np.where(dot_products < 0)[0]

        # get indexes of landmarks that have been seen for a number of frames
        noisy_idxs = np.array([i for i, cone in enumerate(self.properties) if cone.frame_count < FRAME_REM_COUNT])

        # remove noisy and behind landmarks
        idxs_to_remove = np.concatenate((behind_idxs, noisy_idxs))
        unique, count = np.unique(idxs_to_remove, return_counts=True)  # gets unique indexes
        duplicated_idxs = unique[count > 1]  # only gets indexes that are duplicated (behind and noisy)

        if len(duplicated_idxs) > 0:
            self.state = np.delete(self.state, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=0)

            self.sigma = np.delete(self.sigma, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=0)  # rows
            self.sigma = np.delete(self.sigma, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=1)  # columns

            self.properties = np.delete(self.properties, duplicated_idxs, axis=0)

    def get_local_map(self, rotation_mat) -> np.ndarray:
        """
        Get cones within view of the car
        * param rotation_mat: rotation matrix to rotate the map to the car's heading
        * return: array of (x, y) of these cones
        """
        # transform detection to map
        rotation_mat = np.array(
            [
                [cos(self.state[2]), -sin(self.state[2])],
                [sin(self.state[2]), cos(self.state[2])],
            ]
        )

        local_coords = np.array([])
        for i in range(len(self.properties)):
            local = np.linalg.inv(rotation_mat) @ (self.properties[i].map_coords - self.state[0:2])
            local_coords = np.append(local_coords, [local])
            self.properties[i].local_x = local[0]
            self.properties[i].local_y = local[1]
        local_coords = local_coords.reshape(-1, 2)

        # get any cones that are within -10m to 10m beside car
        side_idxs = np.where(np.logical_and(local_coords[:, 1] > -Y_RANGE, local_coords[:, 1] < Y_RANGE))[0]
        # get any cones that are within 15m in front of car
        forward_idxs = np.where(np.logical_and(local_coords[:, 0] > 0, local_coords[:, 0] < X_RANGE))[0]
        # combine indexes
        idxs = np.intersect1d(side_idxs, forward_idxs)

        return self.properties[idxs]


def main(args=None):
    rclpy.init(args=args)
    node = IMUSlam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
