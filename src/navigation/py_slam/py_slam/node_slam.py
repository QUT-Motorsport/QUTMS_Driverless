from math import atan2, cos, hypot, pi, sin
import time

import numpy as np
from sklearn.neighbors import KDTree
from transforms3d.euler import quat2euler, euler2quat

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from tf2_ros import TransformBroadcaster

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance, TrackDetectionStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion, TransformStamped

from driverless_common.cone_props import ConeProps

from typing import List, Tuple


def wrap_to_pi(angle: float) -> float:  # in rads
    return (angle + pi) % (2 * pi) - pi


class EKFSlam(Node):
    R = np.diag([0.005, 0.005, 0.005])  # very confident of odom (cause its OP)
    Q = np.diag([1, 0.8]) ** 2  # detections are a bit meh
    radius = 2  # nn kdtree nearch
    leaf = 50  # nodes per tree before it starts brute forcing?
    in_frames = 6  # minimum frames that cones have to be seen in
    mu = np.array([0.0, 0.0, 0.0])  # initial pose
    Sigma: np.ndarray = np.diag([0.01, 0.01, 0.01])
    track: np.ndarray = []

    # init pose on first odom message
    last_measure: np.ndarray = np.array([0.0, 0.0, 0.0])

    def __init__(self):
        super().__init__("ekf_slam")

        # sync subscribers
        pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "/zed2i/zed_node/pose_with_covariance")
        vision_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/vision/cone_detection")
        lidar_sub = message_filters.Subscriber(self, ConeDetectionStamped, "/lidar/cone_detection")
        vision_synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[pose_sub, vision_sub], queue_size=20, slop=0.2
        )
        vision_synchronizer.registerCallback(self.callback)
        lidar_synchronizer = message_filters.ApproximateTimeSynchronizer(
            fs=[pose_sub, lidar_sub], queue_size=20, slop=0.2
        )
        lidar_synchronizer.registerCallback(self.callback)

        # slam publisher
        self.slam_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/slam/track", 1)
        self.local_publisher: Publisher = self.create_publisher(TrackDetectionStamped, "/slam/local", 1)
        self.pose_publisher: Publisher = self.create_publisher(PoseWithCovarianceStamped, "/slam/pose", 1)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        self.get_logger().info("---SLAM node initialised---")


    def callback(self, pose_msg: PoseWithCovarianceStamped, detection_msg: ConeDetectionStamped):
        self.get_logger().debug("Received detection")
        start: float = time.perf_counter()

        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        theta = quat2euler(
            [
                pose_msg.pose.pose.orientation.w,
                pose_msg.pose.pose.orientation.x,
                pose_msg.pose.pose.orientation.y,
                pose_msg.pose.pose.orientation.z,
            ]
        )[2]
        cov = pose_msg.pose.covariance

        if (self.last_measure == 0.0).all():
            self.last_measure = np.array([x, y, theta])
            return

        # predict car location
        self.predict((x, y, theta), cov)

        # process detected cones
        for cone in detection_msg.cones:
            det = ConeProps(cone)  # detection with properties

            mapx = self.mu[0] + det.range * cos(self.mu[2] + det.bearing)
            mapy = self.mu[1] + det.range * sin(self.mu[2] + det.bearing)

            on_map = False  # by default its a new detection

            if len(self.track) != 0:
                neighbourhood = KDTree(self.track[:, :2], leaf_size=self.leaf)
                check = np.reshape([mapx, mapy], (1, -1))  # turn into a 2D row array
                ind = neighbourhood.query_radius(check, r=self.radius)  # check neighbours in radius
                close = ind[0]  # index from the single colour list
                if close.size != 0:
                    on_map = True
                    # update step
                    self.update(close[0], det.sense_rb)

                    if det.colour != Cone.UNKNOWN:  # updated cone was not a lidar detection
                        self.track[close[0]][2] = det.colour  # override colour

            if not on_map:
                if self.track == []:  # first in this list
                    self.track = np.array([mapx, mapy, det.colour, 1])
                    self.track = np.reshape(self.track, (1, -1))  # turn 2D
                else:  # otherwise append vertically
                    self.track = np.vstack([self.track, [mapx, mapy, det.colour, 1]])
                # initialise new landmark
                self.init_landmark(det.sense_rb)

        # remove noise
        self.flush_map()

        # get local map
        local_map = self.get_local_map()

        # publish track msg
        track_msg = TrackDetectionStamped()
        track_msg.header.stamp = self.get_clock().now().to_msg()
        track_msg.header.frame_id = "map"
        for i, cone in enumerate(self.track):
            cov_i = i * 2 + 3
            curr_cov: np.ndarray = self.Sigma[cov_i : cov_i + 2, cov_i : cov_i + 2]  # 2x2 covariance to plot
            cone_msg = Cone(location=Point(x=cone[0], y=cone[1], z=0.0), color=int(cone[2]))
            cone_cov = curr_cov.flatten().tolist()
            track_msg.cones.append(ConeWithCovariance(cone=cone_msg, covariance=cone_cov))
        self.slam_publisher.publish(track_msg)

        # publish local map msg
        local_map_msg = TrackDetectionStamped()
        local_map_msg.header.stamp = self.get_clock().now().to_msg()
        local_map_msg.header.frame_id = "base_link"
        for i, cone in enumerate(local_map):
            cov_i = i * 2 + 3
            curr_cov: np.ndarray = self.Sigma[cov_i : cov_i + 2, cov_i : cov_i + 2]
            cone_msg = Cone(location=Point(x=cone[0], y=cone[1], z=0.0), color=int(cone[2]))
            cone_cov = curr_cov.flatten().tolist()
            local_map_msg.cones.append(ConeWithCovariance(cone=cone_msg, covariance=cone_cov))
        self.local_publisher.publish(local_map_msg)

        # publish pose msg
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position = Point(x=self.mu[0], y=self.mu[1], z=0.2)
        quaternion = euler2quat(0.0, 0.0, self.mu[2])
        pose_msg.pose.pose.orientation = Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
        # covariance as a 6x6 matrix
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.Sigma[0:2, 0:2]
        cov[0:2, 5] = self.Sigma[0:2, 2]
        cov[5, 0:2] = self.Sigma[2, 0:2]
        cov[5, 5] = self.Sigma[2, 2]
        pose_msg.pose.covariance = cov.flatten().tolist()
        self.pose_publisher.publish(pose_msg)

        # send transformation
        t = TransformStamped()
        # read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "car"

        t.transform.translation.x = self.mu[0]
        t.transform.translation.y = self.mu[1]
        t.transform.translation.z = 0.2
        t.transform.rotation = Quaternion(x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0])
        self.broadcaster.sendTransform(t)

        # self.get_logger().info(f"Wait time: {str(time.perf_counter()-start)}")  # log time

    # predict step
    def predict(self, pose: tuple, cov: np.ndarray) -> Tuple[np.ndarray]:
        dx = pose[0] - self.last_measure[0]
        dy = pose[1] - self.last_measure[1]
        dx = (pose[0] - self.last_measure[0]) * cos(self.last_measure[2]) \
            + (pose[1] - self.last_measure[1]) * sin(self.last_measure[2])
        dy = -(pose[0] - self.last_measure[0]) * sin(self.last_measure[2]) \
            + (pose[1] - self.last_measure[1]) * cos(self.last_measure[2])
            
        dtheta = wrap_to_pi(pose[2] - self.last_measure[2])

        self.mu[0:3] += np.array([dx, dy, dtheta])
        self.mu[2] = wrap_to_pi(self.mu[2])

        """
        Covariance from odom, X Y Z Roll Pitch yaW in 1 list
        xx, xy, xz, xr, xp, xw
        yx, yy, yz, yr, yp, yw
        zx, zy, zz, zr, zp, zw
        rx, ry, rz, rr, rp, rw
        px, py, pz, pr, pp, pw
        wx, wy, wz, wr, wp, ww
        """
        cov = np.reshape(cov, (6, 6))
        # self.Sigma[0:3, 0:3] = np.array(
        #     [[cov[0, 0], cov[0, 1], cov[0, 5]], [cov[1, 0], cov[1, 1], cov[1, 5]], [cov[5, 0], cov[5, 1], cov[5, 5]]]
        # )

        # uncertainty
        Jx = np.diag([1, 1, 1])
        # self.Sigma[0:3, 0:3] = Jx @ self.Sigma[0:3, 0:3] @ Jx.T + self.R

        self.last_measure = np.array([pose[0], pose[1], pose[2]])

    # update step
    def update(self, index: int, cone: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray]:
        i = index * 2 + 3  # landmark index, first 3 are robot, each landmark has 2 values
        muL = self.mu[i : i + 2]  # omit colour from location mean

        r = hypot(self.mu[0] - muL[0], self.mu[1] - muL[1])  # range to landmark
        b = wrap_to_pi(atan2(muL[1] - self.mu[1], muL[0] - self.mu[0]) - self.mu[2])  # bearing to lm
        h = [r, b]

        sig_len = len(self.Sigma)  # length of robot+landmarks we've seen so far

        Gt = np.zeros((2, sig_len))
        # robot jacobian
        Gt[0:2, 0:3] = [
            [-(muL[0] - self.mu[0]) / r, -(muL[1] - self.mu[1]) / r, 0],
            [(muL[1] - self.mu[1]) / (r**2), -(muL[0] - self.mu[0]) / (r**2), 1],
        ]
        # landmark jacobian
        Gt[0:2, i : i + 2] = [[(muL[0] - self.mu[0]) / r, (muL[1] - self.mu[1]) / r], [-(muL[1] - self.mu[1]) / (r**2), (muL[0] - self.mu[0]) / (r**2)]]

        Kt = self.Sigma @ Gt.T @ np.linalg.inv(Gt @ self.Sigma @ Gt.T + self.Q)

        self.mu = self.mu + (Kt @ np.array([cone[0] - h[0], cone[1] - h[1]])).T 
        self.mu[3] = wrap_to_pi(self.mu[3])  # wrap angle
        self.Sigma = (np.eye(sig_len) - Kt @ Gt) @ self.Sigma

        self.track[index, :2] = self.mu[i : i + 2]  # track for this cone is just cone's mu

        self.track[index, 3] += 1  # increment cone's seen count
    
    # add new landmark to state
    def init_landmark(self, cone: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray]:
        new_x = self.mu[0] + cone[0] * cos(self.mu[2] + cone[1])  # add local xy to robot xy
        new_y = self.mu[1] + cone[0] * sin(self.mu[2] + cone[1])
        self.mu = np.append(self.mu, [new_x, new_y])  # append new landmark

        # landmark Jacobian
        Lz = np.array(
            [
                [cos(self.mu[2] + cone[1]), cone[0] * -sin(self.mu[2] + cone[1])],
                [sin(self.mu[2] + cone[1]), cone[0] * cos(self.mu[2] + cone[1])],
            ]
        )

        sig_len = len(self.Sigma)
        new_sig = np.zeros((sig_len + 2, sig_len + 2))  # create zeros
        new_sig[0:sig_len, 0:sig_len] = self.Sigma  # top left corner is existing sigma
        new_sig[sig_len : sig_len + 2, sig_len : sig_len + 2] = Lz @ self.Q @ Lz.T  # bottom right is new lm
        self.Sigma = new_sig

    # remove landmarks not seen for a number of frames and only behind the car
    def flush_map(self):
        # get heading and position vector
        heading = np.array([cos(self.mu[2]), sin(self.mu[2])])
        position = np.array([self.mu[0], self.mu[1]])

        # get the landmark position vectors
        # if the landmark is behind the robot, the dot product will be negative
        landmark_position_vectors = self.track[:, :2] - position
        dot_products = np.dot(landmark_position_vectors, heading)
        behind_idxs = np.where(dot_products < 0)[0]

        # get indexes of landmarks that have been seen for a number of frames
        noisy_idxs = np.where(self.track[:, 3] < self.in_frames)[0]

        # remove noisy and behind landmarks
        idxs_to_remove = np.concatenate((behind_idxs, noisy_idxs))
        unique, count = np.unique(idxs_to_remove, return_counts=True)  # gets unique indexes
        duplicated_idxs = unique[count > 1]  # only gets indexes that are duplicated (behind and noisy)

        if len(duplicated_idxs) > 0:
            # remove landmarks from mu and Sigma
            self.mu = np.delete(self.mu, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=0)

            # remove landmarks from Sigma
            self.Sigma = np.delete(self.Sigma, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=0)  # rows
            self.Sigma = np.delete(self.Sigma, [duplicated_idxs * 2 + 3, duplicated_idxs * 2 + 4], axis=1)  # columns

            # remove landmarks from track
            self.track = np.delete(self.track, duplicated_idxs, axis=0)

    # get cones within view of the car
    def get_local_map(self) -> np.ndarray:
        # global to local
        local_xs = (self.track[:, 0] - self.mu[0]) * cos(self.mu[2]) \
            + (self.track[:, 1] - self.mu[1]) * sin(self.mu[2])
        local_ys = -(self.track[:, 0] - self.mu[0]) * sin(self.mu[2]) \
            + (self.track[:, 1] - self.mu[1]) * cos(self.mu[2])

        local_track = np.stack((local_xs, local_ys, self.track[:, 2], self.track[:, 3]), axis=1)

        # get any cones that are within -10m to 10m beside car
        side_idxs = np.where(np.logical_and(local_ys > -10, local_ys < 10))[0]
        # get any cones that are within 10m in front of car
        forward_idxs = np.where(np.logical_and(local_xs > 0, local_xs < 10))[0]

        # combine indexes
        idxs = np.intersect1d(side_idxs, forward_idxs)

        # get local map
        return local_track[idxs]


def main(args=None):
    rclpy.init(args=args)
    node = EKFSlam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
