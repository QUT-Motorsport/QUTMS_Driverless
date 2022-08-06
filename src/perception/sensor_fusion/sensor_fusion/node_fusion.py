# import ROS2 libraries
import time

import numpy as np
from transforms3d.euler import quat2euler

import message_filters
import rclpy
from rclpy.clock import ClockType
import rclpy.logging
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Duration, Time

# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped
from nav_msgs.msg import Odometry

# import ROS2 message libraries
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .kdtree import KDNode
from .kdtree import Node as kdNode
from .kdtree import create

# import required sub modules
from .point import PointWithCov

# other python modules
from typing import List, Tuple

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


class ConeFusion(Node):
    def __init__(self):
        super().__init__("cone_fusion")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.lidarCallback, 1)
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.visionCallback, 1)
        # fused cone location publisher
        self.filtered_cones_pub: Publisher = self.create_publisher(ConeDetectionStamped, "/fusion/cone_detection", 1)
        self.filtered_local_pub: Publisher = self.create_publisher(
            ConeDetectionStamped, "/fusion/loc_cone_detection", 1
        )
        # rviz marker pubs
        self.lidar_markers: Publisher = self.create_publisher(MarkerArray, "/fusion/lidar_marker", 1)
        self.vision_markers: Publisher = self.create_publisher(MarkerArray, "/fusion/vision_marker", 1)
        self.filtered_markers: Publisher = self.create_publisher(MarkerArray, "/fusion/filtered_marker", 1)
        # odom sub
        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        # odom msg cache
        # needs to be the more than the max latency of perception in ms
        self.actualodom = message_filters.Cache(odom_sub, 1000)

        self.printmarkers: bool = True  # this could be a ros-arg

        self.conesKDTree: KDNode = None
        self.bufferKDTree: KDNode = None

        self.get_logger().info("---Cone Fusion Node Initalised---")

    def getNearestOdom(self, stamp: Time) -> Tuple[Odometry, np.ndarray]:
        # need to switch this over to a position from a EKF with covariance
        # have to do the time stuff this way because the compating of time in the message_filters __init__.py is wack
        locodom: Odometry = self.actualodom.getElemAfterTime(Time.from_msg(stamp))
        # standin covariance for ekf assuming the variance is sigma = 5cm with no covariance
        cov: np.ndarray = np.identity(3) * 0.0025

        # if the nearest Odom in the cache is more than 0.05 sec off then just throw it away
        if locodom is not None:
            # print(Time.from_msg(stamp) - Time.from_msg(locodom.header.stamp))
            if Time.from_msg(stamp) - Time.from_msg(locodom.header.stamp) > Duration(nanoseconds=0.05 * (10**9)):
                return None, cov
        return locodom, cov

    def fuseCone(self, point: PointWithCov):
        # find the closest cone in the cone tree
        closestcone = self.conesKDTree.search_knn(point, 1)
        # if its close enough to a actual cone than fuse it and rebalance in case it moved a bit too much
        # (should only really matter for the orange cones near the start and may not need to rebalance at this step but why not)
        # (im sure i will remove the rebalance to spare my cpu later and then the whole thing will break lol)
        if len(closestcone) > 0 and closestcone[0][0].data.inFourSigma(point):
            closestcone[0][0].data.update(point)
            self.conesKDTree.rebalance()
        # otherwise check it against the buffer
        else:
            self.bufferCone(point)

    def bufferCone(self, point: PointWithCov):
        # if we already have a list of possible cones then look through that tree for something close
        if self.bufferKDTree is not None and self.bufferKDTree.data is not None:
            # find the closest possible cone to the possible cone
            closestcone = self.bufferKDTree.search_knn(point, 1)
            # see if it is close enough for our liking (ths distance in m) than we will turn it into a actual cone
            if closestcone[0][0].data.inTwoSigma(point):
                # select the first group from the returned tuple (point objects) and then get the first one
                # (which will be our point since we only asked for one)
                pointnew: kdNode = closestcone[0][0]
                # fuse the points together
                pointnew.data.update(point)
                # if there is already a tree of Offical Cones tm than add it to that tree and then rebalance it
                if self.conesKDTree is not None:
                    self.conesKDTree.add(pointnew.data)
                    self.conesKDTree.rebalance()
                # if not than make a tree for them
                else:
                    self.conesKDTree = create([point])
                # remove the point from the buffer tree
                self.bufferKDTree.remove(pointnew.data)
                # the rebalance the buffer tree since we removed something
                self.bufferKDTree.rebalance()
            # if nothind is close in the buffer than add it to the buffer tree and rebalance
            else:
                self.bufferKDTree.add(point)
                self.bufferKDTree.rebalance()
        # if we dont already have a buffer tree than create one
        # (this should only ever happen once i hope, otherwise something has gone horrible wrong)
        else:
            self.bufferKDTree = create([point])

    def fusePoints(self, points: List[PointWithCov], header: Header):
        # if we have cones in the cone tree than check if we can fuse our new points with them
        if self.conesKDTree is not None:
            for point in points:
                self.fuseCone(point)
        # if not we will check them against the buffer (or create a buffer)
        # this should happen the tirst two cycles the fuse points is run
        # (first there wont be either tree and the second time there wont be a cone tree yet)
        else:
            for point in points:
                self.bufferCone(point)

        # if we have cones we are sure about than send them out in messages
        if self.conesKDTree is not None:
            # make a bool to determine if we are going to spend the time to generate markers
            rviz = self.printmarkers and self.filtered_markers.get_subscription_count() > 0
            # get all the cone elements from the tree structure
            curcones = self.conesKDTree.returnElements()

            # create the lists to fill with our elements
            cone_list: List[Cone] = []
            local_list: List[Cone] = []
            markers: List[Marker] = []
            msgid = 0
            for cone in curcones:
                # should probably put a filter for how many times a cone has actually been spotted
                if cone.covMax(0.5):
                    # create out messages to be published with the final cones that we found
                    pub_cone = Cone()
                    # create the row major 3x3 cov matrix for the message elements
                    pub_cone.covariance = cone.global_cov.flatten()
                    # set those parts of the message
                    pub_cone.color = cone.color
                    pub_cone.location.x = cone.global_x
                    pub_cone.location.y = cone.global_y
                    pub_cone.location.z = cone.global_z
                    # set its color
                    pub_cone.color = cone.color

                    # append those elements to the list of elements for the message
                    cone_list.append(pub_cone)

                    pub_cone.location.x = cone.loc_x
                    pub_cone.location.y = cone.loc_y
                    pub_cone.location.z = cone.loc_z
                    local_list.append(pub_cone)

                    if rviz:
                        # added clock stamp so they'd remove themselves after duration
                        cov_marker = cone.getCov(msgid, False)
                        cov_marker.header.stamp = self.get_clock().now().to_msg()
                        marker = cone.getMarker(msgid + 1)
                        marker.header.stamp = self.get_clock().now().to_msg()
                        markers.append(cov_marker)
                        markers.append(marker)
                    msgid += 2

            if rviz:
                marker_msg = MarkerArray()
                marker_msg.markers = markers
                self.filtered_markers.publish(marker_msg)

            # create a ConeDetectionStamped message
            coneListMsg = ConeDetectionStamped()
            coneListMsg.cones = cone_list
            coneListMsg.header = header
            # publish the messages
            self.filtered_cones_pub.publish(coneListMsg)
            # create a ConeDetectionStamped message
            coneLocalMsg = ConeDetectionStamped()
            coneLocalMsg.cones = local_list
            coneLocalMsg.header = header
            # publish the messages
            self.filtered_local_pub.publish(coneLocalMsg)

        # need to make a section to remove old points from the buffer
        if self.bufferKDTree is not None and self.bufferKDTree.data is not None:
            for point in self.bufferKDTree.returnElements():
                if self.get_clock().now() - Duration(nanoseconds=2 * 10**9) > Time.from_msg(header.stamp):
                    self.bufferKDTree.remove(point)

            self.bufferKDTree.rebalance()

        if self.conesKDTree is not None and self.conesKDTree.data is not None:
            for point in self.conesKDTree.returnElements():
                knn = self.conesKDTree.search_knn(point, 2)
                # print(point.global_z)
                if len(knn) > 1:
                    if point.inFourSigma(knn[1][0].data) and (
                        point.color == knn[1][0].data.color or (point.color == 4 or knn[1][0].data.color == 4)
                    ):

                        knn[1][0].data.update(point)
                        self.conesKDTree.remove(point)
                        self.conesKDTree.rebalance()

                    elif (point.nMeasurments > 3 and point.covMin(1)) or point.global_z < 0.2 or point.global_z > 0.7:
                        self.conesKDTree.remove(point)
                        self.conesKDTree.rebalance()
                if point.global_z < 0.1 or point.global_z > 0.1:
                    self.conesKDTree.remove(point)
                    self.conesKDTree.rebalance()

    def processConeData(self, msg: ConeDetectionStamped, method: str):
        # determines which callback this function was called from
        if method == "lidar":
            marker_topic = self.lidar_markers
        elif method == "vision":
            marker_topic = self.vision_markers

        # single header for message (all points read from same timestamp)
        header = msg.header

        if len(msg.cones) > 0:
            # determine if RVIZ markers will be published
            rviz: bool = self.printmarkers and marker_topic.get_subscription_count() > 0

            # pulls closest odom message + simulated cov from cached timestamps
            odomloc, odomcov = self.getNearestOdom(header.stamp)
            if odomloc is None:
                return None
            orientation_q = odomloc.pose.pose.orientation
            # r,p,y angles from quaternion
            roll, pitch, theta = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
            # placeholder shorter variable
            odom_x = odomloc.pose.pose.position.x
            odom_y = odomloc.pose.pose.position.y
            odom_z = odomloc.pose.pose.position.z

            conelist: List[PointWithCov] = []
            markers: List[Marker] = []
            msgid = 0
            for cone in msg.cones:
                # creates a 'point with covariance' object
                p = PointWithCov(
                    cone.location.x,
                    cone.location.y,
                    cone.location.z,
                    np.array(cone.covariance).reshape((3, 3)),
                    cone.color,
                )
                p.translate(odom_x, odom_y, odom_z, theta, odomcov)
                conelist.append(p)

                if rviz:
                    # added clock stamp so they'd remove themselves after duration
                    cov_marker = p.getCov(msgid, True)
                    cov_marker.header.stamp = self.get_clock().now().to_msg()
                    marker = p.getMarker(msgid + 1)
                    marker.header.stamp = self.get_clock().now().to_msg()
                    markers.append(cov_marker)
                    markers.append(marker)
                msgid += 2

            if rviz:
                marker_msg = MarkerArray()
                marker_msg.markers = markers
                marker_topic.publish(marker_msg)

            # now have list of possible cone points
            self.fusePoints(conelist, header)

    def lidarCallback(self, msg: ConeDetectionStamped):
        # common function
        start: float = time.time()
        self.processConeData(msg, "lidar")
        self.get_logger().info("Lidar fusion time: " + str(time.time() - start))

    def visionCallback(self, msg: ConeDetectionStamped):
        # common function
        start: float = time.time()
        self.processConeData(msg, "vision")
        self.get_logger().info("Vision fusion time: " + str(time.time() - start))


def main():
    # begin ros node
    rclpy.init()

    node = ConeFusion()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
