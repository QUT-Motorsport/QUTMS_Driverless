# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy.logging
import message_filters
# import ROS2 message libraries
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration as DurationMsg
# import custom message libraries
from driverless_msgs.msg import ConeDetectionStamped, Cone

# other python modules
from typing import List
import math
import numpy as np

from scipy.spatial import Delaunay

# import required sub modules
from .point import PointWithCov
from . import kdtree


class ConePipeline(Node):
    def __init__(self):
        super().__init__("cone_pipeline")

        self.logger = self.get_logger()

        self.create_subscription(ConeDetectionStamped, "/fusion/cone_detection_cov", self.mapCallback, 10) 

        self.track_markers: Publisher = self.create_publisher(MarkerArray, "/fusion/track_marker", 1)

        self.delaunayLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/delaunay_lines", 1)
        self.leftLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/left_line", 1)
        self.rightLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/right_line", 1)
        self.startLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/start_line", 1)
        self.delLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/del_line", 1)
        self.qsLinesVisualPub: Publisher = self.create_publisher(Marker, "/fusion/qs_line", 1)

        odom_sub = message_filters.Subscriber(self, Odometry, "/testing_only/odom")
        self.actualodom = message_filters.Cache(odom_sub, 1000) # needs to be the more than the max latency of perception in ms

        self.printmarkers: bool = True

        self.logger.debug("---Cone Fusion Node Initalised---")


    def mapCallback(self, track_msg: ConeDetectionStamped):
        delaunayLines = self.getDelaunayEdges(track_msg.cones)
        if delaunayLines:
            self.publishDelaunayEdgesVisual(delaunayLines)
            self.getEdges(delaunayLines)


    def orderLines(self, lines):
        usedLines = []
        orderedLines = []
        for i in range(len(lines)):
            line = lines[i]
            if line not in usedLines:
                pass


    def getEdges(self, delaunayLines):
        usedEdges = []
        leftHandEdges = []
        rightHandEdges = []
        discardedEdges = []
        startingLineEdges = []
        qsLineEdges = []
        for edge in delaunayLines:
            p1 = edge.p1
            p2 = edge.p2
            if p1.data.color == 0 and p2.data.color == 0:
                leftHandEdges.append(edge)
            elif p1.data.color == 1 and p2.data.color == 1:
                rightHandEdges.append(edge)
            elif p1.data.color == 2 or p1.data.color == 3 or p2.data.color == 2 or p2.data.color == 3:
                startingLineEdges.append(edge)
            elif p1.data.color == 4 or p2.data.color == 4:
                qsLineEdges.append(edge)
            else:
                discardedEdges.append(edge)


        leftHandMsg = self.trackEdgesVisual(leftHandEdges, 0)
        rightHandMsg = self.trackEdgesVisual(rightHandEdges, 1)
        discardedEdgesMsg = self.trackEdgesVisual(discardedEdges, 3)
        startingEdgesMsg = self.trackEdgesVisual(startingLineEdges, 2)
        qsEdgesMsg = self.trackEdgesVisual(qsLineEdges, 2)
        if leftHandMsg is not None:
            self.leftLinesVisualPub.publish(leftHandMsg)
        if rightHandMsg is not None:
            self.rightLinesVisualPub.publish(rightHandMsg)
        if discardedEdgesMsg is not None:
            self.delLinesVisualPub.publish(discardedEdgesMsg)
        if startingEdgesMsg is not None:
            self.startLinesVisualPub.publish(startingEdgesMsg)
        if qsEdgesMsg is not None:
            self.qsLinesVisualPub.publish(qsEdgesMsg)


    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return

        conePoints = np.zeros((len(frontCones), 2))
        coneList: List[PointWithCov] = []

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.position.x, cone.position.y])
            coneList.append(PointWithCov(0, 0, 0, None, cone.color, cone.header, cone.position.x, cone.position.y, cone.position.z, np.array(cone.covariance).reshape((3,3))))

        tri = Delaunay(conePoints)
        self.coneKDTree = kdtree.create(coneList)

        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                p1 = self.coneKDTree.search_knn(PointWithCov(0, 0, 0, None, 4, Header(), conePoints[simp[i]][0], conePoints[simp[i]][1], 0.0, None), 1)[0][0]
                p2 = self.coneKDTree.search_knn(PointWithCov(0, 0, 0, None, 4, Header(), conePoints[simp[j]][0], conePoints[simp[j]][1], 0.0, None), 1)[0][0]
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1], p1, p2)

                # add the line if its not already in the list and filter out if its longer than 7.5 m
                if edge not in delaunayEdges and edge.length() < 6:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def trackEdgesVisual(self, edges, cc: int):
        if not edges:
            return None

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.lifetime = DurationMsg(sec=1)
        marker.ns = "publishTrackLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.5
        if cc == 0:
            marker.color.b = 1.0
        elif cc == 1:
            marker.color.r = 1.0
            marker.color.g = 1.0
        elif cc == 2:
            marker.color.r = 1.0
            marker.color.g = 0.7
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        path_markers: List[Point] = []

        for edge in edges:
            # print edge

            p1 = Point()
            p1.x = edge.x1
            p1.y = edge.y1
            p1.z = 0.0
            p2 = Point()
            p2.x = edge.x2
            p2.y = edge.y2
            p2.z = 0.0

            path_markers.append(p1)
            path_markers.append(p2)

        marker.points = path_markers

        return marker

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.lifetime = DurationMsg(sec=1)
        marker.ns = "publishDelaunayLinesVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05

        marker.pose.orientation.w = 1.0

        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.b = 1.0

        path_markers: List[Point] = []

        for edge in edges:
            # print edge

            p1 = Point()
            p1.x = edge.x1
            p1.y = edge.y1
            p1.z = 0.0
            p2 = Point()
            p2.x = edge.x2
            p2.y = edge.y2
            p2.z = 0.0

            path_markers.append(p1)
            path_markers.append(p2)

        marker.points = path_markers

        self.delaunayLinesVisualPub.publish(marker)
        

class Edge():
    def __init__(self, x1, y1, x2, y2, p1, p2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None
        self.p1 = p1
        self.p2 = p2

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)


def main():
    # begin ros node
    rclpy.init()

    node = ConePipeline()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()
