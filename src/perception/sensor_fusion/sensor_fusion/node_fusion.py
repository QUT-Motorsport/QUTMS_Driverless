# import ROS2 libraries
import imp
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
import rclpy.logging
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
# import custom message libraries
from driverless_msgs.msg import Cone
from driverless_msgs.msg import ConeDetectionStamped

# other python modules
from math import sqrt, atan2, pi, sin, cos, atan
import cv2
import numpy as np
from typing import Tuple, List, Optional
import sys
import os
import getopt
import pathlib
import time

# import required sub modules
from driverless_common.point import Point
from transforms3d.euler import quat2euler

from . import kmeans_clustering as km

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# image display geometry
SCALE = 20
WIDTH = 20*SCALE # 10m either side
HEIGHT = 20*SCALE # 20m forward
ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), HEIGHT)

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255) # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0) # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255) # bgr - orange
PURP_DISP_COLOUR: Colour = (230, 230, 250) # bgr - purp
GREY_DISP_COLOUR: Colour = (220, 220, 220) # bgr - grey

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(WIDTH/2 - y*SCALE)),
        int(round(HEIGHT - x*SCALE)),
    )


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def marker_msg(
    x_coord: float, 
    y_coord: float, 
    ID: int, 
) -> Marker: 
    """
    Creates a Marker object for cones or a car.
    * param x_coord: x position relative to parent frame
    * param y_coord: y position relative to parent frame
    * param ID: Unique for markers in the same frame
    * param header: passed in because creating time is dumb
    * return: Marker
    """

    marker = Marker()
    marker.ns = "current_path"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0 # alpha
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker


class ConeFusion(Node):
    def __init__(self):
        super().__init__("cone_fusion")

        # subscribers
        vision_cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/vision/cone_detection"
        )
        lidar_cones_sub = message_filters.Subscriber(
            self, ConeDetectionStamped, "/lidar/cone_detection"
        )
        synchronizer = message_filters.TimeSynchronizer(
            fs=[vision_cones_sub, lidar_cones_sub],
            queue_size=100,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.track_img_publisher: Publisher = self.create_publisher(Image, "/fusion/plot_img", 1)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/fusion/debug_img", 1)

        self.get_logger().info("---Cone detector fusion Node Initalised---")


    def callback(self, vision_msg: ConeDetectionStamped, lidar_msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.info("Received image")

        vision_cones: List[Cone] = vision_msg.cones
        lidar_cones: List[Cone] = lidar_msg.cones

        # create black image
        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        yellow_list: List[float] = []
        blue_list: List[float] = []
        unknown_list: List[float] = []
        for cone in vision_cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
                yellow_list.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
                blue_list.append([cone.location.x, cone.location.y])
            else:
                unknown_list.append([cone.location.x, cone.location.y])
                colour = (255, 255, 255)

            # draws location of cone w/ colour
            print(cone.location.x, cone.location.y)
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5
            )

        self.track_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))



    def update_kmeans(self, new_cones, odom_msg: Odometry):
        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        for cone in new_cones:
            x_dist = cone.location.x
            y_dist = cone.location.x
            a = cos(ak)
            b = sin(ak)
            c = x + x_dist*a - y_dist*b
            d = y + x_dist*b - y_dist*a

            ref_cone = Cone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = c
            ref_cone.location.y = d
            ref_cone.location.z = 0.0
            ref_cone.color = 3
            self.cones.append(ref_cone)
        raw_x, raw_y = [], []
        for cone in self.cones:
            raw_x.append(cone.location.x)
            raw_y.append(cone.location.y)
        if len(raw_x) < self.num_cones: #unclear if this is actually needed
            kms = km.kmeans_clustering(raw_x, raw_y, len(raw_x)-1)
        else:
            kms = km.kmeans_clustering(raw_x, raw_y, self.num_cones)
        self.clusters = kms.get_cent()

    def get_km_cones(self, odom_msg: Odometry):
        
        # header used to create markers
        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        ref_cones: List[Cone] = []
        for cone in self.cones:
            # displacement from car to cone
            x_dist = cone.location.x - x
            y_dist = cone.location.y - y

            ref_cone = Cone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = x_dist*cos(ak) + y_dist*sin(ak)
            ref_cone.location.y = y_dist*cos(ak) - x_dist*sin(ak)
            ref_cone.location.z = 0.0
            ref_cone.color = 3

            if ref_cone.location.x > 0 and ref_cone.location.x < self.max_range and ref_cone.location.y > -9 and ref_cone.location.y < 9:
                ref_cones.append(ref_cone)
        return ref_cones

    def get_clust_cones(self, odom_msg: Odometry):
        
        # header used to create markers
        self.odom_header = odom_msg.header

        w = odom_msg.pose.pose.orientation.w
        i = odom_msg.pose.pose.orientation.x
        j = odom_msg.pose.pose.orientation.y
        k = odom_msg.pose.pose.orientation.z

        # i, j, k angles in rad
        ai, aj, ak = quat2euler([w, i, j, k])

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        ref_cones: List[Cone] = []
        for cone in self.clusters:
            # displacement from car to cone
            x_dist = cone[0] - x
            y_dist = cone[1] - y

            ref_cone = Cone()
            # reference frame displacement with rotation 
            # uses k angle (z axis)
            ref_cone.location.x = x_dist*cos(ak) + y_dist*sin(ak)
            ref_cone.location.y = y_dist*cos(ak) - x_dist*sin(ak)
            ref_cone.location.z = 0.0
            ref_cone.color = 3

            if ref_cone.location.x > 0 and ref_cone.location.x < self.max_range and ref_cone.location.y > -9 and ref_cone.location.y < 9:
                ref_cones.append(ref_cone)
        return ref_cones


    def xcallback(self, lidar_cone_msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.info("Received detection")

        lidar_scan_time: Time = Time.from_msg(lidar_cone_msg.header.stamp)

        a: ConeDetectionStamped = self.actualcone.getElemAfterTime(lidar_scan_time)
        o: Odometry = self.actualodom.getElemAfterTime(lidar_scan_time)
        self.num_cones = 400
        print("it does something")
        #b = self.actualcone.getElemBeforeTime(lidar_scan_time)
        try: #if True:#
            cones: List[Cone] = a.cones
            lidar_cones: List[Cone] = lidar_cone_msg.cones
            # create black image
            debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


            if len(lidar_cones) > 0:
                self.update_kmeans(lidar_cones, o)

            for cone in cones:
                if cone.color == Cone.YELLOW:
                    colour = YELLOW_DISP_COLOUR
                elif cone.color == Cone.BLUE:
                    colour = BLUE_DISP_COLOUR
                else:
                    colour = (255, 255, 255)

                # draws location of cone w/ colour
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt((cone.location.x - 1.2), cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=3,
                    thickness=3
                )

            for cone in self.get_km_cones(o):
                colour = PURP_DISP_COLOUR
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=3,
                    thickness=3
                )

            for cone in self.get_clust_cones(o):
                colour = GREY_DISP_COLOUR
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=2,
                    thickness=2
                )

            for cone in lidar_cones:
                colour = ORANGE_DISP_COLOUR
                cv2.drawMarker(
                    debug_img, 
                    robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                    colour,
                    markerType=cv2.MARKER_SQUARE,
                    markerSize=3,
                    thickness=3
                )

            text_vel = f"Lidar N: {len(lidar_cones)}"
            cv2.putText(
                debug_img, text_vel, (10, HEIGHT-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
            )

            self.plot_track(o)

            self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        except Exception as e: print(e)


## initialise ROS2 logging system
def init_logs() -> List[str]:
    args = ['--ros-args']

    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    # defaults args
    print_mode = '--disable-stdout-logs'
    # processing args
    opts, arg = getopt.getopt(sys.argv[1:], str(), ['print', 'ros-args'])
    for opt, arg in opts:
        if opt == '--print':
            print_mode = '--enable-stdout-logs'

    args.append(print_mode)

    os.environ['ROS_LOG_DIR'] = f'{path}/logs/'
    os.environ.get('ROS_LOG_DIR')

    return args


def main():
    args = init_logs()

    # begin ros node
    rclpy.init(args=args)
    node = ConeFusion()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
