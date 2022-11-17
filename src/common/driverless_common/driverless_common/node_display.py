from colour import Color

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped, PathStamped, TrackDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from driverless_common.draw import draw_map, draw_markers, draw_steering
from driverless_common.marker import marker_array_from_cone_detection, marker_array_from_map, path_marker_msg

from typing import List

cv_bridge = CvBridge()  # translate ROS image messages to OpenCV

red = Color("red")
blue = Color("blue")
col_range = list(blue.range_to(red, 100))


class DisplayDetections(Node):
    steering_angle: float = 0.0
    velocity: float = 0.0

    def __init__(self):
        super().__init__("display_detections")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.vision_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.lidar_callback, 1)
        self.create_subscription(ConeDetectionStamped, "/sim/cone_detection", self.sim_cones_callback, 1)

        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection2", self.vision_callback2, 1)

        # steering angle target sub
        self.create_subscription(AckermannDrive, "/driving_command", self.steering_callback, 1)

        # path spline sub
        self.create_subscription(PathStamped, "/planner/path", self.path_callback, 1)

        # track subs
        self.create_subscription(TrackDetectionStamped, "/sim/track", self.sim_track_callback, 1)
        self.create_subscription(TrackDetectionStamped, "/slam/track", self.slam_callback, 1)
        self.create_subscription(TrackDetectionStamped, "/slam/local", self.local_callback, 1)

        # cv2 rosboard image pubs
        self.vision_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/vision_det_img", 1)
        self.lidar_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/lidar_det_img", 1)
        self.sim_cones_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/sim_cones_det_img", 1)

        self.sim_track_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/sim_track_image", 1)
        self.slam_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/slam_image", 1)
        self.local_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/local_image", 1)

        # rviz marker pubs
        self.vision_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/vision_markers", 1)
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/lidar_markers", 1)
        self.sim_cones_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/sim_cones_markers", 1)

        self.sim_track_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/sim_track", 1)
        self.slam_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/markers/slam_with_cov", 1)

        self.vision_img_publisher2: Publisher = self.create_publisher(Image, "/debug_imgs/vision_det_img2", 1)
        self.vision_mkr_publisher2: Publisher = self.create_publisher(MarkerArray, "/markers/vision_markers2", 1)

        # path marker pubs
        self.path_marker_publisher: Publisher = self.create_publisher(Marker, "/markers/path_line", 1)

        self.get_logger().info("---Cone display node initialised---")

    def steering_callback(self, msg: AckermannDrive):
        # get most recent driving command msg from a controller
        self.steering_angle = msg.steering_angle
        self.velocity = msg.speed

    def vision_callback(self, msg: ConeDetectionStamped):
        # subscribed to vision cone detections
        debug_img = draw_markers(msg.cones)  # draw cones on image
        debug_img = draw_steering(
            debug_img, self.steering_angle, self.velocity
        )  # draw steering angle and vel data on image
        self.vision_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.vision_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def vision_callback2(self, msg: ConeDetectionStamped):
        # subscribed to vision cone detections
        debug_img = draw_markers(msg.cones)  # draw cones on image
        debug_img = draw_steering(
            debug_img, self.steering_angle, self.velocity
        )  # draw steering angle and vel data on image
        self.vision_img_publisher2.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.vision_mkr_publisher2.publish(marker_array_from_cone_detection(msg))

    def lidar_callback(self, msg: ConeDetectionStamped):
        # subscribed to lidar cone detections
        debug_img = draw_markers(msg.cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.lidar_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.lidar_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def sim_cones_callback(self, msg: ConeDetectionStamped):
        # subscribed to sim cone detections
        debug_img = draw_markers(msg.cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.sim_cones_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

        self.sim_cones_mkr_publisher.publish(marker_array_from_cone_detection(msg))

    def path_callback(self, msg: PathStamped):
        # subscribed to a desired path
        path_markers: List[Point] = []
        path_colours: List[ColorRGBA] = []

        for point in msg.path:
            # set colour proportional to angle
            try:
                col = col_range[round(point.turn_intensity)].get_rgb()

                line_point = point.location
                line_colour = ColorRGBA()
                line_colour.a = 1.0  # alpha
                line_colour.r = col[0]
                line_colour.g = col[1]
                line_colour.b = col[2]
                path_markers.append(line_point)
                path_colours.append(line_colour)

            except:
                continue

        self.path_marker_publisher.publish(path_marker_msg(path_markers, path_colours))

    def sim_track_callback(self, msg: TrackDetectionStamped):
        # subscribed to sim map
        map_img = draw_map(msg)
        self.sim_track_img_publisher.publish(cv_bridge.cv2_to_imgmsg(map_img, encoding="bgr8"))
        self.sim_track_mkr_publisher.publish(marker_array_from_map(msg, ground_truth=True))

    def slam_callback(self, msg: TrackDetectionStamped):
        # subscribed to slam map
        map_img = draw_map(msg)
        self.slam_img_publisher.publish(cv_bridge.cv2_to_imgmsg(map_img, encoding="bgr8"))
        self.slam_mkr_publisher.publish(marker_array_from_map(msg))

    def local_callback(self, msg: TrackDetectionStamped):
        # subscribed to sim cone detections
        cones: List[Cone] = [c.cone for c in msg.cones]
        debug_img = draw_markers(cones)
        self.local_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
