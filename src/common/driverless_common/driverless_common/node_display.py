from colour import Color

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray

from driverless_common.common import QOS_LATEST
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
        super().__init__("display_node")

        # cone detection subscribers
        self.create_subscription(ConeDetectionStamped, "/vision/cone_detection", self.vision_callback, QOS_LATEST)
        self.create_subscription(ConeDetectionStamped, "/lidar/cone_detection", self.lidar_callback, QOS_LATEST)

        # track subs
        self.create_subscription(ConeDetectionStamped, "/slam/global_map", self.slam_callback, QOS_LATEST)
        self.create_subscription(ConeDetectionStamped, "/slam/local_map", self.local_callback, QOS_LATEST)

        # steering angle target sub
        self.create_subscription(AckermannDriveStamped, "/control/driving_command", self.steering_callback, QOS_LATEST)

        # cv2 rosboard image pubs
        # self.vision_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/vision_det_img", 1)
        # self.lidar_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/lidar_det_img", 1)
        self.slam_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/slam_image", 1)
        self.local_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/local_image", 1)

        # marker pubs
        self.vision_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/debug_markers/vision_markers", 1)
        self.lidar_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/debug_markers/lidar_markers", 1)
        self.slam_mkr_publisher: Publisher = self.create_publisher(MarkerArray, "/debug_markers/slam_with_cov", 1)

        self.get_logger().info("---Common visualisation node initialised---")

    def steering_callback(self, msg: AckermannDriveStamped):
        # get most recent driving command msg from a controller
        self.steering_angle = msg.drive.steering_angle
        self.velocity = msg.drive.speed

    def vision_callback(self, msg: ConeDetectionStamped):
        if self.vision_mkr_publisher.get_subscription_count() != 0:
            self.vision_mkr_publisher.publish(marker_array_from_cone_detection(msg))

        # if self.vision_img_publisher.get_subscription_count() == 0:
        #     return
        # # subscribed to vision cone detections
        # cones = []
        # for cone in msg.cones:
        #     new = cone
        #     new.location.x = cone.location.x - 0.37
        #     cones.append(new)
        # debug_img = draw_markers(msg.cones)  # draw cones on image
        # debug_img = draw_steering(
        #     debug_img, self.steering_angle, self.velocity
        # )  # draw steering angle and vel data on image
        # self.vision_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

    def lidar_callback(self, msg: ConeDetectionStamped):
        if self.lidar_mkr_publisher.get_subscription_count() != 0:
            self.lidar_mkr_publisher.publish(marker_array_from_cone_detection(msg))

        # if self.lidar_img_publisher.get_subscription_count() == 0:
        #     return
        # # subscribed to lidar cone detections
        # cones = []
        # for cone in msg.cones:
        #     new = cone
        #     new.location.x = cone.location.x + 1.59
        #     cones.append(new)

        # debug_img = draw_markers(cones)
        # debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        # self.lidar_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

    def slam_callback(self, msg: ConeDetectionStamped):
        if self.slam_mkr_publisher.get_subscription_count() != 0:
            self.slam_mkr_publisher.publish(marker_array_from_map(msg))

        if self.slam_img_publisher.get_subscription_count() == 0:
            return
        # subscribed to slam map
        map_img = draw_map(msg)
        self.slam_img_publisher.publish(cv_bridge.cv2_to_imgmsg(map_img, encoding="bgr8"))

    def local_callback(self, msg: ConeDetectionStamped):
        if self.local_img_publisher.get_subscription_count() == 0:
            return
        # subscribed to local cone detections
        debug_img = draw_markers(msg.cones)
        debug_img = draw_steering(debug_img, self.steering_angle, self.velocity)
        self.local_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))


def main():
    rclpy.init()
    node = DisplayDetections()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
