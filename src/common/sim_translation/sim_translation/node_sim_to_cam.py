import cv2
import numpy as np

from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import CameraInfo, Image

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()


class CamSimulator(Node):
    def __init__(self):
        super().__init__("cam_simulator")

        # sync sub to depth and camera
        colour_sub = message_filters.Subscriber(self, Image, "/fsds/camera/image_rect_color")
        depth_sub = message_filters.Subscriber(self, Image, "/fsds/camera/depth_registered")
        synchronizer = message_filters.ApproximateTimeSynchronizer(fs=[colour_sub, depth_sub], queue_size=20, slop=0.1)
        synchronizer.registerCallback(self.callback)

        self.depth_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/depth/depth_registered", 1)
        self.rgb_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/rgb/image_rect_color", 1)
        self.rgb_img_info_publisher: Publisher = self.create_publisher(CameraInfo, "/zed2i/zed_node/rgb/camera_info", 1)

        self.get_logger().info("---Initialised ZED camera simulator node---")

    def callback(self, colour_msg: Image, depth_msg: Image):
        self.get_logger().debug("Received sim camera data")

        depth_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        new_depth = np.divide(depth_frame, 2)  # for some reason, depth is 2x in the depth camera
        depth_msg = cv_bridge.cv2_to_imgmsg(new_depth, encoding="32FC1")
        depth_msg.header.stamp = colour_msg.header.stamp

        camera_info = CameraInfo()
        camera_info.height = 360
        camera_info.width = 640
        camera_info.header = colour_msg.header

        self.depth_publisher.publish(depth_msg)
        self.rgb_img_info_publisher.publish(camera_info)
        self.rgb_img_publisher.publish(colour_msg)

        self.get_logger().debug("Published RBG, Info, Depth")


def main(args=None):
    rclpy.init(args=args)
    node = CamSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
