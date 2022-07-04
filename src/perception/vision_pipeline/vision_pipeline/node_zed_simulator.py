# import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from rclpy.publisher import Publisher

# other python libraries
import cv2
import numpy as np

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()


class ZedNode(Node):
    def __init__(self):
        super().__init__("zed_simulator")

        self.create_subscription(Image, "/fsds/camera/depth_registered", self.depth_callback, 10)
        self.create_subscription(Image, "/fsds/camera/image_rect_color", self.rgb_callback, 10)

        self.depth_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/depth/depth_registered", 1)
        self.rgb_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/rgb/image_rect_color", 1)
        self.rgb_img_info_publisher: Publisher = self.create_publisher(CameraInfo, "/zed2i/zed_node/rgb/camera_info", 1)
        
        self.camera_info = CameraInfo()
        self.camera_info.height = 360
        self.camera_info.width = 640

        self.depth_msg = Image()

        self.get_logger().info("Initialised ZED camera simulator node")
    

    def depth_callback(self, depth_msg: Image):
        depth_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        # for some reason, depth is 2x in the depth camera
        new_depth = np.divide(depth_frame, 2)
        self.depth_msg = cv_bridge.cv2_to_imgmsg(new_depth, encoding="32FC1")
        
    
    def rgb_callback(self, colour_msg: Image):
        self.camera_info.header = colour_msg.header
        self.depth_msg.header.stamp = colour_msg.header.stamp

        self.rgb_img_info_publisher.publish(self.camera_info)
        self.rgb_img_publisher.publish(colour_msg)
        self.depth_img_publisher.publish(self.depth_msg)
        self.get_logger().info("Published RBG, Info, Depth")


def main(args=None):
    rclpy.init(args=args)

    annotator_node = ZedNode()

    rclpy.spin(annotator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
