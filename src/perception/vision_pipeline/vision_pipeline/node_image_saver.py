import cv2

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from driverless_common.common import QOS_ALL


class BagToImageFile(Node):
    bridge = CvBridge()
    count = 0
    path = "bags/"

    def __init__(self):
        super().__init__("bag_to_image_file_node")
        self.subscription = self.create_subscription(
            Image, "/zed2i/zed_node/rgb/image_rect_color", self.listener_callback, QOS_ALL
        )

        # create a folder for the images
        print("Node has been initialized")

    def listener_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = self.path + str(self.count) + ".jpg"
            cv2.imwrite(filename, image)
            self.count += 1

        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)
    node = BagToImageFile()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
