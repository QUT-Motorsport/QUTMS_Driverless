# QUT Motorsport - Driverless

# A node that publishes a test image over the 'Image' topic

# Author: Lachlan Masson

## Importing libraries
# ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
# ROS2 message libraries
from sensor_msgs.msg import Image
# other python libraries
import cv2
import numpy as np


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(
            Image, 
            'Image', 
            10)
        # creates timer for publishing image
        timer_period = 10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cv_image= cv2.imread('/home/lachie/Desktop/filteredImagePINK.jpg')
        self.bridge = CvBridge()

    def timer_callback(self):
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))

        self.get_logger().info('Publishing a test image for the "detectCones" node!')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()