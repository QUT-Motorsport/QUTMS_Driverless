# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from sensor_msgs.msg import Image
# custom sim data message libraries
from qutms_msgs.msg import ConeScan
# OpenCV2 
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class Cam_Pipe(Node):
    def __init__(self):
        super().__init__('control')

         # creates subscriber to 'cam1' with type PointCloud2 that sends data to lidar_callback
        self.cam_subscription_ = self.create_subscription(
            Image,
            '/fsds/camera/cam1',
            self.cam_callback,
            10)
        self.cam_subscription_  # prevent unused variable warning

        self.br = CvBridge()

    
    def cam_callback(self, cam_msg):
        current_frame = self.br.imgmsg_to_cv2(cam_msg)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

## main call
def main(args=None):
    rclpy.init(args=args)

    node = Cam_Pipe()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
