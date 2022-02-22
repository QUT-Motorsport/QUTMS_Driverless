# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from rclpy.publisher import Publisher

class ZedNode(Node):
    def __init__(self):
        super().__init__("cone_annotator")

        self.create_subscription(Image, "/fsds/camera/depth_registered", self.depth_callback, 10)
        self.create_subscription(Image, "/fsds/camera/image_rect_color", self.rgb_callback, 10)

        self.depth_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/depth/depth_registered", 1)
        self.rgb_img_publisher: Publisher = self.create_publisher(Image, "/zed2i/zed_node/rgb/image_rect_color", 1)
        self.rgb_img_info_publisher: Publisher = self.create_publisher(CameraInfo, "/zed2i/zed_node/rgb/camera_info", 1)
        
        self.camera_info = CameraInfo()
        self.camera_info.height = 640
        self.camera_info.width = 360

        self.depth_msg = DisparityImage()

        self.get_logger().info("Initialised ZED camera simulator node")
    

    def depth_callback(self, depth_msg: Image):
        self.depth_msg = depth_msg
    
    def rgb_callback(self, colour_msg: Image):
        self.camera_info.header = colour_msg.header
        self.depth_msg.header.stamp = colour_msg.header.stamp

        self.rgb_img_info_publisher.publish(self.camera_info)
        self.rgb_img_publisher.publish(colour_msg)
        self.depth_img_publisher.publish(self.depth_msg)



def main(args=None):
    rclpy.init(args=args)

    annotator_node = ZedNode()

    rclpy.spin(annotator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
