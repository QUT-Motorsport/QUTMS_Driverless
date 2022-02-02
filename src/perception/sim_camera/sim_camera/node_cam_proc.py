# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge # package to convert between ROS and OpenCV Images
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
# importcustom sim data message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# import other python libraries
import numpy as np
import time
from typing import List, Tuple, Callable

# import helper image processing module
from .img_proc import *
# import helper cone location processing module
from .depth_proc import *

cv_bridge = CvBridge()


def cone_msg(
    # distance: float,
    # bearing: float,
    # colour: int,  # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
) -> Cone:

    location = Point(
        x=10.0,
        y=5.0,
        z=0.0,
    )

    return Cone(
        location=location,
        color=1,
    )


class CamProcessing(Node):
    def __init__(self):
        super().__init__('camera_processing')

        # subscriber
        cam_subs = self.create_subscription(
            Image,
            # '/fsds/cam1', # 785x785 (square defaults)
            '/fsds/cam2', # 1080p (like a usual camera)
            self.callback,
            10)
        cam_subs  # prevent unused variable warning

        # publishers
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "sim_camera/cone_detection", 1)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "sim_camera/debug_img", 1)

        self.get_logger().info(f"Initialised Detector Node with mode: SIMULATOR")


    # callback function for camera image processing
    def callback(self, cam_msg: Image):
        logger = self.get_logger()
        logger.info("Received image")

        start: float = time.time() # begin a timer

        # get image
        raw_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(cam_msg)
        h, w, _ = raw_frame.shape

        # cones = cam_main(raw_frame, False)

        # # call return_locations to return xyzc for every cone
        # cone_coords = return_locations(w/2, cones) # send cones for distance calculation
        
        detected_cones: List[Cone] = []

        for i in range(10):
            detected_cones.append(cone_msg())
       
        detection_msg = ConeDetectionStamped(
            header=cam_msg.header,
            cones=detected_cones,
        )

        self.detection_publisher.publish(detection_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(raw_frame))
        
        logger.info("Time: " + str(time.time() - start)) # log time

        cv2.imshow("frame", raw_frame)
        cv2.waitKey(1)


## main call
def main(args=None):
    rclpy.init(args=args)

    node = CamProcessing()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
