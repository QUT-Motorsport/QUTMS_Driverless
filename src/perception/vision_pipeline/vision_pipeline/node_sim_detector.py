# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
import message_filters
from ament_index_python.packages import get_package_share_directory
# import ROS2 message libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
# translate ROS image messages to OpenCV
cv_bridge = CvBridge()
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python libraries
import os
from math import sin, cos, radians, isnan, isinf
import cv2
import numpy as np
from typing import List, Tuple, Callable
import time
import enum

# import required sub modules
from .rect import Rect, draw_box

CAMERA_FOV = 110  # degrees

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

# display_colour
CONE_DISPLAY_PARAMETERS = [
    BLUE_DISP_COLOUR,
    YELLOW_DISP_COLOUR,
    # ORANGE_DISP_COLOUR,
]

ConeMsgColour = int # define arbitrary variable type


def cone_distance(
    colour_frame_cone_bounding_box: Rect,
    depth_frame: np.ndarray,
) -> float:
    # get center as roi
    depth_roi: np.ndarray = Rect(
        x=colour_frame_cone_bounding_box.center.x - 3,
        y=colour_frame_cone_bounding_box.center.y - 5,
        width=6,
        height=6,
    ).as_roi(depth_frame)
    
    # filter out nans
    depth_roi = depth_roi[~np.isnan(depth_roi) & ~np.isinf(depth_roi)]

    depth_frame[colour_frame_cone_bounding_box.center.x][colour_frame_cone_bounding_box.center.x]

    return np.mean(depth_roi)


def cone_bearing(
    colour_frame_cone_bounding_box: Rect,
    width: int,
) -> float:

    cone_center = colour_frame_cone_bounding_box.center.x
    frame_width = width
    center_scaled = (frame_width / 2 - cone_center) / (frame_width / 2)  # 1 to -1 left to right
    
    return CAMERA_FOV/2 * center_scaled


def cone_msg(
    distance: float,
    bearing: float,
    colour: int,  # {Cone.YELLOW, Cone.BLUE, Cone.ORANGE_SMALL}
) -> Cone:

    location = Point(
        x=distance*cos(radians(bearing)),
        y=distance*sin(radians(bearing)),
        z=0.0,
    )

    return Cone(
        location=location,
        color=colour,
    )


class ModeEnum(enum.Enum):
    cv_thresholding = 0
    torch_inference = 1
    trt_inference = 2

class DetectorNode(Node):
    def __init__(
        self, 
        mode: ModeEnum, # mode of detection. 0==cv2, 1==torch, 2==trt
        get_bounding_boxes_callable: Callable[[np.ndarray], List[Tuple[Rect, ConeMsgColour, Colour]]],
        enable_cv_filters: bool = False
    ):
        super().__init__("cone_detector")

        # subscribers
        self.create_subscription(Image, "/fsds/camera/image_rect_color", self.img_callback, 10)
        self.create_subscription(Image, "/fsds/camera/depth_registered", self.depth_callback, 10)

        # publishers
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/detector/cone_detection", 1)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "/detector/debug_img", 1)

        # set which cone detection this will be using
        self.get_logger().info("Selected detection mode. 0==cv2, 1==torch, 2==trt")
        self.get_logger().info(f"Initialised Detector Node with mode: {mode.name}")
        self.enable_cv_filters = enable_cv_filters
        self.get_bounding_boxes_callable = get_bounding_boxes_callable
        self.depth_frame: np.ndarray = np.zeros([360,640])


    def depth_callback(self, depth_msg: Image):
        self.depth_frame = cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')


    def img_callback(self, colour_msg: Image):
        logger = self.get_logger()
        logger.info("Received image")

        start: float = time.time() # begin a timer

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgra8')
        depth_frame = self.depth_frame
        
        height, width, _ = colour_frame.shape
        logger.info(str(colour_frame.shape) + str(depth_frame.shape))

        detected_cones: List[Cone] = []
        for bounding_box, cone_colour, display_colour in self.get_bounding_boxes_callable(colour_frame):
            if self.enable_cv_filters:
                # filter by height
                if bounding_box.tl.y < height/2:
                    continue
                # filter on area
                if bounding_box.area < 100 or bounding_box.area > 8000: 
                    continue
                # filter by aspect ratio
                if bounding_box.aspect_ratio > 1.2:
                    continue
            
            logger.info(str(bounding_box.center.y) + str(bounding_box.center.x))
            distance = depth_frame[bounding_box.center.y][bounding_box.center.x]
            # filter on distance
            if isnan(distance) or isinf(distance):
                continue

            bearing = cone_bearing(bounding_box, width)
            detected_cones.append(cone_msg(distance, bearing, cone_colour))
            draw_box(colour_frame, box=bounding_box, colour=display_colour, distance=distance)

        detection_msg = ConeDetectionStamped(
            header=colour_msg.header,
            cones=detected_cones,
        )

        self.detection_publisher.publish(detection_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(colour_frame, encoding="bgra8"))

        logger.info("Time: " + str(time.time() - start)) # log time


## OpenCV thresholding
def main_cv2(args=None):
    from .threshold import Threshold
    from .hsv_cv import get_coloured_bounding_boxes

    # HSV threshold constants
    YELLOW_HSV_THRESH = Threshold(lower=[0, 0, 140], upper=[70, 100, 255])
    BLUE_HSV_THRESH = Threshold(lower=[100, 100, 70], upper=[130, 255, 255])
    ORANGE_HSV_THRESH = Threshold(lower=[0, 100, 50], upper=[15, 255, 255])

    # thresh, cone_colour, display_colour
    HSV_CONE_DETECTION_PARAMETERS = [
        (BLUE_HSV_THRESH, Cone.BLUE, BLUE_DISP_COLOUR),
        (YELLOW_HSV_THRESH, Cone.YELLOW, YELLOW_DISP_COLOUR),
        (ORANGE_HSV_THRESH, Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
    ]

    def get_hsv_bounding_boxes(colour_frame: np.ndarray) -> List[Tuple[Rect, ConeMsgColour, Colour]]:  # bbox, msg colour, display colour
        hsv_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2HSV)
        
        bounding_boxes: List[Tuple[Rect, ConeMsgColour, Colour]] = []
        for thresh, cone_colour, display_colour in HSV_CONE_DETECTION_PARAMETERS:
            for bounding_box in get_coloured_bounding_boxes(hsv_frame, thresh):
                bounding_boxes.append((bounding_box, cone_colour, display_colour))
        return bounding_boxes

    rclpy.init(args=args)
    detector_node = DetectorNode(ModeEnum.cv_thresholding, get_hsv_bounding_boxes, enable_cv_filters=True)
    rclpy.spin(detector_node)
    rclpy.shutdown()


## PyTorch inference
def main_torch(args=None):
    from .torch_inference import torch_init, infer
    
    # loading Pytorch model
    MODEL_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "models", "YBV2.pt")
    REPO_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "yolov5")
    CONFIDENCE = 0.35 # higher = tighter filter 
    model = torch_init(CONFIDENCE, MODEL_PATH, REPO_PATH)

    def get_torch_bounding_boxes(colour_frame: np.ndarray) -> List[Tuple[Rect, ConeMsgColour, Colour]]:  # bbox, msg colour, display colour
        bounding_boxes: List[Tuple[Rect, ConeMsgColour, Colour]] = []
        data = infer(colour_frame, model)
        
        for i in range(len(data.index)): 
            cone_colour = int(data.iloc[i, 5]) # locates object i, class ID at index 5
            bounding_box = Rect(
                int(data.xmin[i]),
                int(data.ymin[i]),
                int(data.xmax[i]-data.xmin[i]),
                int(data.ymax[i]-data.ymin[i]),
            )
            bounding_boxes.append((bounding_box, cone_colour, CONE_DISPLAY_PARAMETERS[cone_colour]))
        return bounding_boxes

    rclpy.init(args=args)
    detector_node = DetectorNode(ModeEnum.torch_inference, get_torch_bounding_boxes)
    rclpy.spin(detector_node)
    rclpy.shutdown()
