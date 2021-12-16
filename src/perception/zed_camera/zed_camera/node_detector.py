# import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge
import message_filters
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
# import custom message libraries
from driverless_msgs.msg import Cone, ConeDetectionStamped

# other python libraries
from math import sin, cos, radians, isnan, isinf
import cv2
import numpy as np
from typing import List, Tuple

# import required sub scripts
from .threshold import Threshold
from .hsv_cv import get_coloured_bounding_boxes
from .rect import Rect, draw_box
from .yolo_model import yolov5_init


Colour = Tuple[int, int, int]
ConeMsgColour = int


cv_bridge = CvBridge()

CONFIDENCE = 0.30
MODEL_PATH = "/home/developer/driverless_ws/src/perception/zed_camera/models/YBV2.pt"
model = yolov5_init(CONFIDENCE, MODEL_PATH)


CAMERA_FOV = 110  # degrees


YELLOW_HSV_THRESH = Threshold(
    lower=[27, 160, 130],
    upper=[40, 255, 255],
)

BLUE_HSV_THRESH = Threshold(
    lower=[120, 100, 40],
    upper=[130, 255, 255],
)

ORANGE_HSV_THRESH = Threshold(
    lower=[0, 100, 50],
    upper=[15, 255, 255],
)


YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange


# thresh, cone_colour, display_colour
HSV_CONE_DETECTION_PARAMETERS = [
    (BLUE_HSV_THRESH, Cone.BLUE, BLUE_DISP_COLOUR),
    (YELLOW_HSV_THRESH, Cone.YELLOW, YELLOW_DISP_COLOUR),
    # (ORANGE_HSV_THRESH, Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
]

YOLO_CONE_DETECTION_PARAMETERS = [
    (Cone.BLUE, BLUE_DISP_COLOUR),
    (Cone.YELLOW, YELLOW_DISP_COLOUR),
    (Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
]


def get_hsv_bounding_boxes(colour_frame: np.ndarray) -> List[Tuple[Rect, ConeMsgColour, Colour]]:  # bbox, msg colour, display colour
    hsv_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2HSV)
    
    bounding_boxes = []
    for thresh, cone_colour, display_colour in HSV_CONE_DETECTION_PARAMETERS:
        for bounding_box in get_coloured_bounding_boxes(hsv_frame, [thresh]):
            bounding_boxes.append((bounding_box, cone_colour, display_colour))
    return bounding_boxes


CLASS = 5

def get_yolo_bounding_boxes(colour_frame: np.ndarray) -> List[Tuple[Rect, ConeMsgColour, Colour]]:  # bbox, msg colour, display colour
    rgb_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2RGB)

    bounding_boxes = []
    results = model(rgb_frame)
    data = results.pandas().xyxy[0]

    for cone_colour, display_colour in YOLO_CONE_DETECTION_PARAMETERS:
        for i in range(len(data.index)): 
            if data.iloc[i, CLASS] == cone_colour:
                bounding_box = Rect(
                    int(data.xmin[i]),
                    int(data.ymin[i]),
                    int(data.xmax[i]-data.xmin[i]),
                    int(data.ymax[i]-data.ymin[i]),
                )
                bounding_boxes.append((bounding_box, cone_colour, display_colour))
    return bounding_boxes


def cone_distance(
    colour_frame_cone_bounding_box: Rect,
    depth_frame: np.ndarray,
) -> float:
    # get center as roi
    depth_roi = Rect(
        x=colour_frame_cone_bounding_box.center.x - 3,
        y=colour_frame_cone_bounding_box.center.y - 3,
        width=6,
        height=6,
    ).as_roi(depth_frame)
    
    # filter out nans
    depth_roi = depth_roi[~np.isnan(depth_roi) & ~np.isinf(depth_roi)]

    return np.mean(depth_roi)


def cone_bearing(
    colour_frame_cone_bounding_box: Rect,
    colour_frame_camera_info: CameraInfo,
) -> float:
    cone_center = colour_frame_cone_bounding_box.center.x
    frame_width = colour_frame_camera_info.width
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


class DetectorNode(Node):
    def __init__(self):
        super().__init__("zed_detector")

        # subscribers
        colour_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/rgb/image_rect_color"
        )
        colour_camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/zed2i/zed_node/rgb/camera_info"
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/depth/depth_registered"
        )

        synchronizer = message_filters.TimeSynchronizer(
            fs=[colour_sub, colour_camera_info_sub, depth_sub],
            queue_size=30,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "zed_detector/cone_detection", 1)
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "zed_detector/debug_img", 1)

        self.get_logger().info("ZED Detector Node Initalised")


    def callback(self, colour_msg: Image, colour_camera_info_msg: CameraInfo, depth_msg: Image):
        logger = self.get_logger()
        logger.info("Recieved image")

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg)
        depth_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        detected_cones: List[Cone] = []
        # for bounding_box, cone_colour, display_colour in get_hsv_bounding_boxes(colour_frame):
        for bounding_box, cone_colour, display_colour in get_yolo_bounding_boxes(colour_frame):
            
            # filter by height
            if bounding_box.tl.y < colour_camera_info_msg.height/2:
                continue

            # filter on area
            if bounding_box.area < 100 or bounding_box.area > 8000: 
                continue
            
            # filter by aspect ratio
            if bounding_box.aspect_ratio > 1.2:
                continue
            
            distance = cone_distance(bounding_box, depth_frame)

            # filter on distance
            if isnan(distance) or isinf(distance):
                continue

            bearing = cone_bearing(bounding_box, colour_camera_info_msg)
            detected_cones.append(cone_msg(distance, bearing, cone_colour))
            draw_box(colour_frame, box=bounding_box, colour=display_colour, distance=distance)

        detection_msg = ConeDetectionStamped(
            header=colour_msg.header,
            cones=detected_cones,
        )

        cv2.imshow('debug', colour_frame)
        cv2.waitKey(1)

        self.detection_publisher.publish(detection_msg)
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(colour_frame, encoding="bgra8"))


def main(args=None):
    rclpy.init(args=args)

    detector_node = DetectorNode()

    rclpy.spin(detector_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
