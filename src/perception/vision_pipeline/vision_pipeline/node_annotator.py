# import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import message_filters
from ament_index_python.packages import get_package_share_directory
# import ROS2 message libraries
from sensor_msgs.msg import Image, CameraInfo
# import custom message libraries
from driverless_msgs.msg import Cone

# other python libraries
import os
import cv2
import numpy as np
from typing import List, Tuple

# import required sub modules
from .rect import Rect, draw_box
from .torch_inference import torch_init

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

CAMERA_FOV = 110  # degrees

# loading Pytorch model
MODEL_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "models", "YBV2.pt")
REPO_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "yolov5")
CONFIDENCE = 0.40 # higher = tighter filter 
model = torch_init(CONFIDENCE, MODEL_PATH, REPO_PATH)

ANNOTATION_PATH = "datasets/annotations/"
VALIDATION_PATH = "datasets/validation/"

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

# cone_colour, display_colour
YOLO_CONE_DETECTION_PARAMETERS = [
    (Cone.BLUE, BLUE_DISP_COLOUR),
    (Cone.YELLOW, YELLOW_DISP_COLOUR),
    (Cone.ORANGE_SMALL, ORANGE_DISP_COLOUR),
]

ConeMsgColour = int # define arbitrary variable type


def get_yolo_bounding_boxes(colour_frame: np.ndarray) -> List[Tuple[Rect, ConeMsgColour, Colour]]:  # bbox, msg colour, display colour
    rgb_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2RGB)

    bounding_boxes: List[Tuple[Rect, ConeMsgColour, Colour]] = []
    results = model(rgb_frame)
    data = results.pandas().xyxy[0]

    for cone_colour, display_colour in YOLO_CONE_DETECTION_PARAMETERS:
        for i in range(len(data.index)): 
            if data.iloc[i, 5] == cone_colour: # locates object i, class ID at index 5
                bounding_box = Rect(
                    int(data.xmin[i]),
                    int(data.ymin[i]),
                    int(data.xmax[i]-data.xmin[i]),
                    int(data.ymax[i]-data.ymin[i]),
                )
                bounding_boxes.append((bounding_box, cone_colour, display_colour))
    
    return bounding_boxes


class AnnotatorNode(Node):
    def __init__(self):
        super().__init__("cone_annotator")

        # subscribers
        CAMERA = "left" ## SWITCH CAMERAS TO INCREASE DATASET SIZE ON ADDITIONAL RUNS
        # CAMERA = "right"
        colour_sub = message_filters.Subscriber(
            self, Image, f"/zed2i/zed_node/{CAMERA}/image_rect_color"
        )
        colour_camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, f"/zed2i/zed_node/{CAMERA}/camera_info"
        )

        synchronizer = message_filters.TimeSynchronizer(
            fs=[colour_sub, colour_camera_info_sub],
            queue_size=30,
        )
        synchronizer.registerCallback(self.callback)

        self.get_logger().info("Initialised Annotator Node")
        
        self.count: int = 1500 ## START WHERE THE MOST RECENT ANNOTATION SET FINISHED


    def callback(self, colour_msg: Image, colour_camera_info_msg: CameraInfo):
        logger = self.get_logger()
        logger.info("Received image")
        
        # location of annotation files (png, txt)
        CURR_COUNT = str(self.count)
        PATHSTR: str = f"{ANNOTATION_PATH}annotation_{CURR_COUNT}"

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg)
        # colour_frame = cv2.flip(colour_frame, 1) ## UNCOMMENT TO INCREASE DATASET SIZE ON ADDITIONAL RUNS
        
        # save current frame
        cv2.imwrite(f"{PATHSTR}.png", colour_frame)

        # open txt file for annotations to pair with current frame
        file = open(f"{PATHSTR}.txt", 'w')

        # extract frame dimensions
        h, w, _ = colour_frame.shape
        for bounding_box, cone_colour, display_colour in get_yolo_bounding_boxes(colour_frame):
            # draw box for validation image check
            draw_box(colour_frame, box=bounding_box, colour=display_colour)
            # normalise all dimensions that annotating requires
            norm_x: float = bounding_box.center.x/w
            norm_y: float = bounding_box.center.y/h
            norm_w: float = bounding_box.width/w
            norm_h: float = bounding_box.height/h

            # write YOLOv5 annotation format to txt file
            file.write(
                str(cone_colour) + " " + \
                str(norm_x) + " " + \
                str(norm_y) + " " + \
                str(norm_w) + " " + \
                str(norm_h) + "\n"
            )
        file.close()

        # show and save validation frame (check false positives/negatives)
        cv2.imshow("debug", colour_frame)
        cv2.imwrite(f"{VALIDATION_PATH}annotation_{self.count}val.jpeg", colour_frame) 
        cv2.waitKey(1)

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    annotator_node = AnnotatorNode()

    rclpy.spin(annotator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
