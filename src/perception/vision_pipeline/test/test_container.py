import cv2
import numpy as np
from typing import List, Tuple

from ..vision_pipeline.rect import Rect
from ..vision_pipeline.yolo_model import yolov5_init

Colour = Tuple[int, int, int]
ConeMsgColour = int


CONFIDENCE = 0.45
MODEL_PATH = "src/perception/vision_pipeline/models/model.pt"
model = yolov5_init(CONFIDENCE, MODEL_PATH)

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

YOLO_CONE_DETECTION_PARAMETERS = [
    (0, BLUE_DISP_COLOUR),
    (1, YELLOW_DISP_COLOUR),
    (2, ORANGE_DISP_COLOUR),
]

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
                    data.xmin[i],
                    data.ymin[i],
                    (data.xmax[i]-data.xmin[i]),
                    (data.ymax[i]-data.ymin[i]),
                )
                bounding_boxes.append([bounding_box, cone_colour, display_colour])
    return bounding_boxes


colour_frame = cv2.imread("src/perception/vision_pipeline/test/testImage.png")

for bounding_box, cone_colour, display_colour in get_yolo_bounding_boxes(colour_frame):
    print(bounding_box)