import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.engine.results import Results


class YOLOv8Wrapper:
    def __init__(self, model_path: str, conf_thresh: float = 0.7, imgsz: int = 1280, segment: bool = False):
        """
        initialising function for the YOLOv8 PyTorch model with confidence threshold
        """
        if segment:
            task = "segment"
        else:
            task = "detect"

        self.model = YOLO(model_path, task=task)
        if model_path.endswith(".pt"):
            self.model.info(verbose=True, detailed=False)
        self.model.conf = conf_thresh
        self.imgsz = imgsz
        self.segment = segment

    def infer(self, colour_frame: np.ndarray, verbose: bool = False):
        """
        function for running inference on a single frame
        """
        start = time.time()
        colour_frame = cv2.cvtColor(colour_frame, cv2.COLOR_RGBA2RGB)
        frame_result: Results = self.model.predict(colour_frame, verbose=verbose, imgsz=self.imgsz)[0]
        end = time.time()

        detection_boxes = []
        detection_masks = []
        if frame_result.boxes.xyxy.shape[0] == 0:
            return []

        for i in range(len(frame_result.boxes.xyxy)):
            # ensure box is int list type
            class_id = [frame_result.boxes.cls[i].to(device=torch.device("cpu"), dtype=torch.int32).tolist()]
            box = frame_result.boxes.xyxy[i].to(device=torch.device("cpu"), dtype=torch.int32).tolist()

            detection_boxes.append(class_id + box)
            # convert mask to numpy array
            if self.segment:
                # convert from 0-1 float pixel values to 0-255 int pixel values
                int_mask = (frame_result.masks.data[0].to(device=torch.device("cpu")).numpy() * 255).astype(np.uint8)
                # 1080 isnt divisible by 32, so this outputs to 1088. resize to 1080
                int_mask = cv2.resize(
                    int_mask, (colour_frame.shape[0], colour_frame.shape[1]), interpolation=cv2.INTER_NEAREST
                )
                detection_masks.append(int_mask)

        if self.segment:
            return detection_boxes, detection_masks
        else:
            return detection_boxes
