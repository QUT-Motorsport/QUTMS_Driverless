import torch
import cv2

# initialising function for the YOLOv5 model with confidence threshold
def yolov5_init(conf_thresh, model_path):
    #loading the model
    model = torch.hub.load(
        'ultralytics/yolov5', 
        'custom', 
        path=model_path) # location of model in docker env
    model.conf = conf_thresh
    return model

