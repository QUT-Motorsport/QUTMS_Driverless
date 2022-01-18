import torch
import numpy as np
import cv2

# initialising function for the YOLOv5 model with confidence threshold
def torch_init(conf_thresh: float, model_path: str, repo_path: str):
    """
    Returns a YOLOv5 PyTorch model
    """
    model = torch.hub.load(
        repo_path, # having a local path reduces the time taken to load in model
        'custom', 
        source='local',
        path=model_path,
        # force_reload=True, # for fixing bad cache
    ) 
    model.conf = conf_thresh
    return model

def infer(colour_frame: np.ndarray, model):
    rgb_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2RGB)
    results = model(rgb_frame)
    data = results.pandas().xyxy[0]
    return data