import cv2
import numpy as np
import torch


# initialising function for the YOLOv5 model with confidence threshold
def torch_init(model_path: str, repo_path: str, conf_thresh: float, iou_thresh: float) -> torch.nn.Module:
    """
    Returns a YOLOv5 PyTorch model
    """
    model = torch.hub.load(
        repo_path,  # having a local path reduces the time taken to load in model
        "custom",
        source="local",
        path=model_path,
        # force_reload=True,  # for fixing bad cache
    )
    model.conf = conf_thresh
    model.iou = iou_thresh
    model.agnostic = True
    return model


def infer(colour_frame: np.ndarray, model):
    rgb_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2RGB)
    results = model(rgb_frame)
    data = results.pandas().xyxy[0]
    return data
