import torch

# initialising function for the YOLOv5 model with confidence threshold
def yolov5_init(conf_thresh: float, model_path: str, repo_path: str=None):
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
