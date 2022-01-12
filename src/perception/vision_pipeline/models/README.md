## Info about these models

# darknet.labels
Used for importing class names into RoboFlow for decernable annotations.

# model.pt
Very first, albiet small, YOLOv5 model made by Lachlan Massan and Brayth Tarlington to be used on the FSDS.

# YBV2.pt
The second (hence V2) Yellow and Blue cone detection model for real-world:tm: data.
The model was trained on a 4000~ image/augmented dataset, which was automatically annotated using the first, YBV1 model.

# YBV2.yaml
Yaml configuration file for model classes with YOLOv5s weights structure.
YOLOv5s (small) was the base structure used to train all of our YOLOv5 models.

# yolov5s.wts
A weights file, created by a convertion from YBV2.pt. 
Weights would be then used to convert the YOLOv5 PyTorch model a TensorRT optimised model