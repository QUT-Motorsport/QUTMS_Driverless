## Info about these models

# darknet.labels
Used for importing class names into RoboFlow for decernable annotations.

# YBV2.pt
The second (hence V2) Yellow and Blue cone detection model for real-world:tm: data.
The model was trained on a 4000~ image/augmented dataset, which was automatically annotated using the first, YBV1 model.

# YBV2.yaml
Yaml configuration file for model classes with YOLOv5s weights structure.
YOLOv5s (small) was the base structure used to train all of our YOLOv5 models.

# YBV2.wts
A weights file, created by a convertion from YBV2.pt. 
Weights would be then used to convert the YOLOv5 PyTorch model a TensorRT optimised model

# YBV2.engine
Engine file read by TensorRT to optimise YOLOv5 detection

# libplugins.so
C type plugin shared object file for engine interpretation with TensorRT