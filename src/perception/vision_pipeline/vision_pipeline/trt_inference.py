## The code below is adapted from 
# https://github.com/wang-xinyu/tensorrtx/blob/master/yolov5/yolov5_trt.py

# import python libraries
import ctypes
import time
import cv2
import numpy as np
from typing import Tuple, List
# import TensorRT libraries
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt

class TensorWrapper(object):
    """
    description: A YOLOv5 class that wraps TensorRT ops, preprocess and postprocess ops.
    """
    def __init__(
        self, 
        engine_file_path: str, 
        pluggin_file_path: str,
        conf_thresh: float=0.5,
        iou_thresh: float=0.4,
    ):
        # load in object pluggin file
        ctypes.CDLL(pluggin_file_path)
        # create a Context on this device,
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        f = open(engine_file_path, "rb")           
        engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()
        f.close() # might break 2nd loop of inference, hope not

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                self.input_w: int = engine.get_binding_shape(binding)[-1]
                self.input_h: int = engine.get_binding_shape(binding)[-2]
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self.batch_size = engine.max_batch_size
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh


    def infer(self, image_raw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Make self the active context, pushing it on top of the context stack.
        self.ctx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        
        # Do image preprocess
        input_image, image_raw, origin_h, origin_w = self.preprocess_image(image_raw)

        # Copy input image to host buffer
        np.copyto(host_inputs[0], input_image.ravel())
        # Transfer input data to the GPU.
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # Run inference.
        context.execute_async(batch_size=self.batch_size, bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()
        # Here we use the first row of output in that batch_size = 1
        output: np.ndarray = host_outputs[0]

        # Do postprocess and return
        return self.post_process(output, origin_h, origin_w)
                

    def preprocess_image(self, raw_bgr_image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int, int]:
        """
        description: Convert BGR image to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.\n
        param:
            raw_bgr_image: the original image
        return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        image_raw: np.ndarray = raw_bgr_image
        h, w, _ = image_raw.shape
        image: np.ndarray = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w: float = self.input_w / w
        r_h: float = self.input_h / h
        if r_h > r_w:
            tw = self.input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((self.input_h - th) / 2)
            ty2 = self.input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = self.input_h
            tx1 = int((self.input_w - tw) / 2)
            tx2 = self.input_w - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))

        # CHECK SIZING OF ROS MESSAGE
        # print(tw, th, ty1, ty2, tx1, tx2)

        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, (128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w


    def post_process(
        self, output: np.ndarray, origin_h: int, origin_w: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        description: postprocess the prediction.\n
        param:
            output:     A numpy likes [num_boxes,cx,cy,w,h,conf,cls_id, cx,cy,w,h,conf,cls_id, ...] 
            origin_h:   height of original image
            origin_w:   width of original image
        return:
            result_boxes: finally boxes, a boxes numpy, each row is a box [x1, y1, x2, y2]
            result_scores: finally scores, a numpy, each element is the score correspoing to box
            result_classid: finally classid, a numpy, each element is the classid correspoing to box
        """
        # Get the num of boxes detected
        num = int(output[0])
        # Reshape to a two dimentional ndarray
        pred: np.ndarray = np.reshape(output[1:], (-1, 6))[:num, :]
        # Do nms
        boxes: np.ndarray = self.non_max_suppression(
            pred, origin_h, origin_w, conf_thres=self.conf_thresh, nms_thres=self.iou_thresh
        )
        result_boxes: np.ndarray = boxes[:, :4] if len(boxes) else np.array([])
        result_scores: np.ndarray = boxes[:, 4] if len(boxes) else np.array([])
        result_classid: np.ndarray = boxes[:, 5] if len(boxes) else np.array([])
        return result_boxes, result_scores, result_classid


    def non_max_suppression(
        self, prediction: np.ndarray, 
        origin_h: int, 
        origin_w: int, 
        conf_thres: float=0.5, 
        nms_thres: float=0.4
    ) -> np.ndarray:
        """
        description: Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.\n
        param:
            prediction: detections, (x1, y1, x2, y2, conf, cls_id)
            origin_h: original image height
            origin_w: original image width
            conf_thres: a confidence threshold to filter detections
            nms_thres: a iou threshold to filter detections
        return:
            boxes: output after nms with the shape (x1, y1, x2, y2, conf, cls_id)
        """
        # Get the boxes that score > CONF_THRESH
        boxes: np.ndarray = prediction[prediction[:, 4] >= conf_thres]
        # Trandform bbox from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        boxes[:, :4] = self.xywh2xyxy(origin_h, origin_w, boxes[:, :4])
        # clip the coordinates
        boxes[:, 0] = np.clip(boxes[:, 0], 0, origin_w -1)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, origin_w -1)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, origin_h -1)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, origin_h -1)
        # Object confidence
        confs: np.ndarray = boxes[:, 4]
        # Sort by the confs
        boxes = boxes[np.argsort(-confs)]
        # Perform non-maximum suppression
        keep_boxes = []
        while boxes.shape[0]:
            large_overlap: bool = self.bbox_iou(np.expand_dims(boxes[0, :4], 0), boxes[:, :4]) > nms_thres
            label_match: bool = boxes[0, -1] == boxes[:, -1]
            # Indices of boxes with lower confidence scores, large IOUs and matching labels
            invalid: bool = large_overlap & label_match
            keep_boxes += [boxes[0]]
            boxes = boxes[~invalid]
        boxes = np.stack(keep_boxes, 0) if len(keep_boxes) else np.array([])
        return boxes


    def xywh2xyxy(self, origin_h: int, origin_w: int, x: int) -> np.ndarray:
        """
        description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] 
                        where xy1=top-left, xy2=bottom-right.\n
        param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes numpy, each row is a box [center_x, center_y, w, h]
        return:
            y:          A boxes numpy, each row is a box [x1, y1, x2, y2]
        """
        y: np.ndarray = np.zeros_like(x)
        r_w = self.input_w / origin_w
        r_h = self.input_h / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (self.input_h - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (self.input_w - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

        return y


    def bbox_iou(self, box1: np.ndarray, box2: np.ndarray, x1y1x2y2: bool=True) -> float:
        """
        description: compute the IoU of two bounding boxes.\n
        param:
            box1: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            box2: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))            
            x1y1x2y2: select the coordinate format
        return:
            iou: computed iou
        """
        if not x1y1x2y2:
            # Transform from center and width to exact coordinates
            b1_x1, b1_x2 = box1[:, 0] - box1[:, 2] / 2, box1[:, 0] + box1[:, 2] / 2
            b1_y1, b1_y2 = box1[:, 1] - box1[:, 3] / 2, box1[:, 1] + box1[:, 3] / 2
            b2_x1, b2_x2 = box2[:, 0] - box2[:, 2] / 2, box2[:, 0] + box2[:, 2] / 2
            b2_y1, b2_y2 = box2[:, 1] - box2[:, 3] / 2, box2[:, 1] + box2[:, 3] / 2
        else:
            # Get the coordinates of bounding boxes
            b1_x1, b1_y1, b1_x2, b1_y2 = box1[:, 0], box1[:, 1], box1[:, 2], box1[:, 3]
            b2_x1, b2_y1, b2_x2, b2_y2 = box2[:, 0], box2[:, 1], box2[:, 2], box2[:, 3]

        # Get the coordinates of the intersection rectangle
        inter_rect_x1: np.ndarray = np.maximum(b1_x1, b2_x1)
        inter_rect_y1: np.ndarray = np.maximum(b1_y1, b2_y1)
        inter_rect_x2: np.ndarray = np.minimum(b1_x2, b2_x2)
        inter_rect_y2: np.ndarray = np.minimum(b1_y2, b2_y2)
        # Intersection area
        inter_area: np.ndarray = np.clip(inter_rect_x2 - inter_rect_x1 + 1, 0, None) * \
                                 np.clip(inter_rect_y2 - inter_rect_y1 + 1, 0, None)
        # Union Area
        b1_area: float = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
        b2_area: float = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)
        iou: float = inter_area / (b1_area + b2_area - inter_area + 1e-16)

        return iou


    def destroy(self):
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()