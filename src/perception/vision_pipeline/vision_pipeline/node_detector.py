from math import cos, isinf, isnan, radians, sin, sqrt
import os
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from driverless_common.common import dist

from .rect import Rect, draw_box

from typing import Callable, List, Tuple

# translate ROS image messages to OpenCV
cv_bridge = CvBridge()

# display colour constants
Colour = Tuple[int, int, int]
YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 80, 255)  # bgr - orange
UNKNOWN_DISP_COLOUR: Colour = (0, 0, 0)  # bgr - black

# display_colour
CONE_DISPLAY_PARAMETERS = [
    BLUE_DISP_COLOUR,
    UNKNOWN_DISP_COLOUR,
    ORANGE_DISP_COLOUR,
    ORANGE_DISP_COLOUR,
    YELLOW_DISP_COLOUR,
]

# cone heights in metres
HEIGHTS = [0.28, 0.28, 0.525, 0.525, 0.28]
WIDTHS = [0.224, 0.224, 0.261, 0.261, 0.224]

# HEIGHTS = [0.303, 0.303, 0.303, 0.303, 0.303]
# WIDTHS = [ 0.224, 0.224, 0.224, 0.224, 0.224]


def cone_distance_homography(
    bbox: Rect,
    colour: int,
    # bearing: float,
    k: np.ndarray,
    d: np.ndarray,
    img,
) -> float:
    """
    Calculate distance using the bounding box height to camera homography matrices.
    """
    height = HEIGHTS[colour]
    width = height * bbox.aspect_ratio

    real_points = np.array(
        [
            (0, -width / 2, height / 2),
            (0, -width / 2, -height / 2),
            (0, width / 2, -height / 2),
            (0, width / 2, height / 2),
        ],
        dtype=np.float32,
    )

    image_points = np.array(
        [(bbox.tl.x, bbox.tl.y), (bbox.tl.x, bbox.br.y), (bbox.br.x, bbox.br.y), (bbox.br.x, bbox.tl.y)],
        dtype=np.float32,
    )

    success, rvec, tvec = cv2.solvePnP(real_points, image_points, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    # Find the rotation and translation vectors.
    # success, rvec, tvec, inliers = cv2.solvePnPRansac(real_points, image_points, k, d)

    points = np.float32([[0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.2], [0, 0, 0]]).reshape(-1, 3)
    axisPoints, _ = cv2.projectPoints(points, rvec, tvec, k, d)
    # convert to ints
    axisPoints = axisPoints.astype(int)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255, 0, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0, 255, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0, 0, 255), 3)

    rmat = cv2.Rodrigues(rvec)[0]
    cone_position = -np.matrix(rmat).T * np.matrix(tvec)
    return cone_position.flatten().tolist()[0]


def cone_msg(
    pos: tuple,
    colour: int,
) -> Cone:

    location = Point(
        x=pos[0],
        y=pos[1],
        z=-0.8,
    )
    if colour == Cone.ORANGE_SMALL:
        colour = Cone.ORANGE_BIG

    return Cone(
        location=location,
        color=colour,
    )


class FPSHandler:
    def __init__(self):
        self.timestamp = time.time() + 1
        self.start = time.time()
        self.frame_cnt = 0

    def next_iter(self):
        self.timestamp = time.time()
        self.frame_cnt += 1

    def fps(self):
        return self.frame_cnt / (self.timestamp - self.start)


class VisionProcessor(Node):
    end: float = 0.0

    right_distortion_coefficients = None
    right_matrix_coefficients = None
    left_distortion_coefficients = None
    left_matrix_coefficients = None

    fps = FPSHandler()

    def __init__(
        self,
        get_bounding_boxes_callable: Callable[[np.ndarray], List[Tuple[Rect, int, Colour]]],
    ):
        super().__init__("vision_processor_node")

        # declare ros param for debug images
        self.declare_parameter("debug_bbox", True)
        self.declare_parameter("debug_depth", False)
        self.declare_parameter("log_level", "DEBUG")
        self.declare_parameter("max_range", 15.0)
        self.declare_parameter("min_range", 2.0)

        self.debug_bbox = self.get_parameter("debug_bbox").value
        self.debug_depth = self.get_parameter("debug_depth").value
        self.log_level = self.get_parameter("log_level").value
        self.max_range = self.get_parameter("max_range").value
        self.min_range = self.get_parameter("min_range").value

        self.log_level = self.get_parameter("log_level").value
        log_level: int = getattr(rclpy.impl.logging_severity.LoggingSeverity, self.log_level.upper())
        self.get_logger().set_level(log_level)

        # subscribers
        self.create_subscription(CameraInfo, "/zed2i/zed_node/left_raw/camera_info", self.info_define, 1)
        self.create_subscription(CameraInfo, "/zed2i/zed_node/right_raw/camera_info", self.info_define, 1)
        self.create_subscription(Image, "/zed2i/zed_node/left_raw/image_raw_color", self.callback, 1)
        self.create_subscription(Image, "/zed2i/zed_node/right_raw/image_raw_color", self.callback, 1)

        # publishers
        self.detection_publisher: Publisher = self.create_publisher(ConeDetectionStamped, "/vision/cone_detection", 1)
        if self.debug_bbox:
            self.right_debug_img_pub: Publisher = self.create_publisher(Image, "/debug_imgs/vision_bbox_right", 1)
            self.left_debug_img_pub: Publisher = self.create_publisher(Image, "/debug_imgs/vision_bbox_left", 1)

        # set which cone detection this will be using
        self.get_bounding_boxes_callable = get_bounding_boxes_callable

        self.end = time.perf_counter()
        self.get_logger().info(
            "PARAMS: debug_bbox: " + str(self.debug_bbox) + "\t | debug_depth: " + str(self.debug_depth)
        )
        self.get_logger().info("---Vision detector node initialised---")

    def info_define(self, info_msg: CameraInfo):
        if info_msg.header.frame_id == "zed2i_left_camera_optical_frame" and self.left_distortion_coefficients is None:
            self.left_distortion_coefficients = np.array(info_msg.d)
            self.left_matrix_coefficients = np.array(info_msg.k).reshape(3, 3)
            self.get_logger().info("Distortion Coefficients: \n" + str(self.left_distortion_coefficients))
            self.get_logger().info("Calibration Matrix: \n" + str(self.left_matrix_coefficients))
        elif (
            info_msg.header.frame_id == "zed2i_right_camera_optical_frame"
            and self.right_distortion_coefficients is None
        ):
            self.right_distortion_coefficients = np.array(info_msg.d)
            self.right_matrix_coefficients = np.array(info_msg.k).reshape(3, 3)
            self.get_logger().info("Distortion Coefficients: \n" + str(self.right_distortion_coefficients))
            self.get_logger().info("Calibration Matrix: \n" + str(self.right_matrix_coefficients))

    def callback(self, msg: Image):
        self.get_logger().debug("Received image")

        if msg.header.frame_id == "zed2i_left_camera_optical_frame":
            if self.left_distortion_coefficients is None:
                return

            matrix_coefficients = self.left_matrix_coefficients
            distortion_coefficients = self.left_distortion_coefficients
            camera = "left"
        else:
            if self.right_distortion_coefficients is None:
                return

            matrix_coefficients = self.right_matrix_coefficients
            distortion_coefficients = self.right_distortion_coefficients
            camera = "right"

        self.get_logger().debug(
            f"Wait time: {str(time.perf_counter()-self.end)}", throttle_duration_sec=0.5
        )  # log time
        start: float = time.perf_counter()  # begin a timer

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        if colour_frame is None:
            return

        detected_cones: List[Cone] = []
        for bounding_box, cone_colour, display_colour in self.get_bounding_boxes_callable(colour_frame):
            # filter by height
            if bounding_box.tl.y < msg.height / 2:
                continue
            # filter on area
            if bounding_box.area < 10:
                continue
            # filter by aspect ratio
            if bounding_box.aspect_ratio < 0.4 or bounding_box.aspect_ratio > 1.5:
                continue

            # using bounding box sizes for distance
            location = cone_distance_homography(
                bounding_box, cone_colour, matrix_coefficients, distortion_coefficients, colour_frame
            )

            # filter on distance
            distance = dist([location[0], location[1]], [0, 0])
            if distance > self.max_range or distance < self.min_range:
                continue

            detected_cones.append(cone_msg(location, cone_colour))

            if self.debug_bbox:
                draw_box(colour_frame, box=bounding_box, colour=display_colour, distance=distance)

        detection_msg = ConeDetectionStamped(
            header=Header(frame_id="zed2i", stamp=msg.header.stamp),
            cones=detected_cones,
        )
        self.detection_publisher.publish(detection_msg)

        if self.debug_bbox:
            debug_msg = cv_bridge.cv2_to_imgmsg(colour_frame, encoding=msg.encoding)
            debug_msg.header = Header(frame_id="zed2i", stamp=msg.header.stamp)
            if camera == "left":
                self.left_debug_img_pub.publish(debug_msg)
            else:
                self.right_debug_img_pub.publish(debug_msg)

        self.fps.next_iter()
        self.get_logger().debug(
            f"Process Time: {round(time.perf_counter() - start, 4)}\t| EST. FPS: {round(self.fps.fps(), 2)}",
            throttle_duration_sec=0.5,
        )
        self.end = time.perf_counter()


## PyTorch inference
def main_v8_torch(args=None):
    from .yolov8_torch_inference import YOLOv8Wrapper

    # loading Pytorch model
    MODEL_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "models", "yolov8n_720.engine")
    CONFIDENCE = 0.25  # higher = tighter filter
    IMGSZ = 1280
    SEGMENTATION = False
    wrapper = YOLOv8Wrapper(MODEL_PATH, CONFIDENCE, IMGSZ, SEGMENTATION)

    def get_torch_bounding_boxes(
        colour_frame: np.ndarray,
    ) -> List[Tuple[Rect, int, Colour]]:  # bbox, msg colour, display colour
        bounding_boxes: List[Tuple[Rect, int, Colour]] = []
        data = wrapper.infer(colour_frame)

        for detection in data:
            cone_colour = int(detection[0])
            bounding_box = Rect(
                int(detection[1]),
                int(detection[2]),
                int(detection[3] - detection[1]),
                int(detection[4] - detection[2]),
            )
            bounding_boxes.append((bounding_box, cone_colour, CONE_DISPLAY_PARAMETERS[cone_colour]))
        return bounding_boxes

    rclpy.init(args=args)
    node = VisionProcessor(get_torch_bounding_boxes)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


## PyTorch inference
def main_torch(args=None):
    from .torch_inference import infer, torch_init  # , torch_init_v7

    # loading Pytorch model
    MODEL_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "models", "yolov5_small.pt")
    REPO_PATH = os.path.join(get_package_share_directory("vision_pipeline"), "yolov5")
    CONFIDENCE = 0.25  # higher = tighter filter
    IOU = 0.1
    model = torch_init(MODEL_PATH, REPO_PATH, CONFIDENCE, IOU)

    def get_torch_bounding_boxes(
        colour_frame: np.ndarray,
    ) -> List[Tuple[Rect, int, Colour]]:  # bbox, msg colour, display colour
        bounding_boxes: List[Tuple[Rect, int, Colour]] = []
        data = infer(colour_frame, model)

        for i in range(len(data.index)):
            cone_colour = int(data.iloc[i, 5])  # locates object i, class ID at index 5
            bounding_box = Rect(
                int(data.xmin[i]),
                int(data.ymin[i]),
                int(data.xmax[i] - data.xmin[i]),
                int(data.ymax[i] - data.ymin[i]),
            )
            bounding_boxes.append((bounding_box, cone_colour, CONE_DISPLAY_PARAMETERS[cone_colour]))
        return bounding_boxes

    rclpy.init(args=args)
    node = VisionProcessor(get_torch_bounding_boxes)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
