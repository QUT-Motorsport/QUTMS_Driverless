from math import atan2, pi

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from ackermann_msgs.msg import AckermannDrive

from cv_bridge import CvBridge
import message_filters

import cv2
import numpy as np

from .threshold import Threshold
from .hsv_cv import get_coloured_bounding_boxes
from .rect import Rect, Point, draw_box
from .cone import Cone

from typing import Optional


cv_bridge = CvBridge()


LEFT_THRESH = Threshold(  # yellow
    lower=[20, 170, 100],
    upper=[40, 255, 255],
)

RIGHT_THRESH = Threshold(  # blue
    lower=[120, 160, 50],
    upper=[130, 255, 255],
)

LEFT_DISP_COLOUR = (0, 255, 255)  # bgr - yellow
RIGHT_DISP_COLOUR = (255, 0, 0)  # bgr - blue
TARGET_DISP_COLOUR = (0, 0, 255)  # bgr - red


def cone_from_bounding_box(
    bounding_box: Rect,
    disparity_frame: np.ndarray,
    f: float,
    t: float
) -> Cone:
    return Cone(
        bounding_box=bounding_box,
        distance=-f * t / disparity_frame[(bounding_box.center.y, bounding_box.center.x)]
    )


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")

        # subscribers
        colour_sub = message_filters.Subscriber(
            self, Image, "/zed2i/zed_node/rgb/image_rect_color"
        )
        disparity_sub = message_filters.Subscriber(
            self, DisparityImage, "/zed2i/zed_node/disparity/disparity_image"
        )

        synchronizer = message_filters.TimeSynchronizer(
            fs=[colour_sub, disparity_sub],
            queue_size=30,
        )
        synchronizer.registerCallback(self.callback)

        # publishers
        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "steering", 1)

        self.get_logger().info("Controller Node Initalised")


    def callback(self, colour_msg: Image, disparity_msg: DisparityImage):
        logger = self.get_logger()
        logger.info("Recieved image")

        colour_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(colour_msg)
        disparity_frame: np.ndarray = cv_bridge.imgmsg_to_cv2(disparity_msg.image, desired_encoding='32FC1')

        hsv_frame: np.ndarray = cv2.cvtColor(colour_frame, cv2.COLOR_BGR2HSV)

        left_bounds = get_coloured_bounding_boxes(hsv_frame, [LEFT_THRESH])
        right_bounds = get_coloured_bounding_boxes(hsv_frame, [RIGHT_THRESH])

        left_cones = [
            cone_from_bounding_box(b, disparity_frame, disparity_msg.f, disparity_msg.t) for b in left_bounds
        ]
        right_cones = [
            cone_from_bounding_box(b, disparity_frame, disparity_msg.f, disparity_msg.t) for b in right_bounds
        ]

        for c in left_cones:
            draw_box(colour_frame, box=c.bounding_box, colour=LEFT_DISP_COLOUR, distance=c.distance)
        for c in right_cones:
            draw_box(colour_frame, box=c.bounding_box, colour=RIGHT_DISP_COLOUR, distance=c.distance)
        
        closest_left_cone: Optional[Cone] = None
        closest_right_cone: Optional[Cone] = None

        if len(left_cones) > 0:
            closest_left_cone = min(left_cones, key=lambda c: c.distance)
            cv2.drawMarker(
                colour_frame, 
                (closest_left_cone.bounding_box.center.x, closest_left_cone.bounding_box.center.y),
                LEFT_DISP_COLOUR,
                markerSize=cv2.MARKER_STAR,
                thickness=20,
            )
        if len(right_cones) > 0:
            closest_right_cone = min(right_cones, key=lambda c: c.distance)
            cv2.drawMarker(
                colour_frame, 
                (closest_right_cone.bounding_box.center.x, closest_right_cone.bounding_box.center.y),
                RIGHT_DISP_COLOUR,
                markerSize=cv2.MARKER_STAR,
                thickness=20,
            )
        
        target: Optional[Point] = None
        if closest_left_cone is not None and closest_right_cone is not None:
            target = (
                closest_left_cone.bounding_box.bc
                + (closest_right_cone.bounding_box.bc - closest_left_cone.bounding_box.bc) / 2
            )
        elif closest_left_cone is not None:
            target = closest_left_cone.bounding_box.bc + Point(50, 0)
        elif closest_right_cone is not None:
            target = closest_right_cone.bounding_box.bc + Point(-50, 0)
        
        if target is not None:
            cv2.drawMarker(colour_frame, (target.x, target.y), TARGET_DISP_COLOUR, cv2.MARKER_TILTED_CROSS)
            height, width, _ = colour_frame.shape
            bottom_center = Point(int(round(width/2)), height)
            cv2.line(
                colour_frame, (bottom_center.x, bottom_center.y), (target.x, target.y), TARGET_DISP_COLOUR, thickness=2
            )

            steering_angle = (pi/2) - atan2(-(target.y-bottom_center.y), target.x-bottom_center.x)
            steering_msg = AckermannDrive()
            steering_msg.steering_angle = steering_angle
            self.steering_publisher.publish(steering_msg)
            logger.info(f"Published steering angle: {steering_angle}")

        cv2.imshow("frame", colour_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
