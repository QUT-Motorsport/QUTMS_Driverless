
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from driverless_msgs.msg import Cone, ConeDetectionStamped

from cv_bridge import CvBridge

from math import sqrt, atan2, pi, sin, cos
import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate
from typing import Tuple, List, Optional
import time

from .point import Point

Colour = Tuple[int, int, int]

cv_bridge = CvBridge()

SCALE = 20
WIDTH = 20*SCALE  # 10m either side
HEIGHT = 20*SCALE  # 20m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH/2), HEIGHT)

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 165, 255)  # bgr - orange

LEFT_CONE_COLOUR = Cone.BLUE
RIGHT_CONE_COLOUR = Cone.YELLOW


def robot_pt_to_img_pt(x: float, y: float) -> Point:
    return Point(
        int(round(WIDTH/2 - y*SCALE)),
        int(round(HEIGHT - x*SCALE)),
    )


def dist(a: Point, b: Point) -> float:
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def approximate_b_spline_path(x: list, y: list, n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    approximate points with a B-Spline path

    :param x: x position list of approximated points
    :param y: y position list of approximated points
    :param n_path_points: number of path points
    :param degree: (Optional) B Spline curve degree
    :return: x and y position list of the result path
    """
    t = range(len(x))
    x_tup = scipy_interpolate.splrep(t, x, k=degree)
    y_tup = scipy_interpolate.splrep(t, y, k=degree)

    x_list = list(x_tup)
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    rx = scipy_interpolate.splev(ipl_t, x_list)
    ry = scipy_interpolate.splev(ipl_t, y_list)

    return rx, ry


def midpoint(p1: list, p2: list):
    return (p1[0]+p2[0])/2, (p1[1]+p2[1])/2


class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # subscribers
        self.create_subscription(ConeDetectionStamped, "zed_detector/cone_detection", self.cone_detection_callback, 1)

        # publishers
        self.debug_img_publisher: Publisher = self.create_publisher(Image, "simple_controller/debug_img", 1)
        self.plot_img_publisher: Publisher = self.create_publisher(Image, "simple_controller/plot_img", 1)
        self.steering_publisher: Publisher = self.create_publisher(AckermannDrive, "steering", 1)

        self.get_logger().info("Simple Controller Node Initalised")

        self.fig = plt.figure()


    def cone_detection_callback(self, msg: ConeDetectionStamped):
        logger = self.get_logger()
        logger.info("Received detection")

        cones: List[Cone] = msg.cones

        debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        yellow_list: List[float] = []
        blue_list: List[float] = []

        for cone in cones:
            if cone.color == Cone.YELLOW:
                colour = YELLOW_DISP_COLOUR
                yellow_list.append([cone.location.x, cone.location.y])
            elif cone.color == Cone.BLUE:
                colour = BLUE_DISP_COLOUR
                blue_list.append([cone.location.x, cone.location.y])
            else:
                colour = (255, 255, 255)
            
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
                colour,
                markerType=cv2.MARKER_SQUARE,
                markerSize=5,
                thickness=5
            )

        ## SPLINE PLANNER CODE
        start: float = time.time()

        yellow_x: List[float] = []
        yellow_y = []
        blue_x = []
        blue_y = []
        tx = []
        ty = []

        if len(yellow_list) > 1 and len(blue_list) > 1:
            y_degree = len(yellow_list)-1
            b_degree = len(blue_list)-1

            if y_degree > 3: y_degree = 3
            if b_degree > 3: b_degree = 3

            yellow_sort = sorted(yellow_list, key=lambda x: (x[0],x[1]))
            blue_sort = sorted(blue_list, key=lambda x: (x[0],x[1]))
            for i in yellow_sort:
                yellow_x.append(-i[1])
                yellow_y.append(i[0])
            for i in blue_sort:
                blue_x.append(-i[1])
                blue_y.append(i[0])
            yx, yy = approximate_b_spline_path(yellow_x, yellow_y, 100, degree=y_degree)
            bx, by = approximate_b_spline_path(blue_x, blue_y, 100, degree=b_degree)

            for i in range(100):
                mid_x, mid_y = midpoint([yx[i], yy[i]], [bx[i], by[i]])
                tx.append(mid_x)
                ty.append(mid_y)
        logger.info("Algo crunching time: " + str(time.time()-start))

        ## ORIGINAL BANG-BANG CODE
        left_cones = [c for c in cones if c.color == LEFT_CONE_COLOUR]
        right_cones = [c for c in cones if c.color == RIGHT_CONE_COLOUR]

        closest_left: Optional[Cone] = None
        closest_right: Optional[Cone] = None
        if len(left_cones) > 0:
            closest_left = min(left_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        if len(right_cones) > 0:
            closest_right = min(right_cones, key=lambda c: dist(ORIGIN, cone_to_point(c)))
        
        # if we have two cones, check if they are greater than 5 meters apart
        if closest_left is not None and closest_right is not None:
            if dist(cone_to_point(closest_left), cone_to_point(closest_right)) > 5:
                # if so - remove the furthest cone from the targeting
                left_dist = dist(ORIGIN, cone_to_point(closest_left))
                right_dist = dist(ORIGIN, cone_to_point(closest_right))
                if left_dist <= right_dist:
                    closest_right = None
                else:
                    closest_left = None

        target: Optional[Point] = None
        if closest_left is not None and closest_right is not None:
            target = Point(
                x=closest_left.location.x + (closest_right.location.x - closest_left.location.x)/2,
                y=closest_left.location.y + (closest_right.location.y - closest_left.location.y)/2,
            )
        elif closest_left is not None:
            target = Point(
                x=closest_left.location.x,
                y=closest_left.location.y - 2,
            )
        elif closest_right is not None:
            target = Point(
                x=closest_right.location.x,
                y=closest_right.location.y + 2,
            )
        
        if tx != []:
            target = Point(ty[50], -tx[50])


        if target is not None:
            print(target)
            target_img_pt = robot_pt_to_img_pt(target.x, target.y)
            cv2.drawMarker(
                debug_img, 
                robot_pt_to_img_pt(target.x, target.y).to_tuple(),
                (0, 0, 255),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=10,
                thickness=2
            )
            target_img_angle = atan2(target_img_pt.y - IMG_ORIGIN.y, target_img_pt.x - IMG_ORIGIN.x)
            
            cv2.line(
                debug_img,
                (int(50*cos(target_img_angle) + IMG_ORIGIN.x), int(50*sin(target_img_angle) + IMG_ORIGIN.y)),
                IMG_ORIGIN.to_tuple(),
                (0, 0, 255)
            )

            steering_angle = -((pi/2) - atan2(target.x, target.y))*5
            steering_msg = AckermannDrive()
            steering_msg.steering_angle = steering_angle
            self.steering_publisher.publish(steering_msg)
            logger.info(f"Published steering angle: {steering_angle}")
        
        self.debug_img_publisher.publish(cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))
        
        start: float = time.time()
        if len(yellow_list) > 3 and len(blue_list) > 3:
            # show results
            plt.clf()
            plt.plot(yellow_x, yellow_y, '-oy')
            plt.plot(blue_x, blue_y, '-ob')
            plt.plot(yx, yy, '-y')
            plt.plot(bx, by, '-b')
            plt.plot(tx, ty, '-r')
            plt.plot(tx[50], ty[50], 'or')
            plt.grid(True)
            plt.axis("equal")

            self.fig.canvas.draw()

            plot_img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            plot_img = plot_img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            plot_img = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)

            self.plot_img_publisher.publish(cv_bridge.cv2_to_imgmsg(plot_img, encoding="bgr8"))

        logger.info("Plot time: " + str(time.time()-start))


def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()

    rclpy.spin(simple_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
