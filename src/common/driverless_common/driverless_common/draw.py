from math import cos, pi, sin

import cv2
import numpy as np

from driverless_msgs.msg import Cone, TrackDetectionStamped

from driverless_common.point import Point

from typing import List, Tuple

# image display geometry
MAP_SCALE = 4
MAP_HEIGHT = (10 + 170) * MAP_SCALE
MAP_WIDTH = (50 + 50) * MAP_SCALE

SCALE = 20
WIDTH = 20 * SCALE  # 15m either side
HEIGHT = 20 * SCALE  # 30m forward

ORIGIN = Point(0, 0)
IMG_ORIGIN = Point(int(WIDTH / 2), HEIGHT)

Colour = Tuple[int, int, int]

YELLOW_DISP_COLOUR: Colour = (0, 255, 255)  # bgr - yellow
BLUE_DISP_COLOUR: Colour = (255, 0, 0)  # bgr - blue
ORANGE_DISP_COLOUR: Colour = (0, 80, 255)  # bgr - orange
UNKNOWN_DISP_COLOUR: Colour = (255, 255, 255)  # bgr - white

LEFT_CONE_COLOUR = Cone.YELLOW
RIGHT_CONE_COLOUR = Cone.BLUE


def coord_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(50 * MAP_SCALE + x * MAP_SCALE)),
        int(round(170 * MAP_SCALE - y * MAP_SCALE)),
    )


def loc_to_img_pt(x: float, y: float) -> Point:
    """
    Converts a relative depth from the camera into image coords
    * param x: x coord
    * param y: y coord
    * return: Point int pixel coords
    """
    return Point(
        int(round(WIDTH / 2 - y * SCALE)),
        int(round(HEIGHT - x * SCALE)),
    )


def cone_to_point(cone: Cone) -> Point:
    return Point(
        cone.location.x,
        cone.location.y,
    )


def draw_map(track: TrackDetectionStamped) -> np.ndarray:
    map_image = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3), dtype=np.uint8)

    for cone in track.cones:
        if cone.cone.color == Cone.YELLOW:
            colour = YELLOW_DISP_COLOUR
        elif cone.cone.color == Cone.BLUE:
            colour = BLUE_DISP_COLOUR
        elif cone.cone.color == Cone.ORANGE_BIG:
            colour = ORANGE_DISP_COLOUR
        else:
            colour = (255, 255, 255)

        cv2.drawMarker(
            map_image,
            coord_to_img_pt(cone.cone.location.x, cone.cone.location.y).to_tuple(),
            colour,
            markerType=cv2.MARKER_TRIANGLE_UP,
            markerSize=6,
            thickness=2,
        )

    return map_image


def draw_markers(cones: List[Cone]) -> np.ndarray:
    debug_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    for cone in cones:
        if cone.color == Cone.YELLOW:
            colour = YELLOW_DISP_COLOUR
        elif cone.color == Cone.BLUE:
            colour = BLUE_DISP_COLOUR
        elif cone.color == Cone.ORANGE_BIG:
            colour = ORANGE_DISP_COLOUR
        else:
            colour = (255, 255, 255)

        cv2.drawMarker(
            debug_img,
            loc_to_img_pt(cone.location.x, cone.location.y).to_tuple(),
            colour,
            markerType=cv2.MARKER_SQUARE,
            markerSize=5,
            thickness=5,
        )

    return debug_img


def draw_steering(debug_img: np.ndarray, steering_angle: float, velocity: float):
    # draw angle line
    cv2.line(
        debug_img,
        (
            int(50 * cos(steering_angle / 4 - pi / 2) + IMG_ORIGIN.x),
            int(50 * sin(steering_angle / 4 - pi / 2) + IMG_ORIGIN.y),
        ),
        IMG_ORIGIN.to_tuple(),
        (0, 0, 255),
    )
    # add text for targets data
    cv2.putText(debug_img, "Targets", (10, HEIGHT - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    text_angle = "Steering: " + str(round(steering_angle, 2))
    cv2.putText(debug_img, text_angle, (10, HEIGHT - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    text_vel = "Velocity: " + str(round(velocity, 2))
    cv2.putText(debug_img, text_vel, (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return debug_img
