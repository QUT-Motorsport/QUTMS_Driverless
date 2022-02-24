from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from driverless_msgs.msg import ConeDetectionStamped, Cone
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

from typing import List


MARKER_HEIGHT = 0.3
MAX_NUM_CONES = 50


def marker_array_from_cone_detection(detection: ConeDetectionStamped) -> MarkerArray:
    cones: List[Cone] = detection.cones
    
    markers = []
    for i in range(MAX_NUM_CONES):
        if i < len(cones):
            markers.append(
                marker_msg(
                    x=cones[i].location.x,
                    y=cones[i].location.y,
                    id_=i,
                    header=detection.header,
                    cone_colour=cones[i].color
                )
            )
        else:
            markers.append(
                clear_marker_msg(
                    id_=i,
                    header=detection.header,
                )
            )
    
    return MarkerArray(markers=markers)


CONE_TO_RGB_MAP = {
    Cone.BLUE: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
    Cone.YELLOW: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
    Cone.ORANGE_BIG: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    Cone.ORANGE_SMALL: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
}


def marker_msg(
    x: float,
    y: float,
    id_: int,
    header: Header,
    cone_colour: int,
    name_space: str = "current_scan",
) -> Marker:
    return Marker(
        header=header,
        ns=name_space,
        id=id_,
        type=Marker.CYLINDER,
        action=Marker.ADD,
        pose=Pose(
            position=Point(x=x, y=y, z=MARKER_HEIGHT/2),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
        scale=Vector3(x=0.2, y=0.2, z=MARKER_HEIGHT),
        color=CONE_TO_RGB_MAP.get(cone_colour, ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)),
        lifetime=Duration(sec=1, nanosec=0),
    )


def clear_marker_msg(
    id_: int,
    header: Header,
    name_space: str = "current_scan"
) -> Marker:
    return Marker(
        header=header,
        ns=name_space,
        id=id_,
        action=Marker.DELETE,
    )
