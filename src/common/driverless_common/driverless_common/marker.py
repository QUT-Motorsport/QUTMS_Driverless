from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from driverless_msgs.msg import ConeDetectionStamped, Cone
from std_msgs.msg import Header

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


def marker_msg(
    x: float,
    y: float,
    id_: int,
    header: Header,
    cone_colour: int,
    name_space: str = "current_scan",
) -> Marker: 

    marker = Marker()
    marker.header = header
    marker.ns = name_space
    marker.id = id_
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = MARKER_HEIGHT / 2
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # size of the marker
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = MARKER_HEIGHT

    if cone_colour == Cone.BLUE:
        r, g, b = 0, 0, 255
    elif cone_colour == Cone.YELLOW:
        r, g, b = 255, 255, 0
    elif cone_colour in [Cone.ORANGE_BIG, Cone.ORANGE_SMALL]:
        r, g, b = 255, 0, 0
    else:
        r, g, b = 0, 0, 0

    marker.color.r = float(r)
    marker.color.g = float(g)
    marker.color.b = float(b)
    marker.color.a = 1.0

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker


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
