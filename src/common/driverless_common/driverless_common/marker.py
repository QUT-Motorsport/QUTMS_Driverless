from math import sqrt

from builtin_interfaces.msg import Duration
from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance, TrackDetectionStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from typing import List

MARKER_HEIGHT = 0.3
MAX_NUM_CONES = 50


def marker_array_from_map(detection: TrackDetectionStamped) -> MarkerArray:
    cones: List[ConeWithCovariance] = detection.cones

    markers = []
    for i, cone in enumerate(cones):
        markers.append(
            marker_msg(
                x=cone.cone.location.x,
                y=cone.cone.location.y,
                z=MARKER_HEIGHT / 2,
                id_=i,
                header=detection.header,
                cone_colour=cone.cone.color,
                lifetime=Duration(sec=10, nanosec=100000),
            )
        )
        markers.append(
            cov_marker_msg(
                x=cone.cone.location.x,
                y=cone.cone.location.y,
                id_=i,
                x_scale=3 * sqrt(abs(cone.covariance[0])),
                y_scale=3 * sqrt(abs(cone.covariance[3])),
                lifetime=Duration(sec=10, nanosec=100000),
            )
        )

    return MarkerArray(markers=markers)


def marker_array_from_cone_detection(detection: ConeDetectionStamped) -> MarkerArray:
    cones: List[Cone] = detection.cones

    markers = []
    for i in range(MAX_NUM_CONES):
        if i < len(cones):
            markers.append(
                marker_msg(
                    x=cones[i].location.x,
                    y=cones[i].location.y,
                    z=cones[i].location.z,
                    id_=i,
                    header=detection.header,
                    cone_colour=cones[i].color,
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
    Cone.ORANGE_BIG: ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
    Cone.ORANGE_SMALL: ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
}


def marker_msg(
    x: float, y: float, z: float, id_: int, header: Header, cone_colour: int, lifetime=Duration(sec=1, nanosec=0)
) -> Marker:
    return Marker(
        header=header,
        ns="current_scan",
        id=id_,
        type=Marker.CYLINDER,
        action=Marker.ADD,
        pose=Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        scale=Vector3(x=0.2, y=0.2, z=MARKER_HEIGHT),
        color=CONE_TO_RGB_MAP.get(cone_colour, ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)),
        lifetime=lifetime,
    )


def cov_marker_msg(
    x: float,
    y: float,
    id_: int,
    x_scale: float,  # x sigma
    y_scale: float,  # y sigma
    lifetime=Duration(sec=1, nanosec=0),
) -> Marker:
    header = Header(frame_id="map")
    return Marker(
        header=header,
        ns="cov_markers",
        id=id_,
        type=Marker.CYLINDER,
        action=Marker.ADD,
        pose=Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        scale=Vector3(x=x_scale, y=y_scale, z=0.05),
        color=ColorRGBA(r=0.3, g=0.1, b=0.1, a=0.1),
        lifetime=lifetime,
    )


def line_marker_msg(
    path_markers: list,
    path_colours: list,
) -> Marker:
    header = Header(frame_id="map")
    return Marker(
        header=header,
        ns="current_path",
        id=0,
        type=Marker.LINE_STRIP,
        action=Marker.ADD,
        pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        scale=Vector3(x=0.2, y=0.1, z=0.1),
        points=path_markers,
        colors=path_colours,
        lifetime=Duration(sec=10, nanosec=100000),
    )


def clear_marker_msg(id_: int, header: Header, name_space: str = "current_scan") -> Marker:
    return Marker(
        header=header,
        ns=name_space,
        id=id_,
        action=Marker.DELETE,
    )
