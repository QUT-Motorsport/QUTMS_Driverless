from math import sqrt

from builtin_interfaces.msg import Duration
from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from typing import List

BIG_HEIGHT = 1.5
SMALL_HEIGHT = 1.0
MAX_NUM_CONES = 500

def marker_array_from_cone_detection(detection: ConeDetectionStamped, covariance: bool = False) -> MarkerArray:
    if covariance:  # is a cone with covariance list
        cones: List[Cone] = []
        covs: List[float] = []
        for cone_with_cov in detection.cones_with_cov:
            cones.append(cone_with_cov.cone)
            covs.append(cone_with_cov.covariance)
    else:
        cones: List[Cone] = detection.cones

    markers = []
    for i in range(MAX_NUM_CONES):
        if i < len(cones):
            if cones[i].color == Cone.ORANGE_BIG:
                z_scale = BIG_HEIGHT
            else:
                z_scale = SMALL_HEIGHT
            markers.append(
                marker_msg(
                    position=cones[i].location,
                    z_scale=z_scale,
                    id_=i,
                    header=detection.header,
                    cone_colour=cones[i].color,
                )
            )
            if covariance:
                markers.append(
                    cov_marker_msg(
                        position=cones[i].location,
                        id_=i,
                        x_scale=3 * sqrt(abs(covs[i][0])),
                        y_scale=3 * sqrt(abs(covs[i][3])),
                        header=detection.header,
                    )
                )
        else:
            markers.append(
                clear_marker_msg(
                    id_=i,
                    header=detection.header,
                )
            )
            if covariance:
                markers.append(
                    clear_marker_msg(
                        id_=i,
                        header=detection.header,
                        name_space="cone_covs",
                    )
                )
    return MarkerArray(markers=markers)


CONE_TO_RGB_MAP = {
    Cone.BLUE: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
    Cone.UNKNOWN: ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
    Cone.ORANGE_BIG: ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
    Cone.ORANGE_SMALL: ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
    Cone.YELLOW: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
}


def marker_msg(
    position: Point,
    z_scale: float,
    id_: int,
    header: Header,
    cone_colour: int,
    lifetime=Duration(sec=1, nanosec=0),
    alpha: float = 1.0,
) -> Marker:
    marker = Marker(
        header=header,
        ns="cones",
        id=id_,
        # type=Marker.MESH_RESOURCE,
        type=Marker.CYLINDER,
        # mesh_resource="package://driverless_common/meshes/cone.dae",
        # mesh_use_embedded_materials=False,
        action=Marker.ADD,
        pose=Pose(position=position, orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        # scale=Vector3(x=1.0, y=1.0, z=z_scale),
        scale=Vector3(x=0.2, y=0.2, z=z_scale * 0.3),
        color=CONE_TO_RGB_MAP.get(cone_colour, ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)),
        lifetime=lifetime,
    )
    marker.color.a = alpha  # yes, i had to do this
    return marker


def cov_marker_msg(
    position: Point,
    id_: int,
    x_scale: float,  # x sigma
    y_scale: float,  # y sigma
    lifetime=Duration(sec=1, nanosec=0),
    header=Header(frame_id="track"),
    z=0,
) -> Marker:
    return Marker(
        header=header,
        ns="cone_covs",
        id=id_,
        type=Marker.CYLINDER,
        action=Marker.ADD,
        pose=Pose(position=position, orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        scale=Vector3(x=x_scale, y=y_scale, z=0.05),
        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3),
        lifetime=lifetime,
    )


def clear_marker_msg(
    id_: int,
    header: Header,
    name_space: str = "cones",
) -> Marker:
    return Marker(
        header=header,
        ns=name_space,
        id=id_,
        action=Marker.DELETE,
    )
