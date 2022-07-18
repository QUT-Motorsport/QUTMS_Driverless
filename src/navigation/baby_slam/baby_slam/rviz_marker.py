from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


def cov_marker(
    x_coord: float,
    y_coord: float,
    ID: int,
    header: Header,
    x_scale: float,  # x sigma
    y_scale: float,  # y sigma
) -> Marker:

    marker = Marker()
    marker.header = header
    marker.ns = "cov_markers"
    marker.id = ID
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = x_scale
    marker.scale.y = y_scale
    marker.scale.z = 0.05

    marker.color.a = 0.3  # alpha
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.1

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker


def cone_marker(
    x_coord: float,
    y_coord: float,
    ID: int,
    header: Header,
    colour: tuple,
) -> Marker:

    marker = Marker()
    marker.header = header
    marker.ns = "cone_markers"
    marker.id = ID
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position.x = x_coord
    marker.pose.position.y = y_coord
    marker.pose.position.z = 0.14
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # scale out of 1x1x1m
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.28

    marker.color.a = 0.5  # alpha
    marker.color.r = colour[2] / 255  # scale 0-255 to 0-1
    marker.color.g = colour[1] / 255
    marker.color.b = colour[0] / 255

    marker.lifetime = Duration(sec=1, nanosec=0)

    return marker
