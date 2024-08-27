from math import cos, pi, radians, sin

from transforms3d.euler import euler2quat, quat2euler

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw(quat: Quaternion):
    # Convert quaternion to euler angles and return the yaw before processing
    return quat2euler([quat.w, quat.x, quat.y, quat.z])[2]  # yaw

    # Convert eulerian to quaternion for processed message publishing
def orientation(x, y, z):
    new_orientation = euler2quat(x, y, z)
    return Quaternion(w=new_orientation[0], x=new_orientation[1], y=new_orientation[2], z=new_orientation[3])

    # Processing eulerian angles
class OdometryTransformer(Node):
    def __init__(self):
        super().__init__("odometry_transformer")

        self.subscription = self.create_subscription(Odometry, "imu/odometry", self.odometry_callback, 10)

        self.initialised = False
        self.initial_easting = 0.0
        self.initial_northing = 0.0
        self.initial_heading = 0.0

        self.publisher = self.create_publisher(Odometry, "imu/local_odometry", 10)

    def odometry_callback(self, msg: Odometry):
        if not self.initialised:
            self.initial_easting = msg.pose.pose.position.x
            self.initial_northing = msg.pose.pose.position.y
            # Assuming orientation is in quaternion, converting to heading (yaw)
            self.initial_heading = yaw(msg.pose.pose.orientation) + pi / 2
            self.initialised = True

            print(f"Initialised at ({self.initial_easting}, {self.initial_northing}, {self.initial_heading})")
            return

        current_easting = msg.pose.pose.position.x
        current_northing = msg.pose.pose.position.y
        current_heading = yaw(msg.pose.pose.orientation)

        delta_easting = current_easting - self.initial_easting
        delta_northing = current_northing - self.initial_northing

        # relative_heading = -current_heading - self.initial_heading + pi / 2  # use when ENU is false (eg recorded data)
        relative_heading = current_heading - self.initial_heading + pi / 2

        x = delta_easting * cos(self.initial_heading) + delta_northing * sin(self.initial_heading)
        y = -delta_easting * sin(self.initial_heading) + delta_northing * cos(self.initial_heading)

        # Create a new Odometry message for the local frame
        local_odom = Odometry()
        local_odom.header = msg.header
        local_odom.pose.pose.position.x = x
        local_odom.pose.pose.position.y = y
        local_odom.pose.pose.position.z = msg.pose.pose.position.z
        # Adjust the orientation to be relative to the initial heading
        local_odom.pose.pose.orientation = orientation(0, 0, relative_heading)
        local_odom.twist = msg.twist

        self.publisher.publish(local_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTransformer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
