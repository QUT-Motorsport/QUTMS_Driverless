from math import pi

from transforms3d.euler import euler2quat, quat2euler

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw(quat: Quaternion):
    # Convert quaternion to euler angles and return the yaw before processing
    return quat2euler([quat.w, quat.x, quat.y, quat.z])[2]  # yaw


def orientation(x, y, z):
    new_orientation = euler2quat(x, y, z)
    return Quaternion(w=new_orientation[0], x=new_orientation[1], y=new_orientation[2], z=new_orientation[3])


class OdometryRotater(Node):
    def __init__(self):
        super().__init__("odometry_transformer")

        self.subscription = self.create_subscription(Odometry, "imu/odometry", self.odometry_callback, 10)

        self.publisher = self.create_publisher(Odometry, "imu/local_odometry", 10)

        self.get_logger().info("---Odom transformer node initialised---")

    def odometry_callback(self, msg: Odometry):
        current_heading = yaw(msg.pose.pose.orientation)

        rotated_odom = msg
        # rotate the heading by 90 degrees
        relative_heading = current_heading + pi / 2
        rotated_odom.pose.pose.orientation = orientation(0, 0, relative_heading)

        self.publisher.publish(rotated_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryRotater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
