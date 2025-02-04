import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped


class DummyTrackPublisher(Node):
    def __init__(self):
        super().__init__("dummy_track_publisher")
        self.publisher_ = self.create_publisher(ConeDetectionStamped, "slam/cone_detection", 10)
        self.timer = self.create_timer(1.0, self.publish_dummy_track)

    def publish_dummy_track(self):
        msg = ConeDetectionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Define the track points
        blue_cones = [
            {"x": 0.0, "y": 0.0},
            {"x": 2.0, "y": 0.5},
            {"x": 4.0, "y": 1.8},
            {"x": 6.0, "y": 4.0},
            {"x": 8.0, "y": 6.0},
            {"x": 10.0, "y": 8.0},
            {"x": 12.0, "y": 11.0},
            {"x": 10.0, "y": 14.0},
        ]

        yellow_cones = [
            {"x": 0.0, "y": 5.0},
            {"x": 3.0, "y": 8.0},
            {"x": 5.0, "y": 10.0},
            {"x": 4.0, "y": 13.0},
        ]

        orange_cones = []

        # Add blue cones
        for point in blue_cones:
            cone = Cone()
            cone.location.x = point["x"]
            cone.location.y = point["y"]
            cone.color = Cone.BLUE
            msg.cones.append(cone)

        # Add yellow cones
        for point in yellow_cones:
            cone = Cone()
            cone.location.x = point["x"]
            cone.location.y = point["y"]
            cone.color = Cone.YELLOW
            msg.cones.append(cone)

        # Add orange cones
        for point in orange_cones:
            cone = Cone()
            cone.location.x = point["x"]
            cone.location.y = point["y"]
            cone.color = point["color"]
            msg.cones.append(cone)

        self.publisher_.publish(msg)
        self.get_logger().info("Published dummy track")


def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
