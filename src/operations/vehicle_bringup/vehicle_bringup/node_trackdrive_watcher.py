import rclpy
import rclpy.logging
from rclpy.node import Node

# from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class NodeWatcher(Node):
    def __init__(self):
        super().__init__("trackdrive_watcher")
        self.srv = self.create_service(SetBool, "launch/trackdrive", self.ebs_callback)
        self.timer = self.create_timer(1, self.timer_callback)
        self.exit = False

    def ebs_callback(self, request, response):
        if request.data:
            self.get_logger().info("trackdrive Launch request received")
            response.success = True
            self.exit = True
            return response
        else:
            response.success = True
            response.message = "False received. trackdrive not launched"
        return response

    def timer_callback(self):
        if self.exit:
            self.get_logger().info("trackdrive launched")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = NodeWatcher()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("trackdrive_watcher").info("trackdrive launched")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()