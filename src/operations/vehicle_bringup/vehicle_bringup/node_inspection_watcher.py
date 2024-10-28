import rclpy
import rclpy.logging
from rclpy.node import Node

# from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class NodeWatcher(Node):
    def __init__(self):
        super().__init__("inspection_watcher")
        self.srv = self.create_service(SetBool, "launch/inspection", self.ebs_callback)
        self.timer = self.create_timer(1, self.timer_callback)
        self.exit = False

    def ebs_callback(self, request, response):
        if request.data:
            self.get_logger().info("Inspection Launch request received")
            response.success = True
            self.exit = True
            return response
        else:
            response.success = True
            response.message = "False received. Inspection not launched"
        return response

    def timer_callback(self):
        if self.exit:
            self.get_logger().info("Inspection launched")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = NodeWatcher()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("inspection_watcher").info("Inspection launched")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
