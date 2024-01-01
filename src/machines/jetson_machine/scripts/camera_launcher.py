from subprocess import Popen
import time

from rclpy.node import Node

from sensor_msgs.msg import Image


class ZEDHandler(Node):
    def __init__(self):
        super().__init__("zed_camera_handler")

        self.create_subscription(Image, "/zed2i/zed_node/rgb/image_rect_color", self.image_callback, 1)

        self.create_timer(1 / 5, self.timer_callback)

        self.process = None
        self.last_time = time.time()

    def image_callback(self):
        self.last_time = time.time()

    def timer_callback(self):
        if self.process is None:
            self.launch_zed_driver()

        if time.time() - self.last_time > 0.1:
            self.process.kill()
            self.process = None
            self.last_time = time.time()
            self.get_logger().info("Restarting ZED driver")

    def launch_zed_driver(self):
        command = ["stdbuf", "-o", "L", "ros2", "launch", "sensors", "zed_camera.launch.py"]
        self.get_logger().info(f"Command: {' '.join(command)}")
        self.process = Popen(command)
