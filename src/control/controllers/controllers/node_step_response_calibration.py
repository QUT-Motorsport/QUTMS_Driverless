import math
from pathlib import Path
import random
import time

import cv2
import matplotlib as mlb
import matplotlib.pyplot as plt
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import SteeringReading
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from driverless_common.shutdown_node import ShutdownNode


class StepController(Node):
    target: float = 0.0

    change_interval = 2.5  # s
    pub_interval = 0.01  # s

    left_turn = np.empty((0, 2), float)
    right_turn = np.empty((0, 2), float)
    previous_steering = 0
    previous_encoding = 0

    step = 0
    max_ang = 50.0
    inc = 750

    def __init__(self):
        super().__init__("step_controller_node")

        # timed callback
        self.create_timer(self.change_interval, self.change_callback)
        self.create_timer(self.pub_interval, self.pub_callback)

        self.create_subscription(SteeringReading, "/vehicle/steering_reading", self.sub_callback_st, 1)
        self.create_subscription(Int32, "/vehicle/encoder_reading", self.sub_callback_en, 1)

        self.drive_publisher: Publisher = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.model_img_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/model_calibration_image", 1)

        self.get_logger().info("---Step Controller Node Initalised---")

    def pub_callback(self):
        control_msg = AckermannDriveStamped()
        control_msg.drive.steering_angle = self.target
        self.drive_publisher.publish(control_msg)

    def sub_callback_st(self, msg_variable_name: SteeringReading):
        self.previous_steering = msg_variable_name.steering_angle

    def sub_callback_en(self, msg_variable_name: Int32):
        self.previous_encoding = msg_variable_name.data

    def change_callback(self):
        print(self.step)
        if int(self.target) == 0 and self.step == 0:
            self.step = 1
            print(self.target)
        elif int(self.target) > -7500 and self.step == 1:
            self.left_turn = np.append(self.left_turn, [[self.previous_steering, self.previous_encoding]], axis=0)
            self.target -= self.inc
            print(self.target)
        elif int(self.target) == -7500:
            self.right_turn = np.append(self.right_turn, [[self.previous_steering, self.previous_encoding]], axis=0)
            self.target = 0.0
            self.step = 2
            print(self.target)
        elif int(self.target) < 7500 and self.step == 2:
            self.right_turn = np.append(self.right_turn, [[self.previous_steering, self.previous_encoding]], axis=0)
            self.target += self.inc
            print(self.target)
        elif int(self.target) == 7500:
            self.target = 0.0
            self.step = 3
            print(self.target)

        if self.step == 3:
            self.left_turn = self.left_turn.transpose()
            self.right_turn = self.right_turn.transpose()
            left_fit = np.polyfit(self.left_turn[0], self.left_turn[1], deg=1)
            right_fit = np.polyfit(self.right_turn[0], self.right_turn[1], deg=1)

            pl = np.poly1d(left_fit)
            pr = np.poly1d(right_fit)

            pl_test = np.linspace(-90, 90, 100)
            pr_test = np.linspace(-90, 90, 100)

            print(left_fit)
            print(right_fit)

            fig, axs = plt.subplots(2)
            fig.suptitle("Linear Regression of Turning Model")
            fig.supxlabel("Steering Angle")
            fig.supylabel("Encoder Ticks")
            axs[0].scatter(self.left_turn[0], self.left_turn[1])
            axs[0].plot(pl_test, pl(pl_test), "-")
            axs[0].set_title("Left Turn Model")
            axs[0].set_xlim([-5, 90])
            axs[0].set_ylim([-8000, 0])

            axs[1].scatter(self.right_turn[0], self.right_turn[1])
            axs[1].plot(pr_test, pr(pr_test), "-")
            axs[1].set_title("Right Turn Model")
            axs[1].set_xlim([-90, 5])
            axs[1].set_ylim([0, 8000])

            img_dir = Path("./src/csv_data")
            img_dir.mkdir(parents=True, exist_ok=True)
            plt.savefig(img_dir / "step_response_calibration.png")
            plt.close()

            # load image as cv2 for ros image showing
            img = cv2.imread(str(img_dir / "step_response_calibration.png"))
            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.model_img_publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StepController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
