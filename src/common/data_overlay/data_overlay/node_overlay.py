from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import WSSVelocity
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32

from typing import Optional

cv_bridge = CvBridge()

# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = 1920
frame_height = 1080
fps = 60

# Define the codec and create VideoWriter object.
out = cv2.VideoWriter(
    "/home/ubuntu/QUTMS_Driverless/src/common/data_overlay/videos/overlay.mp4",
    cv2.VideoWriter_fourcc(*"MP4V"),
    fps,
    (frame_width, frame_height),
)

vid = cv2.VideoCapture("/home/ubuntu/QUTMS_Driverless/src/common/data_overlay/videos/ebs_test.mp4")
if vid.isOpened() == False:
    print("Error opening video stream or file")


class DataOverlay(Node):
    x_gs: float = 0.0
    y_gs: float = 0.0
    velocity: float = 0.0

    vel_received: bool = False
    imu_received: bool = False

    ebs_timer: float = 0.0

    def __init__(self):
        super().__init__("data_overlay_node")

        # subscribe to velocity
        self.create_subscription(WSSVelocity, "/vehicle/wheel_speed", self.velocity_callback, 10)
        # subscribe to imu
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        # timer to create frames
        self.create_timer(1 / fps, self.timer_callback)

        # publish the image
        self.image_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/data_overlay", 1)
        self.vel_publisher: Publisher = self.create_publisher(Float32, "/vehicle/velocity", 1)
        self.accel_publisher: Publisher = self.create_publisher(Float32, "/vehicle/acceleration", 1)

        self.get_logger().info("---Data overlay node initialised---")

    def velocity_callback(self, msg: WSSVelocity):
        self.vel_publisher.publish(Float32(data=self.velocity))
        self.velocity = msg.velocity * 3.6
        self.vel_received = True

    def imu_callback(self, msg: Imu):
        self.accel_publisher.publish(Float32(data=self.x_gs))
        self.x_gs = -msg.linear_acceleration.x + 0.6
        self.y_gs = -msg.linear_acceleration.y
        self.imu_received = True

    def timer_callback(self):
        # randomise the data
        # self.x_gs = np.random.uniform(-2, 2)
        # self.y_gs = np.random.uniform(-2, 2)
        # self.velocity = np.random.uniform(0, 50)
        # self.vel_received = True
        # self.imu_received = True

        if not self.vel_received or not self.imu_received:
            return

        if not vid.isOpened():
            print("Error opening video stream or file")
            return

        # read the next frame
        ret, img = vid.read()
        if not ret:
            self.destroy_node()
            raise Exception("Finished")

        # create a transparent overlay
        overlay = img.copy()
        overlay = cv2.rectangle(overlay, (0, 0), (frame_width, int(frame_height / 3)), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.6, img, 0.4, 0)

        top_area = int(frame_height / 3)

        circle_center = (int(frame_width / 9), int(top_area / 2))
        circle_radius = 120
        min_radius = int(circle_radius / 3)
        mid_radius = int((circle_radius - min_radius) / 2 + min_radius)

        # create a circle to show g forces
        cv2.circle(img, circle_center, circle_radius, (255, 255, 255), 4)
        cv2.circle(img, circle_center, mid_radius, (255, 255, 255), 4)
        cv2.circle(img, circle_center, min_radius, (255, 255, 255), 4)
        # draw a cross in the middle
        cv2.line(
            img,
            (circle_center[0] - circle_radius, circle_center[1]),
            (circle_center[0] + circle_radius, circle_center[1]),
            (255, 255, 255),
            3,
        )
        cv2.line(
            img,
            (circle_center[0], circle_center[1] - circle_radius),
            (circle_center[0], circle_center[1] + circle_radius),
            (255, 255, 255),
            3,
        )
        # draw a text to show the x acceleration above circle
        cv2.putText(
            img,
            "X: " + str(round(self.x_gs, 2)),
            (int(circle_center[0] - circle_radius / 2 - 30), circle_center[1] - circle_radius),
            cv2.FONT_HERSHEY_DUPLEX,
            2,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        # draw a text to show the y acceleration beside circle
        cv2.putText(
            img,
            "Y: " + str(round(self.y_gs, 2)),
            (circle_center[0] + circle_radius, int(circle_center[1] + circle_radius / 2 - 40)),
            cv2.FONT_HERSHEY_DUPLEX,
            2,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        # draw the x and y acceleration as a dot
        scale_factor = circle_radius / 6
        disp_x = max(min(self.x_gs * scale_factor, circle_radius), -circle_radius)
        disp_y = max(min(self.y_gs * scale_factor, circle_radius), -circle_radius)
        cv2.circle(
            img,
            (int(circle_center[0] + disp_y), int(circle_center[1] + disp_x)),
            8,
            (3, 82, 252),
            5,
        )
        # draw a line to the dot
        cv2.line(
            img,
            (circle_center[0], circle_center[1]),
            (int(circle_center[0] + disp_y), int(circle_center[1] + disp_x)),
            (3, 119, 252),
            5,
        )

        vel_bar = (1700, int(top_area - 50))
        vel_txt = (1000, vel_bar[1])

        # make a vertical bar to show velocity
        cv2.rectangle(img, vel_bar, (vel_bar[0] + 60, int(top_area - 50 - self.velocity * 6)), (0, 255, 0), -1)
        # draw a text to show velocity
        vel_col = (255, 255, 255)
        if self.velocity > 40:
            vel_col = (0, 255, 0)
        cv2.putText(
            img,
            str(round(self.velocity, 2)) + " km/h",
            vel_txt,
            cv2.FONT_HERSHEY_TRIPLEX,
            3,
            vel_col,
            3,
            cv2.LINE_AA,
        )

        # draw the ebs timer with roboto PIL font
        if self.ebs_timer > 10.7:
            if self.ebs_timer % 0.5 < 0.25:
                cv2.putText(
                    img,
                    "EBS ACTIVATED",
                    (int(frame_width / 2) - 350, 100),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    3,
                    (23, 34, 235),
                    6,
                    cv2.LINE_AA,
                )

        # publish the image
        self.image_publisher.publish(cv_bridge.cv2_to_imgmsg(img, "bgr8"))

        # write the frame
        out.write(img)

        self.ebs_timer += 1 / fps


def main():
    rclpy.init()
    node = DataOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
