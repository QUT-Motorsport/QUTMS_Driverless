import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import WSSVelocity
from sensor_msgs.msg import Image, Imu

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

vid = cv2.VideoCapture("/home/ubuntu/QUTMS_Driverless/src/common/data_overlay/videos/ebs_test1.mp4")
if vid.isOpened() == False:
    print("Error opening video stream or file")


class DataOverlay(Node):
    x_accel: Optional[float] = None
    y_accel: Optional[float] = None
    velocity: Optional[float] = None

    ebs_timer: float = 0.0

    def __init__(self):
        super().__init__("data_overlay_node")

        # subscribe to velocity
        self.create_subscription(WSSVelocity, "/vehicle/wss_velocity", self.velocity_callback, 10)
        # subscribe to imu
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        # timer to create frames
        self.create_timer(1 / fps, self.timer_callback)

        # publish the image
        self.image_publisher: Publisher = self.create_publisher(Image, "/debug_imgs/data_overlay", 1)

        self.get_logger().info("---Data overlay node initialised---")

    def velocity_callback(self, msg: WSSVelocity):
        self.velocity = msg.velocity * 3.6

    def imu_callback(self, msg: Imu):
        self.x_gs = msg.linear_acceleration.x / 9.81
        self.y_gs = msg.linear_acceleration.y / 9.81

    def timer_callback(self):
        # randomise the data
        self.x_accel = np.random.uniform(-2, 2)
        self.y_accel = np.random.uniform(-2, 2)
        self.velocity = np.random.uniform(0, 50)

        if not self.x_accel or not self.y_accel or not self.velocity:
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
            "X: " + str(round(self.x_accel, 2)),
            (int(circle_center[0] - circle_radius / 2 - 30), circle_center[1] - circle_radius),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        # draw a text to show the y acceleration beside circle
        cv2.putText(
            img,
            "Y: " + str(round(self.y_accel, 2)),
            (circle_center[0] + circle_radius, int(circle_center[1] + circle_radius / 2 - 40)),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        # draw the x and y acceleration as a dot
        cv2.circle(
            img,
            (int(circle_center[0] + self.x_accel * 50), int(circle_center[1] + self.y_accel * 50)),
            5,
            (0, 0, 255),
            -1,
        )
        # draw a line to the dot
        cv2.line(
            img,
            (circle_center[0], circle_center[1]),
            (int(circle_center[0] + self.x_accel * 50), int(circle_center[1] + self.y_accel * 50)),
            (0, 0, 255),
            2,
        )

        vel_bar = (int(frame_width / 7 * 5), int(top_area - 50))

        # make a vertical bar to show velocity
        cv2.rectangle(img, vel_bar, (vel_bar[0] + 30, int(top_area - 50 - self.velocity * 4)), (0, 0, 255), -1)
        # draw a text to show velocity
        cv2.putText(
            img,
            str(round(self.velocity, 2)) + " km/h",
            (int(vel_bar[0] + 40), int(top_area - 50)),
            cv2.FONT_HERSHEY_SIMPLEX,
            3,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        # draw the ebs timer
        if self.ebs_timer > 11.7:
            if self.ebs_timer % 0.5 < 0.25:
                # draw a text on a background
                cv2.putText(
                    img,
                    "EBS ACTIVATED",
                    (int(frame_width / 2) - 350, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    3,
                    (0, 0, 255),
                    4,
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
