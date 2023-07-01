import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from driverless_msgs.msg import Reset, Shutdown
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, UInt8

from driverless_common.shutdown_node import ShutdownNode


class TrackdriveHandler(ShutdownNode):
    r2d = False
    laps = 0
    last_lap_time = 0.0
    last_x = 0.0

    def __init__(self):
        super().__init__("trackdrive_logic_node")
        self.create_subscription(Bool, "/system/r2d", self.r2d_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.pose_callback, 10)

        self.shutdown_pub = self.create_publisher(Shutdown, "/system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "/system/laps_completed", 1)

        self.get_logger().info("---Trackdrive handler node initialised---")

    def r2d_callback(self, msg: Bool):
        if not self.r2d and msg.data:
            self.r2d = True
            self.last_lap_time = time.time()
            self.get_logger().info("Trackdrive mission started")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        if self.r2d and (self.last_x <= 0 and msg.pose.pose.position.x > 0) and abs(msg.pose.pose.position.y) < 2:
            if time.time() - self.last_lap_time > 20:  # 20 seconds at least between laps
                self.laps += 1
                self.last_lap_time = time.time()
                self.lap_trig_pub.publish(UInt8(data=self.laps))
                self.get_logger().info(f"Lap {self.laps} completed")

        if self.laps == 10:
            self.get_logger().info("Trackdrive mission complete")
            # currently only works when vehicle supervisor node is running on-car
            # TODO: sort out vehicle states for eventual environment agnostic operation
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)

        self.last_x = msg.pose.pose.position.x


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
