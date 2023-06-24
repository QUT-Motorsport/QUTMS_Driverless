import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

import time

from driverless_msgs.msg import Reset, Shutdown
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

from driverless_common.shutdown_node import ShutdownNode


class TrackdriveMission(ShutdownNode):
    started: bool = False
    laps: int = 0
    last_lap_time: float = 0

    def __init__(self):
        super().__init__("trackdrive_logic_node")
        self.create_subscription(Reset, "/system/reset", self.reset_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/slam/car_pose", self.pose_callback, 10)

        self.shutdown_pub: Publisher = self.create_publisher(Shutdown, "/system/shutdown", 1)
        self.lap_trig_pub: Publisher = self.create_publisher(Bool, "/system/lap_completed", 1)
        
        self.last_lap_time = time.time()

        self.get_logger().info("---Inspection mission node initialised---")

    def reset_callback(self, msg: Reset):
        if not self.started:
            self.started = True
            self.lap_trig_pub.publish(Bool(data=False))
            self.get_logger().info("Inspection mission started")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # check if car has crossed the finish line (0,0)
        # get distance from 0,0 and increment laps when within a certain threshold
        # and distance is increasing away from 0,0
        if self.started and msg.pose.pose.position.x < 0.3 and msg.pose.pose.position.y < 0.5:
            if time.time() - self.last_lap_time > 20:  # 20 seconds at least between laps
                self.laps += 1
                self.last_lap_time = time.time()
                self.lap_trig_pub.publish(Bool(data=True))
                self.get_logger().info(f"Lap {self.laps} completed")

        if self.laps == 10:
            self.get_logger().info("Inspection mission complete")
            shutdown_msg = Shutdown(finished_engage_ebs=True)
            self.shutdown_pub.publish(shutdown_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackdriveMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
