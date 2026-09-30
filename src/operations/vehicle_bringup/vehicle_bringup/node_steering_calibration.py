"""steering calibration mission

holds one steering command per run while the car is pushed forward. after run_distance the steering
snaps to opposite lock to tell the push crew to stop, and waits. wheel the car back, then start the next run:
    ros2 service call /calibration/next std_srvs/srv/Trigger
fit the recorded bag with tools/steering_calibration/fit_steering_calibration.py
"""

import math

import rclpy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from std_srvs.srv import Trigger

from vehicle_bringup.shutdown_node_class import ShutdownNode

DEFAULT_ANGLES = [-20.0, -10.0, -30.0, 0.0, -40.0, -15.0, -25.0, -5.0, -35.0, 20.0, -60.0, 40.0, -80.0, 60.0, 80.0, -10.0]


class SteeringCalibration(ShutdownNode):
    def __init__(self):
        super().__init__("steering_calibration_node")
        self.declare_parameter("angles", DEFAULT_ANGLES)
        self.declare_parameter("run_distance", 10.0)
        self.declare_parameter("settle_time", 4.0)
        self.declare_parameter("min_speed", 0.3)
        self.declare_parameter("stop_lock", 80.0)
        self.declare_parameter("rough_centre", -20.0)
        self.declare_parameter("record", True)
        self.angles = list(self.get_parameter("angles").value)
        self.run_distance = self.get_parameter("run_distance").value
        self.settle_time = self.get_parameter("settle_time").value
        self.min_speed = self.get_parameter("min_speed").value

        self.run = 0
        self.measuring = True
        self.finished = False
        self.distance = 0.0
        self.run_start = self.get_clock().now()
        self.last_odom = None

        self.steering_pub = self.create_publisher(AckermannDriveStamped, "/control/driving_command", 1)
        self.run_pub = self.create_publisher(Int32, "/calibration/run", 1)
        self.create_subscription(Odometry, "/imu/odometry", self.odom_callback, 10)
        self.create_service(Trigger, "calibration/next", self.next_callback)
        self.create_timer(0.05, self.timer_callback)

        self.recording = self.get_parameter("record").value
        if self.recording:
            self.start_recording("steering_calibration")
        self.log_run()
        self.get_logger().info("---Steering calibration node initialised---")

    def log_run(self):
        self.get_logger().info(
            f"Run {self.run + 1}/{len(self.angles)}: steering {self.angles[self.run]:.0f}, "
            f"push forward {self.run_distance:.0f} m"
        )

    def stop_command(self):
        """opposite lock from the run command"""
        lock = self.get_parameter("stop_lock").value
        return lock if self.angles[self.run] < self.get_parameter("rough_centre").value else -lock

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = math.cos(yaw) * msg.twist.twist.linear.x + math.sin(yaw) * msg.twist.twist.linear.y
        if self.last_odom is not None and self.measuring:
            settled = (self.get_clock().now() - self.run_start).nanoseconds * 1e-9 > self.settle_time
            if settled and v > self.min_speed:
                self.distance += v * max(0.0, t - self.last_odom)
            if self.distance >= self.run_distance:
                self.measuring = False
                self.get_logger().info(
                    f"Run {self.run + 1} done, stop and wheel back, then call /calibration/next"
                )
        self.last_odom = t

    def next_callback(self, request, response):
        if self.run + 1 >= len(self.angles):
            if self.recording and not self.finished:
                self.stop_record_future = self.bag_stop_cli.call_async(Trigger.Request())
            self.finished = True
            self.measuring = False
            response.success = False
            response.message = "calibration finished, bag closed"
            self.get_logger().info(response.message)
            return response
        self.run += 1
        self.measuring = True
        self.distance = 0.0
        self.run_start = self.get_clock().now()
        self.log_run()
        response.success = True
        response.message = f"run {self.run + 1}/{len(self.angles)}: steering {self.angles[self.run]:.0f}"
        return response

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(self.angles[self.run] if self.measuring else self.stop_command())
        self.steering_pub.publish(msg)
        self.run_pub.publish(Int32(data=self.run if self.measuring else -1))


def main(args=None):
    rclpy.init(args=args)
    node = SteeringCalibration()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
