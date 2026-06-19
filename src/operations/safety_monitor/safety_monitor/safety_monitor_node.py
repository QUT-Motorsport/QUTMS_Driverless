import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from driverless_msgs.msg import Shutdown 
import time
import math

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        # Physical limits
        self.MAX_SPEED = 20.0 
        self.MAX_STEERING = 0.8 
        self.MAX_POSE_JUMP = 3.0 # Meters
        self.system_locked = False 

        # Watchdog parameters
        self.first_command_received = False
        self.last_command_time = time.time()
        self.TIMEOUT_LIMIT = 0.5 

        # Oscillation parameters
        self.last_steering = 0.0
        self.oscillation_count = 0

        # Odometry parameters
        self.last_pose = None

        self.health_status = {
            "lidar": False,
            "planner": False,
            "vcu": False,
            "slam": False
        }
        
        # Subscribers
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/control/driving_command', self.listener_callback, 10)
            
        self.odom_sub = self.create_subscription(
            Odometry, 'imu/odometry', self.odom_callback, 10)
            
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/cone_points', self.lidar_callback, 10)
            
        # Publisher to actively trigger the system shutdown
        self.shutdown_pub = self.create_publisher(
            Shutdown, 'system/shutdown', 1)
        
        self.watchdog_timer = self.create_timer(0.1, self.check_watchdogs)
            
        self.get_logger().info('Safety Monitor active')

    def listener_callback(self, msg):
        if self.system_locked:
            return
        
        self.first_command_received = True
        self.last_command_time = time.time()
        self.health_status["planner"] = True
        
        speed = msg.drive.speed
        steering = msg.drive.steering_angle
        
        # Bounds Checking
        if abs(speed) > self.MAX_SPEED:
            self.trigger_estop(f'Overspeed Detected: {speed:.2f} m/s')
        elif abs(steering) > self.MAX_STEERING:
            self.trigger_estop(f'Oversteer Detected: {steering:.2f} rad')
            
        # Oscillation (Death Wobble) Detection
        if (self.last_steering > 0.5 and steering < -0.5) or (self.last_steering < -0.5 and steering > 0.5):
            self.oscillation_count += 1
            if self.oscillation_count >= 3:
                self.trigger_estop('Steering Oscillation (Death Wobble) Detected')
        else:
            self.oscillation_count = max(0, self.oscillation_count - 1)
            
        self.last_steering = steering

    def odom_callback(self, msg):
        if self.system_locked:
            return
            
        self.health_status["slam"] = True
        current_pose = msg.pose.pose.position
        
        # NaN Check
        if math.isnan(current_pose.x) or math.isnan(current_pose.y):
            self.trigger_estop('NaN value detected in Odometry')
            return
            
        # Teleportation Spike Check
        if self.last_pose:
            dist = math.sqrt((current_pose.x - self.last_pose.x)**2 + (current_pose.y - self.last_pose.y)**2)
            if dist > self.MAX_POSE_JUMP:
                self.trigger_estop(f'Odometry Jump Detected: {dist:.2f}m')
                
        self.last_pose = current_pose

    def lidar_callback(self, msg):
        # Ensure the point cloud isn't completely empty
        if msg.width * msg.height == 0:
            self.health_status["lidar"] = False
            self.trigger_estop('Sensor Blindness: LiDAR published empty point cloud')
        else:
            self.health_status["lidar"] = True
            self.last_lidar_received = self.get_clock().now()

    def check_watchdogs(self):
        if self.system_locked or not self.first_command_received:
            return
        
        current_time = time.time()
        time_since_last_msg = current_time - self.last_command_time

        if time_since_last_msg > self.TIMEOUT_LIMIT:
            self.health_status["planner"] = False
            self.trigger_estop(f"Lost communication with Planner! (Timeout: {time_since_last_msg:.2f}s)")

    def trigger_estop(self, reason):
        if not self.system_locked:
            self.system_locked = True
            self.get_logger().error(f'E-STOP TRIGGERED! Reason: {reason}')
            
            # Broadcast the shutdown command to the network
            shutdown_msg = Shutdown()
            # Assumed shutdown structure
            self.shutdown_pub.publish(shutdown_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()