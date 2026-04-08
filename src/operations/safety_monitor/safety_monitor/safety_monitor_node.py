import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        # Physical limits
        self.MAX_SPEED = 20.0 # Meters per second
        self.MAX_STEERING = 0.8 # Radians (~45 degrees)
        self.system_locked = False # State for E-Stop

        # Watchdog parameters
        self.last_command_time = time.time()
        self.TIMEOUT_LIMIT = 0.5 # Seconds. Trigger E-Stop if no message for 500ms

        self.health_status = {
            "lidar": False,
            "planner": False,
            "vcu": False,
            "slam": False
        }
        
        # Create a subscriber to the driving command topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/control/driving_command',
            self.listener_callback,
            10) # Queue size is 10
        
        self.watchdog_timer = self.create_timer(0.1, self.check_watchdogs)
            
        self.get_logger().info('Safety Monitor active')

    def listener_callback(self, msg):
        # Ignore new commands if E-Stop engaged
        if self.system_locked:
            return
        
        # Update heartbeat timestamp
        self.last_command_time = time.time()
        
        speed = msg.drive.speed
        steering = msg.drive.steering_angle
        
        # Check for safety violations
        if abs(speed) > self.MAX_SPEED:
            self.trigger_estop(f'Overspeed Detected: {speed:.2f} m/s')
        elif abs(steering) > self.MAX_STEERING:
            self.trigger_estop(f'Oversteer Detected: {steering:.2f} rad')
        else:
            self.get_logger().info(f'Monitoring -> Speed: {speed:.2f} m/s, Steering: {steering:.2f} rad')

    def lidar_callback(self, msg):
        # LiDaR is healthy if we recieved a message
        self.health_status["lidar"] = True
        self.last_lidar_received = self.get_clock().now()

    def planner_callback(self, msg):
        # Permorm math check
        speed_ok = abs(msg.drive.speed) <= self.MAX_SPEED

        # Update based on math
        self.health_status["lidar"] = speed_ok
        self.last_lidar_received = self.get_clock().now()

    def check_watchdogs(self):
        if self.system_locked:
            return
        
        current_time = time.time()
        time_since_last_msg = current_time - self.last_command_time

        # Kill the car if planner stops talking
        if time_since_last_msg > self.TIMEOUT_LIMIT:
            self.trigger_estop(f"Lost communication with Planner! (Timeout: {time_since_last_msg:.2f}s)")

    def health_check_loop(self):
        if self.system_locked:
            return
        
        now = self.get_clock().now()

        # Check for timeouts and set health to False if a node is silent
        if (now - self.last_lidar_received).nanoseconds > 0.5 * 1e9: # 500ms
            self.health_status["lidar"] = False
            
        if (now - self.last_planner_received).nanoseconds > 0.2 * 1e9: # 200ms
            self.health_status["planner"] = False
        
        # Returns True if all values are True
        if not all(self.health_status.values()):
            # Find failed for logs
            failed_systems = [k for k, v in self.health_status.items() if not v]
            self.trigger_estop(f"System Health Failure: {failed_systems}")

    def trigger_estop(self, reason):
        # Update system lock
        self.system_locked = True

        # Update logs
        self.get_logger().error(f'E-STOP TRIGGERED! Reason: {reason}')

        # TODO VCU cut power


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