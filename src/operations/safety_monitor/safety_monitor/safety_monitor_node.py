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

    def check_watchdogs(self):
        if self.system_locked:
            return
        
        current_time = time.time()
        time_since_last_msg = current_time - self.last_command_time

        # Kill the car if planner stops talking
        if time_since_last_msg > self.TIMEOUT_LIMIT:
            self.trigger_estop(f"Lost communication with Planner! (Timeout: {time_since_last_msg:.2f}s)")

    def trigger_estop(self, reason):
        # Update system lock
        self.system_locked = True

        # Update logs
        self.get_logger().error('CRITICAL SAFETY VIOLATION!')
        self.get_logger().error(f'Attempted Speed: {speed:.2f}, Attempted Steering: {steering:.2f}')
        self.get_logger().error('E-STOP TRIGGERED. Locking system.')

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