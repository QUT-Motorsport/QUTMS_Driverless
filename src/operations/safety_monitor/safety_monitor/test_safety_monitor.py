import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time

class SafetyMonitorTester(Node):
    def __init__(self, test_mode):
        super().__init__('safety_monitor_tester')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/control/driving_command', 10)
        self.test_mode = test_mode
        self.msg_count = 0
        
        print("Warming up DDS network...")
        time.sleep(1.0)
        
        # 10Hz heartbeat timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish_cmd(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published -> Speed: {speed}, Steering: {steering}')

    def timer_callback(self):
        self.msg_count += 1
        
        if self.test_mode == '1':
            # Normal operation, runs infinitely
            self.publish_cmd(10.0, 0.2)
            
        elif self.test_mode == '2':
            if self.msg_count < 10:
                self.publish_cmd(10.0, 0.0)
            elif self.msg_count == 10:
                self.publish_cmd(25.0, 0.0) # Trigger Overspeed
            else:
                raise SystemExit
                
        elif self.test_mode == '3':
            if self.msg_count < 10:
                self.publish_cmd(5.0, 0.0)
            elif self.msg_count == 10:
                self.publish_cmd(5.0, 1.2) # Trigger Oversteer
            else:
                raise SystemExit
                
        elif self.test_mode == '4':
            if self.msg_count < 10:
                self.publish_cmd(5.0, 0.0)
            elif self.msg_count == 10:
                self.publish_cmd(5.0, 0.6)
            elif self.msg_count == 11:
                self.publish_cmd(5.0, -0.6)
            elif self.msg_count == 12:
                self.publish_cmd(5.0, 0.6)
            elif self.msg_count == 13:
                self.publish_cmd(5.0, -0.6) # Trigger Death Wobble
            else:
                raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    
    print("\n--- Safety Monitor Test Suite ---")
    print("1: Normal Safe Operation (Infinite 10Hz heartbeat, Ctrl+C to stop)")
    print("2: Dangerous Overspeed")
    print("3: Dangerous Oversteer")
    print("4: Death Wobble Simulation")
    
    choice = input("\nEnter test number to execute: ")
    if choice not in ['1', '2', '3', '4']:
        print("Invalid choice. Exiting.")
        return

    tester = SafetyMonitorTester(choice)
    
    try:
        rclpy.spin(tester)
    except (KeyboardInterrupt, SystemExit):
        pass
        
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()