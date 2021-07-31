# ROS2 libraries
import rclpy
from rclpy.node import Node
# ROS2 message libraries
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
# custom sim data message libraries
from qutms_msgs.msg import ConeScan, ConeData
# helper math function
from .sub_module.simple_lidar import find_points

class Lidar_Pipe(Node):
    def __init__(self):
        super().__init__('lidar_pipe')
        ## creates subscriber to 'Lidar2' with type PointCloud2 that sends data to lidar_callback
        self.pcl_subscription_ = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar1',
            self.pcl_callback,
            10)
        self.pcl_subscription_  # prevent unused variable warning
        self.cones = []

        ## creates publisher to 'control_command' with type ControlCommand
        self.scan_publisher_ = self.create_publisher(
            ConeScan,
            'lidar_output', 
            10)
        # creates timer for publishing commands
        self.timer_period = 0.001  # seconds
        self.timer = self.create_timer(self.timer_period, self.publisher)

        # cone point vars
        self.max_range_cutoff = 15 # m
        self.distance_cutoff = 0.1 # m


    ## callback for lidar data to be sent to. used to call funtion to find cone coords
    def pcl_callback(self, pcl_msg):
        """ In here, we will call calculations to ideally get the xyz location 
        and reflectivity of the cones"""
        
        self.cones = find_points(pcl_msg, self.max_range_cutoff, self.distance_cutoff)
        

    def publisher(self):
        cone_scan = ConeScan()
        # head = Header()

        cone_data = []
        for i in range(len(self.cones)):
            cone = ConeData()
            cone.x = self.cones[i][0]
            cone.y = self.cones[i][1]
            cone.z = self.cones[i][2]
            cone.i = self.cones[i][3]
            cone_data.append(cone)

        #     self.get_logger().info('cones: "%s"' % [self.cones[i][0], self.cones[i][1], self.cones[i][2], self.cones[i][3]])
        # self.get_logger().info('cones:\n')
        
        # head.stamp = rospy.Time.now()
        # head.frame_id = "lidar2"
        # cone_scan.header = head

        cone_scan.data = cone_data

        self.scan_publisher_.publish(cone_scan)
        

## main call
def main(args=None):
    rclpy.init(args=args)

    control = Lidar_Pipe()
    rclpy.spin(control)
    
    control.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
