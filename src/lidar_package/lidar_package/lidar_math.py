# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


<<<<<<< HEAD
class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        # self.lidar_subscription = self.create_subscription(
        #     PointCloud2,
        #     '/fsds/lidar/Lidar2',
        #     self.listener_callback,
        #     10)
        # self.lidar_subscription  # prevent unused variable warning

        self.math_publisher = self.create_publisher(
            String, 
            'math_output', 
            10)
=======
class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('math_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Scan: "%s"' % msg.data)


class MathPublisher(Node):
    def __init__(self):
        super().__init__('math_publisher')
        self.publisher_ = self.create_publisher(String, 'math_output', 10)
>>>>>>> master
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

<<<<<<< HEAD
    # def listener_callback(self, msg):
    #     self.get_logger().info('Scan: "%s"' % msg.data)

    def timer_callback(self):
        txt = String()
        txt.data = 'Distance is: %d' % self.i
        self.math_publisher.publish(txt)
        self.get_logger().info('Publishing: "%s"' % txt.data)
=======
    def timer_callback(self):
        msg = String()
        msg.data = 'Distance is' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
>>>>>>> master
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

<<<<<<< HEAD

    lidar_processing = LidarProcessing()
    rclpy.spin(lidar_processing)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_processing.destroy_node()
=======
    math_publisher = MathPublisher()
    rclpy.spin(math_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    math_publisher.destroy_node()

    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
>>>>>>> master

    rclpy.shutdown()


if __name__ == '__main__':
    main()
