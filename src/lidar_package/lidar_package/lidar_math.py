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
from std_msgs.msg import Float32

# import simple_lidar

class LidarProcessing(Node):
    def __init__(self):
        super().__init__('lidar_processing')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/fsds/lidar/Lidar2',
            self.listener_callback,
            10)
        self.lidar_subscription  # prevent unused variable warning

        self.math_publisher = self.create_publisher(
            Float32, 
            'math_output', 
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.left = 0

    def listener_callback(self, pcl2):
        self.frame = pcl2.data
    """ In here, we will call calculations to ideally get the [distance, angle, reflectivity] of the cone"""

    def timer_callback(self):
        # example steering code back right and left
        if self.left == 0:
            if self.i < 0.5:
                self.i += 0.05
            elif self.i >= 0.5:
                self.left = 1

        elif self.left == 1:
            if self.i > -0.5:
                self.i -= 0.05
            elif self.i <= -0.5:
                self.left = 0

        msg = Float32()
        msg.data = self.i
        self.math_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    lidar_processing = LidarProcessing()
    rclpy.spin(lidar_processing)
    
    lidar_processing.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
