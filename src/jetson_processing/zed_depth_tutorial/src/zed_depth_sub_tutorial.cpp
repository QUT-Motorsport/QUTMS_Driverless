/*
 * MIT License
 * 
 * Copyright (c) 2020 Stereolabs
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This tutorial demonstrates simple receipt of ZED depth messages over the ROS system.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;

class MinimalDepthSubscriber : public rclcpp::Node {
  public:
    MinimalDepthSubscriber()
        : Node("zed_depth_tutorial") {

        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

        rclcpp::QoS depth_qos(10);
        depth_qos.keep_last(10);
        depth_qos.best_effort();
        depth_qos.durability_volatile();

        // Create depth map subscriber
        mDepthSub = create_subscription<sensor_msgs::msg::Image>(
                   "depth", depth_qos,
                   std::bind(&MinimalDepthSubscriber::depthCallback, this, _1) );
    }

  protected:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float* depths = (float*)(&msg->data[0]);

        // Image coordinates of the center pixel
        int u = msg->width / 2;
        int v = msg->height / 2;

        // Linear index of the center pixel
        int centerIdx = u + msg->width * v;

        // Output the measure
        RCLCPP_INFO(get_logger(), "Center distance : %g m", depths[centerIdx]);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
};

// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<MinimalDepthSubscriber>();

    rclcpp::spin(depth_node);
    rclcpp::shutdown();
    return 0;
}
