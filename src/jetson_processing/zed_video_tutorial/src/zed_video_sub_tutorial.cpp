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
 * This tutorial demonstrates simple receipt of ZED video messages over the ROS system.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

rclcpp::Node::SharedPtr g_node = nullptr;

/**
 * Subscriber callbacks. The argument of the callback is a constant pointer to the received message
 */


void imageRightRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(),
                "Right Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec,msg->header.stamp.nanosec);
}

void imageLeftRectifiedCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(g_node->get_logger(),
                "Left  Rectified image received from ZED\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec,msg->header.stamp.nanosec);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create the node
    g_node = rclcpp::Node::make_shared("zed_video_tutorial");


    /* Note: it is very important to use a QOS profile for the subscriber that is compatible
     * with the QOS profile of the publisher.
     * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
     * and durability set as "VOLATILE".
     * To be able to receive the subscribed topic the subscriber must use compatible
     * parameters.
     */

    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    video_qos.durability_volatile();

    // Create right image subscriber
    auto right_sub = g_node->create_subscription<sensor_msgs::msg::Image>(
                "right_image", video_qos, imageRightRectifiedCallback );

    // Create left image subscriber
    auto left_sub = g_node->create_subscription<sensor_msgs::msg::Image>(
                "left_image", video_qos, imageLeftRectifiedCallback );

    // Let the node run
    rclcpp::spin(g_node);

    // Shutdown when the node is stopped using Ctrl+C
    rclcpp::shutdown();

    return 0;
}
