#pragma once

#include <cmath>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

// See driverless_common/common.py for explanation of how to use these for your scenario.
const rclcpp::QoS QOS_LATEST(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
const rclcpp::QoS QOS_ALL(rclcpp::KeepLast(100), rmw_qos_profile_parameters);

inline double dist(double x1, double y1, double x2, double y2) {
    double diff_x = x1 - x2;
    double diff_y = y1 - y2;
    return sqrt(diff_x * diff_x + diff_y * diff_y);
}

inline double fast_dist(double x1, double y1, double x2, double y2) {
    double diff_x = x1 - x2;
    double diff_y = y1 - y2;
    return diff_x * diff_x + diff_y * diff_y;
}

inline double angle(double x1, double y1, double x2, double y2) {
    double x_disp = x2 - x1;
    double y_disp = y2 - y1;
    return atan2(y_disp, x_disp);
}

// https://stackoverflow.com/a/29871193
inline double wrap_to_pi(double x) {
    double min = x - (-M_PI);
    double max = M_PI - (-M_PI);
    return -M_PI + fmod(max + fmod(min, max), max);
}
