#include "builtin_interfaces/msg/time.hpp"

double wrap_pi(double x) {
    // [-pi, pi)
    double m = fmod(x + M_PI, M_PI * 2);
    if (m < 0) {
        m += M_PI * 2;
    }
    return m - M_PI;
}

double compute_dt(rclcpp::Time start_, rclcpp::Time end_) { return (end_ - start_).nanoseconds() * 1e-9; }
