#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using std::placeholders::_1;

struct Pose {
    double const x;
    double const y;
    double const yaw;

    Pose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
};

struct Point {
    double const x;
    double const y;

    Point() : x(0), y(0) {}

    Point(double x, double y) : x(x), y(y) {}
};

class PurePursuit {
public:
    PurePursuit(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer,std::shared_ptr<tf2_ros::TransformListener> listener);
    PurePursuit(rclcpp_lifecycle::LifecycleNode::SharedPtr, tf2_ros::Buffer::SharedPtr buffer, std::shared_ptr<tf2_ros::TransformListener> listener);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rvwp_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driving_command_pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector <Point> path{};
    tf2_ros::Buffer::SharedPtr buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;

    double cog2axle{0.5};

    double kp_ang;
    double lookahead;
    double squared_lookahead;  // allows us to skip performing sqrts in get_rvwp()
    double vel_max;
    double vel_min;
    double discovery_lookahead;
    double squared_discovery_lookahead;  // allows us to skip performing sqrts in get_rvwp()
    double discovery_vel_max;
    double discovery_vel_min;

    int fallback_path_points_offset;  // The number of path points to be skipped for an approximate rvwp

    // Internal states
    bool driving{false};
    bool following{false};

    inline double calc_steering(const Pose &pose, const Point &rvwp) const {
        // Calculates the steering angle based on the pose of the car and the RVWP.
        // Gets the angle between the car and the RVWP and calculates the error between the desired heading and the
        // current heading. The steering angle is then calculated using the error and the proportional gain.

        double des_heading_ang = atan2(rvwp.y - pose.y, rvwp.x - pose.x);
        double error = wrap_to_pi(pose.yaw - des_heading_ang);
        double deg_error = error * 180.0 / M_PI;
        return deg_error * kp_ang;
    }

    inline double calc_velocity(double desired_steering) const {
        // Calculates the velocity based on the steering angle.
        // Reduces the velocity as the steering angle increases

        double vel =
                vel_min + std::max((vel_max - vel_min) * (1 - std::pow(std::abs(desired_steering) / 90.0, 2.0)), 0.0);
        return vel;
    }

    inline Pose get_wheel_position(double x, double y, double th) const {
        // Gets the position of the steering axle from the car's
        // center of gravity and heading

        double x_axle = x + cos(th) * this->cog2axle;
        double y_axle = y + sin(th) * this->cog2axle;

        return {x_axle, y_axle, th};
    }

    inline bool can_drive() const {
        return this->driving && this->path.size() > 0;
    }

    Point get_rvwp(Pose const &car_axle_pos) const;
    void timer_callback();
    void path_callback(nav_msgs::msg::Path const &spline_path_msg);
private:
    void initialise_params(rclcpp::Node::SharedPtr);
    void initialise_params(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    rclcpp::Logger get_logger_() const {
        return node != nullptr ? node->get_logger() : lifecycle_node->get_logger();
    }

    rclcpp::Time get_now_() const {
        return node != nullptr ? node->now() : lifecycle_node->now();
    }

    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node;
};