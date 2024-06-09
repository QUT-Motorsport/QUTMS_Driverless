#pragma once

// Much of this odometry conversion is adapted from:
// https://github.com/SBG-Systems/sbg_ros2_driver/blob/master/src/message_wrapper.cpp

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "driverless_common/common.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

typedef struct _UTM0 {
    double easting;
    double northing;
    double altitude;
    int zone;
} UTM0;

class SBGTranslate : public rclcpp::Node {
   private:
    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // initial values
    double last_yaw_rate_ = 0.0;
    double last_yaw_change_ = 0.0;
    double last_yaw_vec_angle_ = 0.0;

    // updated values
    std::vector<float> state_;
    double yaw_cov_ = 0.5;

    std::vector<float> x_deltas_;
    std::vector<float> y_deltas_;

    // store path
    nav_msgs::msg::Path path_msg_;
    rclcpp::Time last_pub_time_;
    rclcpp::Time last_time_;

    bool received_odom_ = false;

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void ekf_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    double filer_yaw(double x, double y);

    void update_pos_buffer(float x, float y) {
        x_deltas_.push_back(x);
        y_deltas_.push_back(y);
        // max at 10
        if (x_deltas_.size() > 10) {
            x_deltas_.erase(x_deltas_.begin());
            y_deltas_.erase(y_deltas_.begin());
        }
    }

    double wrap_to_pi(double angle) {
        return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    sensor_msgs::msg::Imu make_imu_msg(nav_msgs::msg::Odometry odom_msg) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = odom_msg.header.stamp;
        imu_msg.header.frame_id = "chassis";

        imu_msg.orientation = odom_msg.pose.pose.orientation;

        return imu_msg;
    }

    geometry_msgs::msg::PoseStamped make_pose_msg(nav_msgs::msg::Odometry odom_msg) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = odom_msg.header.stamp;
        pose_msg.header.frame_id = "track";

        pose_msg.pose = odom_msg.pose.pose;

        return pose_msg;
    }

    static inline geometry_msgs::msg::Quaternion euler_to_quat(double x, double y, double z) {
        tf2::Quaternion q;
        q.setRPY(x, y, z);
        q.normalize();
        return tf2::toMsg(q);
    }

    static inline double quat_to_euler(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

   public:
    SBGTranslate();
};
