#pragma once

// Much of this odometry conversion is adapted from:
// https://github.com/SBG-Systems/sbg_ros2_driver/blob/master/src/message_wrapper.cpp

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "driverless_common/common.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"
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
    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr euler_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr nav_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // initial values
    float init_yaw_;
    float last_x_;
    float last_y_;
    UTM0 m_utm0_;

    // updated values
    sbg_driver::msg::SbgImuData last_imu_msg_;
    sbg_driver::msg::SbgEkfNav last_nav_msg_;
    sbg_driver::msg::SbgEkfEuler last_euler_msg_;
    std::vector<float> state_;

    bool received_imu_ = false;
    bool received_nav_ = false;
    bool received_euler_ = false;
    bool received_odom_ = false;

    void update_odom();

    sensor_msgs::msg::Imu make_imu_msg(nav_msgs::msg::Odometry odom_msg) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = odom_msg.header.stamp;
        imu_msg.header.frame_id = "base_footprint";

        imu_msg.orientation = odom_msg.pose.pose.orientation;

        return imu_msg;
    }

    void initUTM(double Lat, double Long, double altitude);
    void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting);

    inline static double computeMeridian(int zone_number) {
        return (zone_number == 0) ? 0.0 : (zone_number - 1) * 6.0 - 177.0;
    }
    inline static double sbgDegToRadD(double angle) { return angle * M_PI / 180.0; }

    void euler_callback(sbg_driver::msg::SbgEkfEuler::SharedPtr msg);
    void nav_callback(sbg_driver::msg::SbgEkfNav::SharedPtr msg);
    void imu_callback(sbg_driver::msg::SbgImuData::SharedPtr msg);
    void ekf_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);

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
