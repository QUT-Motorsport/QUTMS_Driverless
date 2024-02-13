#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class YawController : public rclcpp::Node {
   private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr shutdown_sub;

    rclcpp::TimerBase::SharedPtr yaw_timer;
    rclcpp::TimerBase::SharedPtr yaw_rate_timer;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driving_command_pub;

    float target_yaw = 0;
    float current_yaw = 0;
    float integral_yaw = 0;

    float target_yaw_rate = 0;
    float current_yaw_rate = 0;
    float integral_yaw_rate = 0;

    float Kp_yaw = 0;
    float Ki_yaw = 0;
    float Kp_yaw_rate = 0;
    float Ki_yaw_rate = 0;
    float max_integral_steering = 0;
    float target_velocity = 0;
    float integral_kickin = 0;

    void shutdown_callback(const driverless_msgs::msg::State msg);

    inline void velocity_callback(const geometry_msgs::msg::TwistStamped msg) {
        current_yaw_rate = msg.twist.angular.z;
    }

    void imu_callback(const sensor_msgs::msg::Imu msg);

    inline void yaw_callback() {
        // pass
    }

    void yaw_rate_callback();

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event);

   public:
    YawController();
};