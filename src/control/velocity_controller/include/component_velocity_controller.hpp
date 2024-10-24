#ifndef VELOCITY_CONTROLLER__COMPONENT_VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__COMPONENT_VELOCITY_CONTROLLER_HPP_

#include <iostream>

#include "CAN_AV.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "can_interface.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/ros_state_stamped.hpp"
#include "driverless_msgs/msg/av_state_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

namespace velocity_controller {

class VelocityController : public rclcpp::Node, public CanInterface {
   private:
    float Kp_ = 0;
    float Ki_ = 0;
    float max_integral_torque_ = 0;
    float histerisis_kickin_ms_ = 0;
    float histerisis_reset_ms_ = 0;
    float min_time_to_max_accel_sec_ = 0;
    float max_accel_per_tick_ = 0;

    float integral_error_ = 0;

    rclcpp::TimerBase::SharedPtr controller_timer_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    rclcpp::Subscription<driverless_msgs::msg::AVStateStamped>::SharedPtr av_state_sub_;
    rclcpp::Subscription<driverless_msgs::msg::ROSStateStamped>::SharedPtr ros_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_cb_handle_;

    const int loop_ms_ = 10;

    // Enable motors logic
    driverless_msgs::msg::AVStateStamped::SharedPtr state_ = std::make_shared<driverless_msgs::msg::AVStateStamped>();
    bool g2g_ = false;
    bool motors_enabled_ = false;
    bool received_velocity_ = false;
    bool received_ackermann_ = false;

    float avg_velocity_;
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr target_ackermann_ =
        std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
    float prev_accel_ = 0;

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event);
    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void av_state_callback(const driverless_msgs::msg::AVStateStamped::SharedPtr msg);
    void ros_state_callback(const driverless_msgs::msg::ROSStateStamped::SharedPtr msg);

    void controller_callback();

   public:
    VelocityController(const rclcpp::NodeOptions& options);
};

}  // namespace velocity_controller

#endif  // VELOCITY_CONTROLLER__COMPONENT_VELOCITY_CONTROLLER_HPP_
