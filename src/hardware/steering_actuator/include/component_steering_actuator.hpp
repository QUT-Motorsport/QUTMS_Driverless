#ifndef STEERING_ACTUATOR__COMPONENT_STEERING_ACTUATOR_HPP_
#define STEERING_ACTUATOR__COMPONENT_STEERING_ACTUATOR_HPP_

#include <stddef.h>
#include <stdint.h>

#include <chrono>
#include <mutex>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "can_interface.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/av_state_stamped.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/ros_state_stamped.hpp"
#include "encos_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

namespace steering_actuator {

class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    rclcpp::TimerBase::SharedPtr send_timer_;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub_;
    rclcpp::Subscription<driverless_msgs::msg::AVStateStamped>::SharedPtr as_state_sub_;
    rclcpp::Subscription<driverless_msgs::msg::ROSStateStamped>::SharedPtr ros_state_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr motor_sub_;

    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_cb_handle_;

    // params
    int motor_id_;
    double speed_rpm_;
    double current_limit_a_;
    double scale_;
    double steer_offset_deg_;
    bool invert_;
    double max_position_;  // deg at the motor output
    double command_timeout_s_;
    bool require_driving_;

    std::mutex mutex_;
    bool av_driving_ = false;
    bool g2g_ = false;
    bool have_target_ = false;
    float target_deg_ = 0;
    rclcpp::Time last_cmd_time_;
    bool sending_ = false;

    void update_parameters(const rcl_interfaces::msg::ParameterEvent &event);
    void as_state_callback(const driverless_msgs::msg::AVStateStamped::SharedPtr msg);
    void ros_state_callback(const driverless_msgs::msg::ROSStateStamped::SharedPtr msg);
    void driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void motor_callback(const driverless_msgs::msg::Can::SharedPtr msg);
    void send_timer_callback();
    void send_frame(const uint8_t *data, uint8_t dlc);

   public:
    SteeringActuator(const rclcpp::NodeOptions &options);
    ~SteeringActuator() override;
};

}  // namespace steering_actuator

#endif  // STEERING_ACTUATOR__COMPONENT_STEERING_ACTUATOR_HPP_
