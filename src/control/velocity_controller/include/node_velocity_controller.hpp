#include <iostream>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class Velocity_Controller : public rclcpp::Node {
   private:
    float Kp = 0;
    float Ki = 0;
    float max_integral_torque = 0;
    float histerisis_kickin_ms = 0;
    float histerisis_reset_ms = 0;
    float min_time_to_max_accel_sec = 0;
    float max_accel_per_tick = 0;

    float integral_error = 0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr accel_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub;
    rclcpp::TimerBase::SharedPtr controller_timer;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_cb_handle;

    const int loop_ms = 10;

    // Enable motors logic
    driverless_msgs::msg::State state;
    bool motors_enabled = false;
    bool received_velocity = false;
    bool received_ackermann = false;

    float avg_velocity;
    ackermann_msgs::msg::AckermannDriveStamped target_ackermann;
    ackermann_msgs::msg::AckermannDriveStamped prev_accel_cmd;

   public:
    Velocity_Controller();

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event);

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped msg);

    void velocity_callback(const std_msgs::msg::Float32 msg);

    void state_callback(const driverless_msgs::msg::State msg);

    void controller_callback();
};
