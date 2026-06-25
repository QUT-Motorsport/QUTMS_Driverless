#ifndef QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "qutms_hw_interfaces/SocketCAN.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace qutms_hw_interfaces {

class VcuDrivingInterface : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(VcuDrivingInterface)

    void set_socket_can(std::unique_ptr<SocketCAN> socket_can) { socket_can_ = std::move(socket_can); }

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::unique_ptr<SocketCAN> socket_can_;
    std::string can_interface_name_;

    // States: positions and velocities for left and right wheels
    double left_wheel_pos_state_;
    double left_wheel_vel_state_;
    double right_wheel_pos_state_;
    double right_wheel_vel_state_;

    // Commands: velocities for left and right wheels
    double left_wheel_vel_cmd_;
    double right_wheel_vel_cmd_;

    // Configuration parameters
    double wheel_radius_;
    double kp_;
    double ki_;
    double max_integral_torque_;

    // Controller internal states
    double integral_error_;
    double prev_accel_;

    // Track steering angle from Ackermann command
    float target_steering_angle_;

    // ROS 2 node and subscriber for Ackermann drive commands
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void publish_diagnostics();
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_
