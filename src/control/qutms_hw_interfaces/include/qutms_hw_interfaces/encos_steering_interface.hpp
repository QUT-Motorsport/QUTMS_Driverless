#ifndef QUTMS_HW_INTERFACES__ENCOS_STEERING_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__ENCOS_STEERING_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

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
#include "realtime_tools/realtime_publisher.hpp"

namespace qutms_hw_interfaces {

class EncosSteeringInterface : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(EncosSteeringInterface)

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
    uint32_t motor_id_;
    double joint_position_state_;
    double joint_position_command_;

    // Diagnostic/State variables
    double motor_temp_;
    double mos_temp_;
    uint8_t error_code_;
    double current_;
    double fault_code_;
    double dc_voltage_;

    // Target command configuration params
    double target_speed_rpm_;
    double current_limit_a_;

    // ROS 2 diagnostics node/publisher
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_pub_;

    void publish_diagnostics();
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__ENCOS_STEERING_INTERFACE_HPP_
