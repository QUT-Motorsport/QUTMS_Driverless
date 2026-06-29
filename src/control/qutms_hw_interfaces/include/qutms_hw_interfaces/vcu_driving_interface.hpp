#ifndef QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_

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
#include "realtime_tools/realtime_publisher.hpp"
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

    // Commands: efforts for left and right wheels
    double left_wheel_eff_cmd_;
    double right_wheel_eff_cmd_;

    // Configuration parameters
    double wheel_radius_;

    // ROS 2 node and subscriber for Ackermann drive commands
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_pub_;

    double prev_accel_;

    void publish_diagnostics();
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__VCU_DRIVING_INTERFACE_HPP_
