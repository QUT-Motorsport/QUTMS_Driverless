#ifndef QUTMS_HW_INTERFACES__QEV_STEPPER_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__QEV_STEPPER_INTERFACE_HPP_

#include <map>
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

struct c5e_state {
    std::string name;
    uint16_t mask;
    uint16_t state_id;
    uint16_t control_word;
};

class QevStepperInterface : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(QevStepperInterface)

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
    uint8_t node_id_;
    int32_t max_position_;
    bool steering_ang_received_;
    int32_t current_position_;
    double joint_position_state_;
    double joint_position_command_;

    // Diagnostic/State variables
    double motor_temp_;
    double inverter_temp_;
    double fault_code_;
    double dc_voltage_;

    uint16_t current_status_word_;
    c5e_state current_state_;
    c5e_state desired_state_;

    // Config parameters for velocity/acceleration
    uint32_t velocity_;
    uint32_t acceleration_;

    // ROS 2 node for publishing diagnostics
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_pub_;

    void configure_c5e();
    void sdo_write(uint16_t index, uint8_t sub_index, uint8_t* data, size_t data_size);
    void sdo_read(uint16_t index, uint8_t sub_index);
    void target_position(int32_t target);
    void publish_diagnostics(bool has_fault, const std::string& reason);
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__QEV_STEPPER_INTERFACE_HPP_
