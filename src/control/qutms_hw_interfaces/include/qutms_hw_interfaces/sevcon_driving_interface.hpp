#ifndef QUTMS_HW_INTERFACES__SEVCON_DRIVING_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__SEVCON_DRIVING_INTERFACE_HPP_

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

class SevconDrivingInterface : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SevconDrivingInterface)

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
    uint8_t left_motor_id_;
    uint8_t right_motor_id_;
    uint8_t vcu_sa_;

    // States: positions and velocities for left and right wheels
    double left_wheel_pos_state_;
    double left_wheel_vel_state_;
    double right_wheel_pos_state_;
    double right_wheel_vel_state_;

    // Commands: velocities for left and right wheels
    double left_wheel_vel_cmd_;
    double right_wheel_vel_cmd_;

    // Custom state interfaces exported for diagnostics (averaged for the two motors)
    double motor_temp_;
    double inverter_temp_;
    double fault_code_;
    double dc_voltage_;

    // Individual motor states
    double left_motor_temp_;
    double right_motor_temp_;
    double left_inverter_temp_;
    double right_inverter_temp_;
    uint16_t left_fault_code_;
    uint16_t right_fault_code_;
    double left_dc_voltage_;
    double right_dc_voltage_;

    // Configuration parameters
    double gear_ratio_;
    double wheel_radius_;
    std::string control_mode_;
    double torque_limit_nm_;
    double regen_limit_nm_;

    // PID gains for internal_pid mode
    double kp_;
    double ki_;
    double left_integral_error_;
    double right_integral_error_;

    // State machine management
    uint16_t left_status_word_;
    uint16_t right_status_word_;
    uint16_t desired_control_word_;

    // Transmit sequence counters for J1939 messages
    uint8_t left_hc1_seq_;
    uint8_t left_hc2_seq_;
    uint8_t left_hc3_seq_;
    uint8_t right_hc1_seq_;
    uint8_t right_hc2_seq_;
    uint8_t right_hc3_seq_;

    // ROS 2 node and publisher for diagnostics
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_pub_;

    uint32_t get_j1939_id(uint8_t pf, uint8_t ps, uint8_t sa);
    void send_hc1(uint8_t motor_id, uint8_t& seq, double torque_demand_nm, uint16_t control_word);
    void send_hc2(uint8_t motor_id, uint8_t& seq, double forward_speed_limit_rpm, double reverse_speed_limit_rpm);
    void send_hc3(uint8_t motor_id, uint8_t& seq);
    void publish_diagnostics();
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__SEVCON_DRIVING_INTERFACE_HPP_
