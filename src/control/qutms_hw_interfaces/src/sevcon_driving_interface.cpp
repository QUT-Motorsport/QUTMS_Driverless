#include "qutms_hw_interfaces/sevcon_driving_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace qutms_hw_interfaces {

// J1939 PFN Constants
static constexpr uint8_t PF_HC1 = 0x10;
static constexpr uint8_t PF_HC2 = 0x11;
static constexpr uint8_t PF_HC3 = 0x12;
static constexpr uint8_t PF_HS1 = 0x18;
static constexpr uint8_t PF_HS2 = 0x19;
static constexpr uint8_t PF_HS3 = 0x1A;
static constexpr uint8_t PF_HS4 = 0x1B;

hardware_interface::CallbackReturn SevconDrivingInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read parameters from URDF xacro
    can_interface_name_ =
        info_.hardware_parameters.count("can_interface") ? info_.hardware_parameters.at("can_interface") : "can0";
    left_motor_id_ = info_.hardware_parameters.count("left_motor_id")
                         ? static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("left_motor_id")))
                         : 0x22;
    right_motor_id_ = info_.hardware_parameters.count("right_motor_id")
                          ? static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("right_motor_id")))
                          : 0x23;
    vcu_sa_ = info_.hardware_parameters.count("vcu_sa")
                  ? static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("vcu_sa")))
                  : 0x00;

    gear_ratio_ =
        info_.hardware_parameters.count("gear_ratio") ? std::stod(info_.hardware_parameters.at("gear_ratio")) : 4.50;
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius")
                        ? std::stod(info_.hardware_parameters.at("wheel_radius"))
                        : 0.2032;
    control_mode_ =
        info_.hardware_parameters.count("control_mode") ? info_.hardware_parameters.at("control_mode") : "internal_pid";

    torque_limit_nm_ = info_.hardware_parameters.count("torque_limit_nm")
                           ? std::stod(info_.hardware_parameters.at("torque_limit_nm"))
                           : 100.0;
    regen_limit_nm_ = info_.hardware_parameters.count("regen_limit_nm")
                          ? std::stod(info_.hardware_parameters.at("regen_limit_nm"))
                          : -100.0;

    left_wheel_pos_state_ = 0.0;
    left_wheel_vel_state_ = 0.0;
    right_wheel_pos_state_ = 0.0;
    right_wheel_vel_state_ = 0.0;

    left_wheel_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();
    right_wheel_eff_cmd_ = std::numeric_limits<double>::quiet_NaN();

    motor_temp_ = 0.0;
    inverter_temp_ = 0.0;
    fault_code_ = 0.0;
    dc_voltage_ = 0.0;

    left_motor_temp_ = 0.0;
    right_motor_temp_ = 0.0;
    left_inverter_temp_ = 0.0;
    right_inverter_temp_ = 0.0;
    left_fault_code_ = 0;
    right_fault_code_ = 0;
    left_dc_voltage_ = 0.0;
    right_dc_voltage_ = 0.0;

    left_status_word_ = 0;
    right_status_word_ = 0;
    desired_control_word_ = 0x0006;  // SHUTDOWN default

    left_hc1_seq_ = 0;
    left_hc2_seq_ = 0;
    left_hc3_seq_ = 0;
    right_hc1_seq_ = 0;
    right_hc2_seq_ = 0;
    right_hc3_seq_ = 0;

    // Initialize ROS 2 Node for diagnostics
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=sevcon_driving_interface_node"});
    node_ = rclcpp::Node::make_shared("_", options);

    auto pub = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
    diagnostics_pub_ = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(pub);

    if (!socket_can_) {
        socket_can_ = std::make_unique<SocketCAN>();
    }

    RCLCPP_INFO(rclcpp::get_logger("SevconDrivingInterface"),
                "Sevcon Driving Interface initialized. Left Motor ID: 0x%X, Right Motor ID: 0x%X, Mode: %s",
                left_motor_id_, right_motor_id_, control_mode_.c_str());
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SevconDrivingInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!socket_can_->setup(can_interface_name_, rclcpp::get_logger("SevconDrivingInterface"))) {
        RCLCPP_ERROR(rclcpp::get_logger("SevconDrivingInterface"), "Failed to setup SocketCAN on %s",
                     can_interface_name_.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SevconDrivingInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Left wheel states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_state_));

    // Right wheel states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_state_));

    // Export averaged diagnostics as state interfaces on the first joint
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "motor_temp", &motor_temp_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[0].name, "inverter_temp", &inverter_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "fault_code", &fault_code_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "dc_voltage", &dc_voltage_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SevconDrivingInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // Export effort commands
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &left_wheel_eff_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_EFFORT, &right_wheel_eff_cmd_));
    return command_interfaces;
}

hardware_interface::CallbackReturn SevconDrivingInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    desired_control_word_ = 0x0005;  // ENABLE OPERATION
    RCLCPP_INFO(rclcpp::get_logger("SevconDrivingInterface"), "Sevcon Driving Interface activated. Enabling bridge...");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SevconDrivingInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    desired_control_word_ = 0x0006;  // SHUTDOWN
    RCLCPP_INFO(rclcpp::get_logger("SevconDrivingInterface"),
                "Sevcon Driving Interface deactivated. Shutting down bridge...");
    return CallbackReturn::SUCCESS;
}

uint32_t SevconDrivingInterface::get_j1939_id(uint8_t pf, uint8_t ps, uint8_t sa) {
    return (6 << 26) | (pf << 16) | (ps << 8) | sa;
}

hardware_interface::return_type SevconDrivingInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period) {
    auto frames = socket_can_->rx(rclcpp::get_logger("SevconDrivingInterface"), node_->get_clock());

    for (const auto& msg : *frames) {
        // Extract J1939 SA (bits 0-7) and PF (bits 16-23)
        uint8_t sa = msg.id & 0xFF;
        uint8_t pf = (msg.id >> 16) & 0xFF;

        if (sa == left_motor_id_ || sa == right_motor_id_) {
            bool is_left = (sa == left_motor_id_);

            if (pf == PF_HS1) {
                // Byte 0-1: Measured output torque (Signed 16-bit, 0.0625 Nm/bit)
                int16_t torque_raw = static_cast<int16_t>((msg.data[1] << 8) | msg.data[0]);
                // Byte 2-3: Measured motor speed (Signed 16-bit, 1 RPM/bit)
                int16_t speed_raw = static_cast<int16_t>((msg.data[3] << 8) | msg.data[2]);

                // Convert motor RPM to wheel linear velocity (rad/s)
                double motor_rads = static_cast<double>(speed_raw) * 2.0 * M_PI / 60.0;
                double wheel_vel = motor_rads / gear_ratio_;

                if (is_left) {
                    left_wheel_vel_state_ = wheel_vel;
                    left_wheel_pos_state_ += wheel_vel * period.seconds();
                } else {
                    right_wheel_vel_state_ = wheel_vel;
                    right_wheel_pos_state_ += wheel_vel * period.seconds();
                }
                (void)torque_raw;

            } else if (pf == PF_HS2) {
                // Byte 4-5: Status word (Byte 4 lower 4 bits is state)
                uint16_t status_word = static_cast<uint16_t>((msg.data[5] << 8) | msg.data[4]);
                if (is_left) {
                    left_status_word_ = status_word;
                } else {
                    right_status_word_ = status_word;
                }

            } else if (pf == PF_HS3) {
                // Byte 0-1: Measured heat sink temperature (Signed 16-bit, 1 degC/bit)
                int16_t hs_temp = static_cast<int16_t>((msg.data[1] << 8) | msg.data[0]);
                // Byte 4-5: Measured capacitor voltage (Signed 16-bit, 0.0625 V/bit)
                int16_t cap_volt = static_cast<int16_t>((msg.data[5] << 8) | msg.data[4]);

                if (is_left) {
                    left_inverter_temp_ = static_cast<double>(hs_temp);
                    left_dc_voltage_ = static_cast<double>(cap_volt) * 0.0625;
                } else {
                    right_inverter_temp_ = static_cast<double>(hs_temp);
                    right_dc_voltage_ = static_cast<double>(cap_volt) * 0.0625;
                }

            } else if (pf == PF_HS4) {
                // Byte 0-1: Fault code
                uint16_t fault = static_cast<uint16_t>((msg.data[1] << 8) | msg.data[0]);
                if (is_left) {
                    left_fault_code_ = fault;
                } else {
                    right_fault_code_ = fault;
                }
            }
        }
    }

    // Average values for state interfaces
    motor_temp_ = (left_motor_temp_ + right_motor_temp_) / 2.0;
    inverter_temp_ = (left_inverter_temp_ + right_inverter_temp_) / 2.0;
    dc_voltage_ = (left_dc_voltage_ + right_dc_voltage_) / 2.0;
    fault_code_ = static_cast<double>(left_fault_code_ > 0 ? left_fault_code_ : right_fault_code_);

    publish_diagnostics();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SevconDrivingInterface::write(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
    double left_torque = 0.0;
    double left_fwd_limit = 0.0;
    double left_rev_limit = 0.0;

    double right_torque = 0.0;
    double right_fwd_limit = 0.0;
    double right_rev_limit = 0.0;

    // Check fault reset requirements
    uint8_t left_state = left_status_word_ & 0x0F;
    uint8_t right_state = right_status_word_ & 0x0F;

    uint16_t cw = desired_control_word_;
    if (left_state == 0x0D || right_state == 0x0D) {
        cw = 0x0009;  // FAULT RESET
    } else if (left_state == 0x02 && right_state == 0x02 && desired_control_word_ == 0x0005) {
        cw = 0x0003;  // ENERGISE
    } else if (left_state == 0x07 && right_state == 0x07 && desired_control_word_ == 0x0005) {
        cw = 0x0005;  // ENABLE OPERATION
    }

    // Direct effort/torque command mode (from chained PID controller)
    double left_eff = std::isnan(left_wheel_eff_cmd_) ? 0.0 : left_wheel_eff_cmd_;
    double right_eff = std::isnan(right_wheel_eff_cmd_) ? 0.0 : right_wheel_eff_cmd_;

    // Scaled to Nm using limits
    if (left_eff >= 0.0) {
        left_torque = left_eff * torque_limit_nm_;
    } else {
        left_torque = left_eff * (-regen_limit_nm_);
    }
    left_fwd_limit = 5000.0;
    left_rev_limit = -5000.0;

    if (right_eff >= 0.0) {
        right_torque = right_eff * torque_limit_nm_;
    } else {
        right_torque = right_eff * (-regen_limit_nm_);
    }
    right_fwd_limit = 5000.0;
    right_rev_limit = -5000.0;

    send_hc1(left_motor_id_, left_hc1_seq_, left_torque, cw);
    send_hc2(left_motor_id_, left_hc2_seq_, left_fwd_limit, left_rev_limit);
    send_hc3(left_motor_id_, left_hc3_seq_);

    send_hc1(right_motor_id_, right_hc1_seq_, right_torque, cw);
    send_hc2(right_motor_id_, right_hc2_seq_, right_fwd_limit, right_rev_limit);
    send_hc3(right_motor_id_, right_hc3_seq_);

    return hardware_interface::return_type::OK;
}

void SevconDrivingInterface::send_hc1(uint8_t motor_id, uint8_t& seq, double torque_demand_nm, uint16_t control_word) {
    driverless_msgs::msg::Can msg;
    msg.id = get_j1939_id(PF_HC1, motor_id, vcu_sa_);
    msg.id_type = true;
    msg.dlc = 8;
    msg.data.resize(8, 0xFF);

    // Scale torque demand: 0.0625 Nm/bit (1 / 0.0625 = 16)
    int16_t torque_raw = static_cast<int16_t>(torque_demand_nm * 16.0);
    msg.data[0] = torque_raw & 0xFF;
    msg.data[1] = (torque_raw >> 8) & 0xFF;

    // Control word
    msg.data[2] = control_word & 0xFF;
    msg.data[3] = (control_word >> 8) & 0xFF;

    // Drive torque limit
    int16_t limit_raw = static_cast<int16_t>(torque_limit_nm_ * 16.0);
    msg.data[4] = limit_raw & 0xFF;
    msg.data[5] = (limit_raw >> 8) & 0xFF;

    // Sequence and Checksum
    msg.data[6] = seq++;

    uint8_t cs = 0;
    for (int i = 0; i < 7; i++) {
        cs += msg.data[i];
    }
    msg.data[7] = cs;

    socket_can_->tx(&msg, rclcpp::get_logger("SevconDrivingInterface"));
}

void SevconDrivingInterface::send_hc2(uint8_t motor_id, uint8_t& seq, double forward_speed_limit_rpm,
                                      double reverse_speed_limit_rpm) {
    driverless_msgs::msg::Can msg;
    msg.id = get_j1939_id(PF_HC2, motor_id, vcu_sa_);
    msg.id_type = true;
    msg.dlc = 8;
    msg.data.resize(8, 0xFF);

    // Regen torque limit
    int16_t regen_raw = static_cast<int16_t>(regen_limit_nm_ * 16.0);
    msg.data[0] = regen_raw & 0xFF;
    msg.data[1] = (regen_raw >> 8) & 0xFF;

    // Forward speed limit (1 RPM/bit)
    int16_t fwd_raw = static_cast<int16_t>(std::abs(forward_speed_limit_rpm));
    msg.data[2] = fwd_raw & 0xFF;
    msg.data[3] = (fwd_raw >> 8) & 0xFF;

    // Reverse speed limit (1 RPM/bit)
    int16_t rev_raw = static_cast<int16_t>(-std::abs(reverse_speed_limit_rpm));
    msg.data[4] = rev_raw & 0xFF;
    msg.data[5] = (rev_raw >> 8) & 0xFF;

    msg.data[6] = seq++;

    uint8_t cs = 0;
    for (int i = 0; i < 7; i++) {
        cs += msg.data[i];
    }
    msg.data[7] = cs;

    socket_can_->tx(&msg, rclcpp::get_logger("SevconDrivingInterface"));
}

void SevconDrivingInterface::send_hc3(uint8_t motor_id, uint8_t& seq) {
    driverless_msgs::msg::Can msg;
    msg.id = get_j1939_id(PF_HC3, motor_id, vcu_sa_);
    msg.id_type = true;
    msg.dlc = 8;
    msg.data.resize(8, 0xFF);

    // Discharge current limit: 1 A/bit (e.g. 200 A)
    int16_t discharge_raw = 200;
    msg.data[0] = discharge_raw & 0xFF;
    msg.data[1] = (discharge_raw >> 8) & 0xFF;

    // Charge current limit: 1 A/bit (e.g. -100 A)
    int16_t charge_raw = -100;
    msg.data[2] = charge_raw & 0xFF;
    msg.data[3] = (charge_raw >> 8) & 0xFF;

    // Target voltage: 0.0625 V/bit (e.g. 400 V = 6400 bits)
    int16_t target_volts_raw = static_cast<int16_t>(400.0 * 16.0);
    msg.data[4] = target_volts_raw & 0xFF;
    msg.data[5] = (target_volts_raw >> 8) & 0xFF;

    msg.data[6] = seq++;

    uint8_t cs = 0;
    for (int i = 0; i < 7; i++) {
        cs += msg.data[i];
    }
    msg.data[7] = cs;

    socket_can_->tx(&msg, rclcpp::get_logger("SevconDrivingInterface"));
}

void SevconDrivingInterface::publish_diagnostics() {
    if (diagnostics_pub_ && diagnostics_pub_->trylock()) {
        auto& diag_msg = diagnostics_pub_->msg_;
        diag_msg.header.stamp = node_->now();
        diag_msg.status.clear();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Driving: Sevcon Inverters";
        status.hardware_id = "Sevcon_Dual_Setup";

        if (left_fault_code_ != 0 || right_fault_code_ != 0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Fault Active (L: " + std::to_string(left_fault_code_) +
                             ", R: " + std::to_string(right_fault_code_) + ")";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Operational";
        }

        status.values.clear();
        diagnostic_msgs::msg::KeyValue dc_v_val;
        dc_v_val.key = "DC Link Voltage (V)";
        dc_v_val.value = std::to_string(dc_voltage_);
        status.values.push_back(dc_v_val);

        diagnostic_msgs::msg::KeyValue temp_val;
        temp_val.key = "Average Inverter Temp (°C)";
        temp_val.value = std::to_string(inverter_temp_);
        status.values.push_back(temp_val);

        diagnostic_msgs::msg::KeyValue l_state_val;
        l_state_val.key = "Left State";
        l_state_val.value = std::to_string(left_status_word_ & 0x0F);
        status.values.push_back(l_state_val);

        diagnostic_msgs::msg::KeyValue r_state_val;
        r_state_val.key = "Right State";
        r_state_val.value = std::to_string(right_status_word_ & 0x0F);
        status.values.push_back(r_state_val);

        diag_msg.status.push_back(status);
        diagnostics_pub_->unlockAndPublish();
    }
}

}  // namespace qutms_hw_interfaces

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::SevconDrivingInterface, hardware_interface::SystemInterface)
