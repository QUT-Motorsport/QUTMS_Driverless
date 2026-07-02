#include "qutms_hw_interfaces/encos_steering_interface.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace qutms_hw_interfaces {

hardware_interface::CallbackReturn EncosSteeringInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read parameters from URDF xacro
    can_interface_name_ =
        info_.hardware_parameters.count("can_interface") ? info_.hardware_parameters.at("can_interface") : "can0";
    motor_id_ = info_.hardware_parameters.count("motor_id")
                    ? static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("motor_id")))
                    : 0x01;
    target_speed_rpm_ = info_.hardware_parameters.count("target_speed_rpm")
                            ? std::stod(info_.hardware_parameters.at("target_speed_rpm"))
                            : 50.0;
    current_limit_a_ = info_.hardware_parameters.count("current_limit_a")
                           ? std::stod(info_.hardware_parameters.at("current_limit_a"))
                           : 10.0;

    joint_position_state_ = std::numeric_limits<double>::quiet_NaN();
    joint_position_command_ = std::numeric_limits<double>::quiet_NaN();
    motor_temp_ = 0.0;
    mos_temp_ = 0.0;
    error_code_ = 0;
    current_ = 0.0;
    fault_code_ = 0.0;
    dc_voltage_ = 0.0;

    // Initialize ROS 2 Node for diagnostics
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=encos_steering_interface_node"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    auto pub = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
    diagnostics_pub_ = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(pub);

    if (!socket_can_) {
        socket_can_ = std::make_unique<SocketCAN>();
    }

    RCLCPP_INFO(rclcpp::get_logger("EncosSteeringInterface"),
                "Encos Steering Interface initialized. CAN: %s, Motor ID: 0x%X", can_interface_name_.c_str(),
                motor_id_);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncosSteeringInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!socket_can_->setup(can_interface_name_, rclcpp::get_logger("EncosSteeringInterface"))) {
        RCLCPP_ERROR(rclcpp::get_logger("EncosSteeringInterface"), "Failed to setup SocketCAN on %s",
                     can_interface_name_.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EncosSteeringInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_state_));
    // Export extra state interfaces for diagnostics
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "motor_temp", &motor_temp_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[0].name, "inverter_temp", &mos_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "fault_code", &fault_code_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "dc_voltage", &dc_voltage_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EncosSteeringInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_command_));
    return command_interfaces;
}

hardware_interface::CallbackReturn EncosSteeringInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("EncosSteeringInterface"), "Encos Steering Interface activated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EncosSteeringInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("EncosSteeringInterface"), "Encos Steering Interface deactivated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EncosSteeringInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/) {
    auto frames = socket_can_->rx(rclcpp::get_logger("EncosSteeringInterface"), node_->get_clock());

    for (const auto& msg : *frames) {
        // Encos feedback uses its Motor ID as CAN ID
        if (msg.id == motor_id_) {
            uint8_t type = (msg.data[0] >> 5) & 0x7;
            if (type == 0x01) {  // Type 1 feedback
                error_code_ = msg.data[0] & 0x1F;
                fault_code_ = static_cast<double>(error_code_);

                uint16_t pos_raw = static_cast<uint16_t>((msg.data[1] << 8) | msg.data[2]);
                uint16_t speed_raw = static_cast<uint16_t>((msg.data[3] << 4) | ((msg.data[4] >> 4) & 0x0F));
                uint16_t current_raw = static_cast<uint16_t>(((msg.data[4] & 0x0F) << 8) | msg.data[5]);
                uint8_t motor_temp_raw = msg.data[6];
                uint8_t mos_temp_raw = msg.data[7];

                // Decode position: 0 ~ 65535 corresponds to -12.5f ~ 12.5f rad
                joint_position_state_ = (static_cast<double>(pos_raw) / 65535.0) * 25.0 - 12.5;

                // Decode temperatures: actual temp * 2 + 50
                motor_temp_ = (static_cast<double>(motor_temp_raw) - 50.0) / 2.0;
                mos_temp_ = (static_cast<double>(mos_temp_raw) - 50.0) / 2.0;

                // Decode current: ratio 10 (needs table 9-1, placeholder mapping)
                current_ = static_cast<double>(current_raw) / 10.0;

                (void)speed_raw;  // unused speed for now
            }
        }
    }

    publish_diagnostics();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type EncosSteeringInterface::write(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
    if (std::isnan(joint_position_command_)) {
        return hardware_interface::return_type::OK;
    }

    // Convert joint position (rad) to output degrees
    float target_pos_deg = static_cast<float>(joint_position_command_ * 180.0 / M_PI);

    // Pack ENCOS Position command
    driverless_msgs::msg::Can msg;
    msg.id = motor_id_;
    msg.id_type = false;
    msg.dlc = 8;
    msg.data.resize(8, 0x00);

    uint64_t packet = 0;
    // Mode: 3 bits = 0x01
    packet |= (static_cast<uint64_t>(0x01) & 0x7) << 61;
    // Expected position: 32-bit float
    uint32_t pos_bits;
    std::memcpy(&pos_bits, &target_pos_deg, 4);
    packet |= (static_cast<uint64_t>(pos_bits) & 0xFFFFFFFF) << 29;
    // Expected speed: 15-bit uint, scale 10
    uint16_t speed_val = static_cast<uint16_t>(target_speed_rpm_ * 10.0);
    packet |= (static_cast<uint64_t>(speed_val) & 0x7FFF) << 14;
    // Current limit: 12-bit uint, scale 10
    uint16_t cur_limit_val = static_cast<uint16_t>(current_limit_a_ * 10.0);
    packet |= (static_cast<uint64_t>(cur_limit_val) & 0xFFF) << 2;
    // Msg return status: 2 bits = 0x01 (Type 1 feedback)
    packet |= (static_cast<uint64_t>(0x01) & 0x03);

    // Populate byte array (Big Endian)
    for (int i = 0; i < 8; i++) {
        msg.data[static_cast<size_t>(i)] = (packet >> (8 * (7 - i))) & 0xFF;
    }

    socket_can_->tx(&msg, rclcpp::get_logger("EncosSteeringInterface"));
    return hardware_interface::return_type::OK;
}

void EncosSteeringInterface::publish_diagnostics() {
    if (diagnostics_pub_ && diagnostics_pub_->trylock()) {
        auto& diag_msg = diagnostics_pub_->msg_;
        diag_msg.header.stamp = node_->now();
        diag_msg.status.clear();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Steering: ENCOS Steering Motor";
        status.hardware_id = std::to_string(motor_id_);

        if (error_code_ != 0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Fault Active (Code: " + std::to_string(error_code_) + ")";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Operational";
        }

        status.values.clear();
        diagnostic_msgs::msg::KeyValue motor_temp_val;
        motor_temp_val.key = "Motor Temp (°C)";
        motor_temp_val.value = std::to_string(motor_temp_);
        status.values.push_back(motor_temp_val);

        diagnostic_msgs::msg::KeyValue mos_temp_val;
        mos_temp_val.key = "MOS Temp (°C)";
        mos_temp_val.value = std::to_string(mos_temp_);
        status.values.push_back(mos_temp_val);

        diagnostic_msgs::msg::KeyValue current_val;
        current_val.key = "Current (A)";
        current_val.value = std::to_string(current_);
        status.values.push_back(current_val);

        diag_msg.status.push_back(status);
        diagnostics_pub_->unlockAndPublish();
    }
}

}  // namespace qutms_hw_interfaces

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::EncosSteeringInterface, hardware_interface::SystemInterface)
