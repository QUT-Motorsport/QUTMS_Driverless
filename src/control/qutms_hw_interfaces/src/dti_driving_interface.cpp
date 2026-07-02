#include "qutms_hw_interfaces/dti_driving_interface.hpp"

#include <cmath>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace qutms_hw_interfaces {

hardware_interface::CallbackReturn DtiDrivingInterface::on_init(
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
    use_extended_id_ = info_.hardware_parameters.count("use_extended_id")
                           ? (info_.hardware_parameters.at("use_extended_id") == "true")
                           : false;

    gear_ratio_ =
        info_.hardware_parameters.count("gear_ratio") ? std::stod(info_.hardware_parameters.at("gear_ratio")) : 4.50;
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius")
                        ? std::stod(info_.hardware_parameters.at("wheel_radius"))
                        : 0.2032;
    pole_pairs_ = info_.hardware_parameters.count("pole_pairs")
                      ? static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("pole_pairs")))
                      : 21;

    left_wheel_pos_state_ = 0.0;
    left_wheel_vel_state_ = 0.0;
    right_wheel_pos_state_ = 0.0;
    right_wheel_vel_state_ = 0.0;

    left_wheel_vel_cmd_ = 0.0;
    right_wheel_vel_cmd_ = 0.0;

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

    // Initialize ROS 2 Node for diagnostics
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=dti_driving_interface_node"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    auto pub = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
    diagnostics_pub_ = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(pub);

    if (!socket_can_) {
        socket_can_ = std::make_unique<SocketCAN>();
    }

    RCLCPP_INFO(rclcpp::get_logger("DtiDrivingInterface"),
                "DTI Driving Interface initialized. Left Motor ID: 0x%X, Right Motor ID: 0x%X, Extended ID: %s",
                left_motor_id_, right_motor_id_, use_extended_id_ ? "true" : "false");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DtiDrivingInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!socket_can_->setup(can_interface_name_, rclcpp::get_logger("DtiDrivingInterface"))) {
        RCLCPP_ERROR(rclcpp::get_logger("DtiDrivingInterface"), "Failed to setup SocketCAN on %s",
                     can_interface_name_.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DtiDrivingInterface::export_state_interfaces() {
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

std::vector<hardware_interface::CommandInterface> DtiDrivingInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_cmd_));
    return command_interfaces;
}

hardware_interface::CallbackReturn DtiDrivingInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    send_drive_enable(left_motor_id_, true);
    send_drive_enable(right_motor_id_, true);
    RCLCPP_INFO(rclcpp::get_logger("DtiDrivingInterface"), "DTI Driving Interface activated. Enabling drives...");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DtiDrivingInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    send_drive_enable(left_motor_id_, false);
    send_drive_enable(right_motor_id_, false);
    RCLCPP_INFO(rclcpp::get_logger("DtiDrivingInterface"), "DTI Driving Interface deactivated. Disabling drives...");
    return CallbackReturn::SUCCESS;
}

uint32_t DtiDrivingInterface::get_dti_can_id(uint8_t packet_id, uint8_t motor_id) {
    if (use_extended_id_) {
        return (static_cast<uint32_t>(packet_id) << 8) | motor_id;
    } else {
        return (static_cast<uint32_t>(packet_id) << 5) | motor_id;
    }
}

hardware_interface::return_type DtiDrivingInterface::read(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& period) {
    auto frames = socket_can_->rx(rclcpp::get_logger("DtiDrivingInterface"), node_->get_clock());

    for (const auto& msg : *frames) {
        uint8_t packet_id = 0;
        uint8_t motor_id = 0;

        if (use_extended_id_) {
            packet_id = (msg.id >> 8) & 0xFFFFFF;
            motor_id = msg.id & 0xFF;
        } else {
            packet_id = (msg.id >> 5) & 0x7F;
            motor_id = msg.id & 0x1F;
        }

        if (motor_id == left_motor_id_ || motor_id == right_motor_id_) {
            bool is_left = (motor_id == left_motor_id_);

            if (packet_id == 0x20) {
                // Byte 0-3: ERPM (Signed 32-bit, Big Endian)
                int32_t erpm =
                    static_cast<int32_t>((msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3]);
                // Byte 6-7: Input DC voltage (Signed 16-bit, Big Endian, multiplied by 10)
                int16_t dc_v_raw = static_cast<int16_t>((msg.data[6] << 8) | msg.data[7]);

                // Convert ERPM to mechanical rad/s at wheel:
                // ERPM / pole_pairs = motor RPM
                // motor RPM to rad/s = RPM * 2pi / 60
                // wheel rad/s = motor rad/s / gear_ratio
                double motor_rpm = static_cast<double>(erpm) / static_cast<double>(pole_pairs_);
                double motor_rads = motor_rpm * 2.0 * M_PI / 60.0;
                double wheel_vel = motor_rads / gear_ratio_;

                if (is_left) {
                    left_wheel_vel_state_ = wheel_vel;
                    left_wheel_pos_state_ += wheel_vel * period.seconds();
                    left_dc_voltage_ = static_cast<double>(dc_v_raw) / 10.0;
                } else {
                    right_wheel_vel_state_ = wheel_vel;
                    right_wheel_pos_state_ += wheel_vel * period.seconds();
                    right_dc_voltage_ = static_cast<double>(dc_v_raw) / 10.0;
                }

            } else if (packet_id == 0x22) {
                // Byte 0-1: Controller temp (Signed 16-bit, Big Endian, multiplied by 10)
                int16_t c_temp_raw = static_cast<int16_t>((msg.data[0] << 8) | msg.data[1]);
                // Byte 2-3: Motor temp (Signed 16-bit, Big Endian, multiplied by 10)
                int16_t m_temp_raw = static_cast<int16_t>((msg.data[2] << 8) | msg.data[3]);
                // Byte 4: Fault code
                uint8_t fault = msg.data[4];

                if (is_left) {
                    left_inverter_temp_ = static_cast<double>(c_temp_raw) / 10.0;
                    left_motor_temp_ = static_cast<double>(m_temp_raw) / 10.0;
                    left_fault_code_ = fault;
                } else {
                    right_inverter_temp_ = static_cast<double>(c_temp_raw) / 10.0;
                    right_motor_temp_ = static_cast<double>(m_temp_raw) / 10.0;
                    right_fault_code_ = fault;
                }
            }
        }
    }

    // Average variables for diagnostics
    motor_temp_ = (left_motor_temp_ + right_motor_temp_) / 2.0;
    inverter_temp_ = (left_inverter_temp_ + right_inverter_temp_) / 2.0;
    dc_voltage_ = (left_dc_voltage_ + right_dc_voltage_) / 2.0;
    fault_code_ = static_cast<double>(left_fault_code_ > 0 ? left_fault_code_ : right_fault_code_);

    publish_diagnostics();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DtiDrivingInterface::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
    if (std::isnan(left_wheel_vel_cmd_) || std::isnan(right_wheel_vel_cmd_)) {
        return hardware_interface::return_type::OK;
    }

    // Convert target wheel velocities (rad/s) to target ERPM
    // ERPM = (wheel_vel * gear_ratio * 60 / 2pi) * pole_pairs
    double left_motor_rpm = (left_wheel_vel_cmd_ * gear_ratio_ * 60.0) / (2.0 * M_PI);
    int32_t left_target_erpm = static_cast<int32_t>(left_motor_rpm * pole_pairs_);

    double right_motor_rpm = (right_wheel_vel_cmd_ * gear_ratio_ * 60.0) / (2.0 * M_PI);
    int32_t right_target_erpm = static_cast<int32_t>(right_motor_rpm * pole_pairs_);

    // Send Set ERPM Command (Packet 0x03)
    send_set_erpm(left_motor_id_, left_target_erpm);
    send_set_erpm(right_motor_id_, right_target_erpm);

    // Periodically send Drive Enable (Packet 0x0C) to reset watchdog timeout
    send_drive_enable(left_motor_id_, true);
    send_drive_enable(right_motor_id_, true);

    return hardware_interface::return_type::OK;
}

void DtiDrivingInterface::send_drive_enable(uint8_t motor_id, bool enable) {
    driverless_msgs::msg::Can msg;
    msg.id = get_dti_can_id(0x0C, motor_id);
    msg.id_type = use_extended_id_;
    msg.dlc = 8;
    msg.data.resize(8, 0xFF);

    msg.data[0] = enable ? 1 : 0;

    socket_can_->tx(&msg, rclcpp::get_logger("DtiDrivingInterface"));
}

void DtiDrivingInterface::send_set_erpm(uint8_t motor_id, int32_t target_erpm) {
    driverless_msgs::msg::Can msg;
    msg.id = get_dti_can_id(0x03, motor_id);
    msg.id_type = use_extended_id_;
    msg.dlc = 8;
    msg.data.resize(8, 0xFF);

    // Populate Big Endian signed 32-bit ERPM
    msg.data[0] = (target_erpm >> 24) & 0xFF;
    msg.data[1] = (target_erpm >> 16) & 0xFF;
    msg.data[2] = (target_erpm >> 8) & 0xFF;
    msg.data[3] = target_erpm & 0xFF;

    socket_can_->tx(&msg, rclcpp::get_logger("DtiDrivingInterface"));
}

void DtiDrivingInterface::publish_diagnostics() {
    if (diagnostics_pub_ && diagnostics_pub_->trylock()) {
        auto& diag_msg = diagnostics_pub_->msg_;
        diag_msg.header.stamp = node_->now();
        diag_msg.status.clear();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Driving: DTI Inverters";
        status.hardware_id = "DTI_Dual_Setup";

        if (left_fault_code_ != 0 || right_fault_code_ != 0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            // Decode active fault code names
            std::string fault_msg = "Fault Active: ";
            if (left_fault_code_ != 0) fault_msg += "Left Code " + std::to_string(left_fault_code_) + " ";
            if (right_fault_code_ != 0) fault_msg += "Right Code " + std::to_string(right_fault_code_) + " ";
            status.message = fault_msg;
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Operational";
        }

        status.values.clear();
        diagnostic_msgs::msg::KeyValue dc_v_val;
        dc_v_val.key = "DC Link Voltage (V)";
        dc_v_val.value = std::to_string(dc_voltage_);
        status.values.push_back(dc_v_val);

        diagnostic_msgs::msg::KeyValue inverter_temp_val;
        inverter_temp_val.key = "Average Controller Temp (°C)";
        inverter_temp_val.value = std::to_string(inverter_temp_);
        status.values.push_back(inverter_temp_val);

        diagnostic_msgs::msg::KeyValue motor_temp_val;
        motor_temp_val.key = "Average Motor Temp (°C)";
        motor_temp_val.value = std::to_string(motor_temp_);
        status.values.push_back(motor_temp_val);

        diag_msg.status.push_back(status);
        diagnostics_pub_->unlockAndPublish();
    }
}

}  // namespace qutms_hw_interfaces

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::DtiDrivingInterface, hardware_interface::SystemInterface)
