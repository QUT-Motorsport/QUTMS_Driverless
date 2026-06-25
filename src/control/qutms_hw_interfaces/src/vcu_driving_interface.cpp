#include "qutms_hw_interfaces/vcu_driving_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

// Include embedded headers from QUTMS_Embedded_Common
#include "CAN_AV.h"
#include "CAN_VESC.h"

namespace qutms_hw_interfaces {

hardware_interface::CallbackReturn VcuDrivingInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read parameters from URDF xacro
    can_interface_name_ =
        info_.hardware_parameters.count("can_interface") ? info_.hardware_parameters.at("can_interface") : "can0";
    wheel_radius_ = info_.hardware_parameters.count("wheel_radius")
                        ? std::stod(info_.hardware_parameters.at("wheel_radius"))
                        : 0.2032;
    kp_ = info_.hardware_parameters.count("Kp") ? std::stod(info_.hardware_parameters.at("Kp")) : 0.05;
    ki_ = info_.hardware_parameters.count("Ki") ? std::stod(info_.hardware_parameters.at("Ki")) : 0.0;
    max_integral_torque_ = info_.hardware_parameters.count("max_integral_torque")
                               ? std::stod(info_.hardware_parameters.at("max_integral_torque"))
                               : 0.0;

    left_wheel_pos_state_ = 0.0;
    left_wheel_vel_state_ = 0.0;
    right_wheel_pos_state_ = 0.0;
    right_wheel_vel_state_ = 0.0;

    left_wheel_vel_cmd_ = 0.0;
    right_wheel_vel_cmd_ = 0.0;

    integral_error_ = 0.0;
    prev_accel_ = 0.0;
    target_steering_angle_ = 0.0;

    // Initialize ROS 2 Node
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=vcu_driving_interface_node"});
    node_ = rclcpp::Node::make_shared("_", options);

    // Subscribe to Ackermann command for target steering angle
    ackermann_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/driving_command", 10,
        std::bind(&VcuDrivingInterface::ackermann_callback, this, std::placeholders::_1));

    diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));

    socket_can_ = std::make_unique<SocketCAN>();

    RCLCPP_INFO(rclcpp::get_logger("VcuDrivingInterface"),
                "VCU Driving Interface initialized. CAN: %s, Wheel Radius: %.4fm", can_interface_name_.c_str(),
                wheel_radius_);
    return CallbackReturn::SUCCESS;
}

void VcuDrivingInterface::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    target_steering_angle_ = msg->drive.steering_angle;
}

hardware_interface::CallbackReturn VcuDrivingInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!socket_can_->setup(can_interface_name_, rclcpp::get_logger("VcuDrivingInterface"))) {
        RCLCPP_ERROR(rclcpp::get_logger("VcuDrivingInterface"), "Failed to setup SocketCAN on %s",
                     can_interface_name_.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VcuDrivingInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // Export states for left wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_state_));

    // Export states for right wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_pos_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_state_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VcuDrivingInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // Export commands
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_cmd_));
    return command_interfaces;
}

hardware_interface::CallbackReturn VcuDrivingInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("VcuDrivingInterface"), "VCU Driving Interface activated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VcuDrivingInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("VcuDrivingInterface"), "VCU Driving Interface deactivated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type VcuDrivingInterface::read(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& period) {
    auto frames = socket_can_->rx(rclcpp::get_logger("VcuDrivingInterface"), node_->get_clock());

    for (const auto& msg : *frames) {
        uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
        uint8_t vesc_id = msg.id & 0xFF;

        if (vesc_masked_id == VESC_CAN_PACKET_STATUS) {
            uint8_t data[8];
            std::memcpy(data, msg.data.data(), std::min(static_cast<size_t>(msg.dlc), sizeof(data)));

            int32_t rpm;
            float current;
            float duty;
            Parse_VESC_CANPacketStatus(data, &rpm, &current, &duty);

            // Convert RPM to linear velocity (rad/s at wheel)
            // Motor RPM to rad/s = RPM * 2pi / 60
            // Considering gearbox ratio (e.g. 4.50) and pole pairs if needed (Parse_VESC_CANPacketStatus outputs
            // electrical RPM or mechanical RPM? In component_canbus_translator.cpp: (rpm / (21.0 * 4.50)) * M_PI *
            // WHEEL_DIAMETER / 60 where 21.0 is pole pairs, 4.50 is gear ratio. So mechanical wheel velocity (rad/s) =
            // (rpm / (pole_pairs * gear_ratio)) * 2 * pi / 60
            double wheel_vel_rads = (static_cast<double>(rpm) / (21.0 * 4.50)) * 2.0 * M_PI / 60.0;

            if (vesc_id == 2) {  // Left wheel
                left_wheel_vel_state_ = wheel_vel_rads;
                left_wheel_pos_state_ += wheel_vel_rads * period.seconds();
            } else if (vesc_id == 3) {  // Right wheel
                right_wheel_vel_state_ = wheel_vel_rads;
                right_wheel_pos_state_ += wheel_vel_rads * period.seconds();
            }
        }
    }

    // Spin node to receive Ackermann subscriber callbacks
    rclcpp::spin_some(node_);

    publish_diagnostics();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VcuDrivingInterface::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
    if (std::isnan(left_wheel_vel_cmd_) || std::isnan(right_wheel_vel_cmd_)) {
        return hardware_interface::return_type::OK;
    }

    // Average target velocity in rad/s
    double target_wheel_vel = (left_wheel_vel_cmd_ + right_wheel_vel_cmd_) / 2.0;
    // Current average velocity in rad/s
    double current_wheel_vel = (left_wheel_vel_state_ + right_wheel_vel_state_) / 2.0;

    // Convert rad/s to m/s for VCU speed field
    double target_speed_mps = target_wheel_vel * wheel_radius_;
    double current_speed_mps = current_wheel_vel * wheel_radius_;

    // Calculate error and control loop
    double error = target_speed_mps - current_speed_mps;
    integral_error_ += error;

    // Clip integral error
    if (ki_ > 0.0) {
        if (integral_error_ < 0.0) {
            integral_error_ = 0.0;
        } else if (integral_error_ > (max_integral_torque_ / ki_)) {
            integral_error_ = max_integral_torque_ / ki_;
        }
    } else {
        integral_error_ = 0.0;
    }

    double accel = kp_ * error + ki_ * integral_error_;
    // Limit acceleration demand
    accel = std::clamp(accel, -1.0, 1.0);

    // Create Request message
    Request_t request_msg;
    request_msg.torque = static_cast<int16_t>(accel * 100.0);  // convert to percentage
    request_msg.steering =
        static_cast<int16_t>(target_steering_angle_ * 10.0);             // convert angle to scale 10 if needed by VCU
    request_msg.speed = static_cast<int16_t>(target_speed_mps * 100.0);  // scale 100 for speed

    auto request_heartbeat = Compose_Request_Heartbeat(&request_msg);

    // Build CAN message
    driverless_msgs::msg::Can msg;
    msg.id = request_heartbeat.id;
    msg.id_type = true;  // VCU requests are usually extended frames
    msg.dlc = sizeof(request_heartbeat.data);
    msg.data.resize(msg.dlc);
    std::memcpy(msg.data.data(), request_heartbeat.data, msg.dlc);

    socket_can_->tx(&msg, rclcpp::get_logger("VcuDrivingInterface"));

    prev_accel_ = accel;

    return hardware_interface::return_type::OK;
}

void VcuDrivingInterface::publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = node_->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Driving: VCU Heartbeat Interface";
    status.hardware_id = "VCU_Gateway";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Operational";

    diagnostic_msgs::msg::KeyValue target_speed_val;
    target_speed_val.key = "Target Speed (m/s)";
    target_speed_val.value = std::to_string(((left_wheel_vel_cmd_ + right_wheel_vel_cmd_) / 2.0) * wheel_radius_);
    status.values.push_back(target_speed_val);

    diagnostic_msgs::msg::KeyValue current_speed_val;
    current_speed_val.key = "Current Speed (m/s)";
    current_speed_val.value = std::to_string(((left_wheel_vel_state_ + right_wheel_vel_state_) / 2.0) * wheel_radius_);
    status.values.push_back(current_speed_val);

    diagnostic_msgs::msg::KeyValue torque_req_val;
    torque_req_val.key = "Torque Request (%)";
    torque_req_val.value = std::to_string(prev_accel_ * 100.0);
    status.values.push_back(torque_req_val);

    diag_msg.status.push_back(status);
    diagnostics_pub_->publish(diag_msg);
}

}  // namespace qutms_hw_interfaces

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::VcuDrivingInterface, hardware_interface::SystemInterface)
