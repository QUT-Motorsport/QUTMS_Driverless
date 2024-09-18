// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "qutms_hw_interfaces/qev_3d_topic_interface.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qutms_hw_interfaces {
hardware_interface::CallbackReturn Qev3dTopicInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read parameters and initialize the hardware
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Populate valid joints as joint objects and save it in hw_interfaces_
    for (const hardware_interface::ComponentInfo& joint_cfg : info_.joints) {
        if (available_joints.find(joint_cfg.name) != available_joints.end()) {
            hw_interfaces_[joint_cfg.name] = Joint(joint_cfg.name);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                        "Joint '%s' is not a supported interface. Skipping", joint_cfg.name.c_str());
        }
    }

    // Create ros Node to publish and subscribe to topics
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=qev_3d_topic_interface_" + info_.name});

    node_ = rclcpp::Node::make_shared("_", options);

    // Function to set publish and subscription topic based on parameter or default
    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };

    // Setup publisher and subscribers based on configured parameters or configuration
    ackermann_pub_ = node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        get_hardware_parameter("ackermann_topic", "/ackermann_cmd"), rclcpp::QoS(1));
    status_sub_ = node_->create_subscription<driverless_msgs::msg::State>(
        get_hardware_parameter("status_topic", "/status"), rclcpp::QoS(1),
        [this](const driverless_msgs::msg::State::SharedPtr status) { last_status_ = *status; });
    velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        get_hardware_parameter("velocity_topic", "/velocity"), rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr velocity) { last_velocity_ = *velocity; });
    position_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        get_hardware_parameter("position_topic", "/position"), rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr position) { last_position_ = *position; });
    steering_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        get_hardware_parameter("steering_angle_topic", "/steering_angle"), rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32::SharedPtr steering_angle) { last_steering_angle_ = *steering_angle; });

    // Get Driving joint name (Bicycle)
    driving_joint_ = get_hardware_parameter("driving_joint", "virtual_rear_wheel_joint");
    // Get Steering joint name (Bicycle)
    steering_joint_ = get_hardware_parameter("steering_joint", "virtual_steering_hinge_joint");

    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been initialized");

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dTopicInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been configured");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Qev3dTopicInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto& joint : hw_interfaces_) {
        for (auto& interface : available_joints.find(joint.second.joint_name)[1]) {
            if (interface == hardware_interface::HW_IF_POSITION) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(joint.second.joint_name, interface,
                                                                                 &joint.second.state.position));
            } else if (interface == hardware_interface::HW_IF_VELOCITY) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(joint.second.joint_name, interface,
                                                                                 &joint.second.state.velocity));
            } else if (interface == hardware_interface::HW_IF_ACCELERATION) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(joint.second.joint_name, interface,
                                                                                 &joint.second.state.acceleration));
            } else if (interface == hardware_interface::HW_IF_EFFORT) {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(joint.second.joint_name, interface, &joint.second.state.effort));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                            "State interface '%s' is not supported for joint '%s'. Skipping", interface.c_str(),
                            joint.second.joint_name.c_str());
            }
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Qev3dTopicInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto& joint_interface : hw_interfaces_) {
        for (auto& interface : available_joints.find(joint_interface.second.joint_name)[0]) {
            if (interface == hardware_interface::HW_IF_POSITION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_POSITION,
                    &joint_interface.second.command.position));
            } else if (interface == hardware_interface::HW_IF_VELOCITY) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_VELOCITY,
                    &joint_interface.second.command.velocity));
            } else if (interface == hardware_interface::HW_IF_ACCELERATION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_ACCELERATION,
                    &joint_interface.second.command.acceleration));
            } else if (interface == hardware_interface::HW_IF_EFFORT) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_EFFORT,
                    &joint_interface.second.command.effort));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                            "Command interface '%s' is not supported for joint '%s'. Skipping", interface.c_str(),
                            joint_interface.second.joint_name.c_str());
            }
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn Qev3dTopicInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to receive commands
    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been activated");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dTopicInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to stop receiving commands
    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been deactivated");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Qev3dTopicInterface::read(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& /*period*/) {
    if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }

    for (auto& joint : hw_interfaces_) {
        if (available_joints.find(joint.second.joint_name)[2] == "steering") {
            joint.second.state.position = last_steering_angle_.data;
        } else if (available_joints.find(joint.second.joint_name)[2] == "drive") {
            joint.second.state.position = last_position_.data;
            joint.second.state.velocity = last_velocity_.data;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Qev3dTopicInterface::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
    if (last_status_.data.state == driverless_msgs::msg::State::DRIVING) {
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
        ackermann_msg.header.stamp = node_->now();
        ackermann_msg.header.frame_id = "base_footprint";
        ackermann_msg.drive.steering_angle = hw_interfaces_[steering_joint_].command.position;
        ackermann_msg.drive.speed = hw_interfaces_[driving_joint_].command.effort;
        ackermann_pub_->publish(ackermann_msg);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace qutms_hw_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::Qev3dTopicInterface, hardware_interface::SystemInterface)
