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
    hw_states_.resize(info_.joints.size(), ::std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), ::std::numeric_limits<double>::quiet_NaN());

    // Create ros Node to publish and subscribe to topics
    rclcpp::NodeOptions options;
    std::string node_name;
    if (info_.name == "qev-3d") {
        node_name = "qev_3d";
    } else {
        node_name = info_.name;
    }
    options.arguments({"--ros-args", "-r", "__node:=qev_3d_topic_interface_" + node_name});

    node_ = rclcpp::Node::make_shared("_", options);

    // Populate valid joints as joint objects and save it in hw_interfaces_
    for (const hardware_interface::ComponentInfo& joint_cfg : info_.joints) {
        if (available_joints.find(joint_cfg.name) != available_joints.end()) {
            hw_interfaces_[joint_cfg.name] = Joint(joint_cfg.name, joint_cfg.parameters.find("joint_type")->second);

            // Create publishers and subscribers for each joint using URDF parameters,
            // creating null optionals if no data type is provided
            for (auto command_interface : joint_cfg.command_interfaces) {
                hw_interfaces_[joint_cfg.name].command_interfaces.emplace_back(command_interface.name);
                if (command_interface.data_type != "") {
                    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher =
                        node_->create_publisher<std_msgs::msg::Float64>(command_interface.data_type, rclcpp::QoS(1));
                    hw_interfaces_[joint_cfg.name].command_publishers.emplace_back(publisher);
                } else {
                    hw_interfaces_[joint_cfg.name].command_publishers.emplace_back(std::nullopt);
                }
            }
            // Create subscribers based on data type, and joint type and saved in vector of optionals.
            // If no data type is provided, create a null optional
            for (auto state_interface : joint_cfg.state_interfaces) {
                hw_interfaces_[joint_cfg.name].state_interfaces.emplace_back(state_interface.name);
                if (state_interface.data_type != "") {
                    if (state_interface.data_type == hardware_interface::HW_IF_POSITION) {
                        if (joint_cfg.parameters.find("joint_type")->second == "driving") {
                            auto subscriber = node_->create_subscription<std_msgs::msg::Float64>(
                                state_interface.data_type, rclcpp::SensorDataQoS(),
                                [this, joint_cfg](const std_msgs::msg::Float64::SharedPtr msg) {
                                    hw_interfaces_[joint_cfg.name].state.position = msg->data;
                                });
                            hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(subscriber);
                        } else {
                            auto subscriber = node_->create_subscription<std_msgs::msg::Float32>(
                                state_interface.data_type, rclcpp::SensorDataQoS(),
                                [this, joint_cfg](const std_msgs::msg::Float32::SharedPtr msg) {
                                    hw_interfaces_[joint_cfg.name].state.position = msg->data;
                                });
                            hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(subscriber);
                        }
                    } else if (state_interface.data_type == hardware_interface::HW_IF_VELOCITY) {
                        auto subscriber = node_->create_subscription<std_msgs::msg::Float64>(
                            state_interface.data_type, rclcpp::SensorDataQoS(),
                            [this, joint_cfg](const std_msgs::msg::Float64::SharedPtr msg) {
                                hw_interfaces_[joint_cfg.name].state.velocity = msg->data;
                            });
                        hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(subscriber);
                    } else if (state_interface.data_type == hardware_interface::HW_IF_ACCELERATION) {
                        auto subscriber = node_->create_subscription<std_msgs::msg::Float64>(
                            state_interface.data_type, rclcpp::SensorDataQoS(),
                            [this, joint_cfg](const std_msgs::msg::Float64::SharedPtr msg) {
                                hw_interfaces_[joint_cfg.name].state.acceleration = msg->data;
                            });
                        hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(subscriber);
                    } else if (state_interface.data_type == hardware_interface::HW_IF_EFFORT) {
                        auto subscriber = node_->create_subscription<std_msgs::msg::Float64>(
                            state_interface.data_type, rclcpp::SensorDataQoS(),
                            [this, joint_cfg](const std_msgs::msg::Float64::SharedPtr msg) {
                                hw_interfaces_[joint_cfg.name].state.effort = msg->data;
                            });
                        hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(subscriber);
                    }
                } else {
                    hw_interfaces_[joint_cfg.name].state_subscribers.emplace_back(std::nullopt);
                }
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                        "Joint '%s' is not a supported interface. Skipping", joint_cfg.name.c_str());
        }
    }

    // Function to set publish and subscription topic based on parameter or defaults
    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };

    // Setup publisher and subscribers that cannot be correlated to a single joint
    // based on configured parameters or default values
    ackermann_pub_ = node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        get_hardware_parameter("ackermann_topic", "vehicle/ackermann_cmd"), rclcpp::QoS(1));
    status_sub_ = node_->create_subscription<driverless_msgs::msg::State>(
        get_hardware_parameter("status_topic", "/status"), rclcpp::QoS(1),
        [this](const driverless_msgs::msg::State::SharedPtr status) { last_status_ = *status; });
    // velocity_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    //     get_hardware_parameter("velocity_topic", "/velocity"), rclcpp::SensorDataQoS(),
    //     [this](const std_msgs::msg::Float32::SharedPtr velocity) { last_velocity_ = *velocity; });
    // position_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    //     get_hardware_parameter("position_topic", "/position"), rclcpp::SensorDataQoS(),
    //     [this](const std_msgs::msg::Float64::SharedPtr position) { last_position_ = *position; });
    // steering_angle_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    //     get_hardware_parameter("steering_angle_topic", "/steering_angle"), rclcpp::SensorDataQoS(),
    //     [this](const std_msgs::msg::Float32::SharedPtr steering_angle) { last_steering_angle_ = *steering_angle; });

    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been initialized");

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dTopicInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "QEV3D Hardware Interface has been configured");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief Exports the state interfaces for the Qev3dTopicInterface.
 *
 * This function iterates over the hardware interfaces and exports the state interfaces
 * (Pointer to the state vector elements of the joint struct) for each joint.
 * It supports the following state interfaces:
 * - Position
 * - Velocity
 * - Acceleration
 * - Effort
 *
 * If a state interface is not supported, a warning is logged.
 *
 * @return A vector of hardware_interface::StateInterface objects representing the state interfaces.
 */
std::vector<hardware_interface::StateInterface> Qev3dTopicInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto& joint : hw_interfaces_) {
        for (std::string state_interface : joint.second.state_interfaces) {
            RCLCPP_INFO(rclcpp::get_logger("qev_3d_topic_interface"), "State interface: %s", state_interface.c_str());
            if (state_interface == hardware_interface::HW_IF_POSITION) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));
            } else if (state_interface == hardware_interface::HW_IF_VELOCITY) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
            } else if (state_interface == hardware_interface::HW_IF_ACCELERATION) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint.second.joint_name, hardware_interface::HW_IF_ACCELERATION, &joint.second.state.acceleration));
            } else if (state_interface == hardware_interface::HW_IF_EFFORT) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    joint.second.joint_name, hardware_interface::HW_IF_EFFORT, &joint.second.state.effort));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                            "State interface '%s' is not supported for joint '%s'. Skipping", state_interface.c_str(),
                            joint.second.joint_name.c_str());
            }
        }
    }

    return state_interfaces;
}

/**
 * @brief Exports the command interfaces for the Qev3dTopicInterface.
 *
 * This function iterates over the hardware interfaces and exports the command interfaces
 * (Pointer to the command vector elements of the joint struct) for each joint.
 * It supports the following state interfaces:
 * - Position
 * - Velocity
 * - Acceleration
 * - Effort
 * If a command interface is not supported, a warning is logged.
 *
 * @return A vector of hardware_interface::CommandInterface objects representing the exported command interfaces.
 */
std::vector<hardware_interface::CommandInterface> Qev3dTopicInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto& joint_interface : hw_interfaces_) {
        for (std::string command_interface : joint_interface.second.command_interfaces) {
            if (command_interface == hardware_interface::HW_IF_POSITION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_POSITION,
                    &joint_interface.second.command.position));
            } else if (command_interface == hardware_interface::HW_IF_VELOCITY) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_VELOCITY,
                    &joint_interface.second.command.velocity));
            } else if (command_interface == hardware_interface::HW_IF_ACCELERATION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_ACCELERATION,
                    &joint_interface.second.command.acceleration));
            } else if (command_interface == hardware_interface::HW_IF_EFFORT) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    joint_interface.second.joint_name, hardware_interface::HW_IF_EFFORT,
                    &joint_interface.second.command.effort));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("qev_3d_topic_interface"),
                            "Command interface '%s' is not supported for joint '%s'. Skipping",
                            command_interface.c_str(), joint_interface.second.joint_name.c_str());
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

    // for (auto& joint : hw_interfaces_) {
    //     if (&available_joints.find(joint.second.joint_name)->second[2][0] == "steering") {
    //         joint.second.state.position = last_steering_angle_.data;
    //     } else if (&available_joints.find(joint.second.joint_name)->second[2][0] == "drive") {
    //         joint.second.state.position = last_position_.data;
    //         joint.second.state.velocity = last_velocity_.data;
    //     }
    // }
    return hardware_interface::return_type::OK;
}

// Publishes the message to the publisher based on the message type
void publishMessage(const MessageTypeVariant& msg_var, const PublisherVariant& publisher_) {
    using T = std::decay_t<decltype(msg_var)>;
    if (std::is_same<T, std_msgs::msg::Float64>::value) {
        std_msgs::msg::Float64 msg = std::get<std_msgs::msg::Float64>(msg_var);
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub =
            std::get<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>(publisher_);
        pub->publish(msg);
    } else if (std::is_same<T, std_msgs::msg::Float32>::value) {
        std_msgs::msg::Float32 msg = std::get<std_msgs::msg::Float32>(msg_var);
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub =
            std::get<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>(publisher_);
        pub->publish(msg);
    }
}

/**
 * @brief Writes commands to the hardware interface.
 *
 * This function iterates over the hardware interfaces and publishes the appropriate command messages
 * based on the type of command interface (position, velocity, or acceleration).
 * It also fuses steering and driving commands into an AckermannDriveStamped message and publishes it.
 *
 * @return hardware_interface::return_type The result of the write operation.
 */
hardware_interface::return_type Qev3dTopicInterface::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
    if (last_status_.state == driverless_msgs::msg::State::DRIVING) {
        float steering_angle_cmd = 0.0;
        float velocity_cmd = 0.0;
        for (auto& joint : hw_interfaces_) {
            for (size_t i = 0; i < joint.second.command_interfaces.size(); i++) {
                qutms_hw_interfaces::MessageTypeVariant msg;
                if (joint.second.command_interfaces[i] == hardware_interface::HW_IF_POSITION &&
                    joint.second.joint_type == "steering") {
                    msg = std_msgs::msg::Float32();
                    std::get<std_msgs::msg::Float32>(msg).data = static_cast<float>(joint.second.command.position);
                    steering_angle_cmd = std::get<std_msgs::msg::Float32>(msg).data;
                } else if (joint.second.command_interfaces[i] == hardware_interface::HW_IF_POSITION &&
                           joint.second.joint_type == "driving") {
                    msg = std_msgs::msg::Float64();
                    std::get<std_msgs::msg::Float64>(msg).data = joint.second.command.position;
                } else if (joint.second.command_interfaces[i] == hardware_interface::HW_IF_VELOCITY) {
                    msg = std_msgs::msg::Float64();
                    std::get<std_msgs::msg::Float64>(msg).data = joint.second.command.velocity;
                } else if (joint.second.command_interfaces[i] == hardware_interface::HW_IF_ACCELERATION) {
                    msg = std_msgs::msg::Float64();
                    std::get<std_msgs::msg::Float64>(msg).data = joint.second.command.acceleration;
                } else if (joint.second.command_interfaces[i] == hardware_interface::HW_IF_EFFORT) {
                    msg = std_msgs::msg::Float64();
                    std::get<std_msgs::msg::Float64>(msg).data = joint.second.command.effort;
                    velocity_cmd = static_cast<float>(std::get<std_msgs::msg::Float64>(msg).data);
                } else {
                    continue;
                }
                if (joint.second.command_publishers[i].has_value()) {
                    publishMessage(msg, joint.second.command_publishers[i].value());
                }
            }
        }
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
        ackermann_msg.header.stamp = node_->now();
        ackermann_msg.header.frame_id = "base_footprint";
        ackermann_msg.drive.steering_angle = steering_angle_cmd;
        ackermann_msg.drive.speed = velocity_cmd;
        ackermann_pub_->publish(ackermann_msg);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace qutms_hw_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::Qev3dTopicInterface, hardware_interface::SystemInterface)
