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

#ifndef QUTMS_HW_INTERFACES__QEV_3D_TOPIC_INTERFACE_HPP_
#define QUTMS_HW_INTERFACES__QEV_3D_TOPIC_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "qutms_hw_interfaces/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

namespace qutms_hw_interfaces {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
// Structs to store joint states
struct JointValue {
    _Float64 position{0.0};
    _Float32 velocity{0.0};
    _Float64 acceleration{0.0};
    double effort{0.0};
};

struct Joint {
    explicit Joint(const std::string& name) : joint_name(name) {
        state = JointValue();
        command = JointValue();
    }

    Joint() = default;

    std::string joint_name;
    JointValue state;
    JointValue command;
};
class Qev3dTopicInterface : public hardware_interface::SystemInterface {
   public:
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    // Definition for joints that are configured for this interface
    // Format:
    // {
    //   (joint_name,
    //   {
    //    {
    //      command_interfaces(Unlimited Unique)->
    //      {
    //        position/velocity/acceleration/effort/custom_interface
    //      }
    //    },
    //    {
    //      state_interfaces(Unlimited Unique)->
    //      {
    //        position/velocity/acceleration/effort/custom_interface
    //      },
    //    },
    //    {Joint_type(1 Unique)->{steering/drive/other}}
    //   }),
    const std::map<std::string, std::vector<std::vector<std::string>>> available_joints = {
        ("virtual_steering_hinge_joint",
         {{hardware_interface::HW_IF_POSITION}, {hardware_interface::HW_IF_POSITION}, {"steering"}}),
        ("virtual_rear_wheel_joint", {{hardware_interface::HW_IF_EFFORT},
                                      {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY},
                                      {"drive"}})};
    std::map<std::string, Joint> hw_interfaces_;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
    rclcpp::Node::SharedPtr node_;
    driverless_msgs::msg::State last_status_;
    std_msgs::msg::Float32 last_velocity_;
    std_msgs::msg::Float64 last_position_;
    std_msgs::msg::Float32 last_steering_angle_;
    std::string driving_joint_;
    std::string steering_joint_;

    struct config {
        // Use the following for Tricycle drive:
        std::string left_wheel_joint = "";
        std::string right_wheel_joint = "";
        // For Bicycle Drive:
        std::string rear_wheel_joint = "";
        // Publishing and Subscription Topics
        std::string ackermann_pub_topic = "";
        std::string steering_sub_topic = "";
        std::string velocity_sub_topic = "";
        std::string position_sub_topic = "";
        std::string status_sub_topic = "";
    };
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__QEV_3D_TOPIC_INTERFACE_HPP_
