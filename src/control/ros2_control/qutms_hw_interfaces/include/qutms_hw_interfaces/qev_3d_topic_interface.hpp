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
#include <set>
#include <string>
#include <variant>
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
using MessageTypeVariant = std::variant<ackermann_msgs::msg::AckermannDriveStamped, driverless_msgs::msg::State,
                                        std_msgs::msg::Float32, std_msgs::msg::Float64>;

using PublisherVariant = std::variant<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr,
                                      rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr,
                                      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
                                      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>;

using SubscriberVariant = std::variant<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr,
                                       rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr,
                                       rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr,
                                       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>;
// Structs to store joint states
struct JointValue {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double effort{0.0};
};

struct Joint {
    explicit Joint(const std::string& name, const std::string& type) : joint_name(name), joint_type(type) {
        state = JointValue();
        command = JointValue();
    }

    Joint() = default;

    std::string joint_name;
    std::string joint_type;
    std::vector<char> command_interfaces;
    // Currently unused: For future use
    std::vector<std::optional<PublisherWrapper>> command_publishers;
    std::vector<char> state_interfaces;
    std::vector<std::optional<SubscriberWrapper>> state_subscribers;
    JointValue state;
    JointValue command;
};

class PublisherWrapper {
   public:
    template <typename T>
    PublisherWrapper(typename rclcpp::Publisher<T>::SharedPtr publisher) : publisher_(publisher) {}

    void publish(const MessageTypeVariant& msgVariant) {
        std::visit(
            [this](auto& msg) {
                using MsgType = std::decay_t<decltype(msg)>;
                if (auto pub =
                        std::get_if<typename rclcpp::Publisher<typename std::decay<decltype(msg)>::type>::SharedPtr>(
                            &publisher_)) {
                    (*pub)->publish(msg);
                }
            },
            msgVariant);
    }

   private:
    std::variant<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr,
                 rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr,
                 rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
                 rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
        publisher_;
};

class SubscriberWrapper {
   public:
    template <typename T>
    SubscriberWrapper(typename rclcpp::Subscription<T>::SharedPtr subscriber) : subscriber_(std::move(subscriber)) {}

    // MessageTypeVariant handle_message(const MessageTypeVariant& msgVariant){
    //     std::visit([this](auto& msg){
    //         using MsgType = std::decay_t<decltype(msg)>;
    //         if (auto sub = std::get_if<rclcpp::Subscription<MsgType>::SharedPtr>(&subscriber_)){
    //             return msg;
    //         }
    //     }, msgVariant);
    // }
   private:
    std::variant<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr,
                 rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr,
                 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr,
                 rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>
        subscriber_;
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
    // const std::map<std::string, std::vector<std::vector<char>>> available_joints = {
    //     {"virtual_steering_hinge_joint",
    //      {{hardware_interface::HW_IF_POSITION}, {hardware_interface::HW_IF_POSITION}, {"steering"}}},
    //     {"virtual_rear_wheel_joint", {{hardware_interface::HW_IF_EFFORT},
    //                                   {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY},
    //                                   {"drive"}}}};
    const std::set<std::string> available_joints = {"virtual_steering_hinge_joint", "virtual_rear_wheel_joint"};
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
};

}  // namespace qutms_hw_interfaces

#endif  // QUTMS_HW_INTERFACES__QEV_3D_TOPIC_INTERFACE_HPP_
