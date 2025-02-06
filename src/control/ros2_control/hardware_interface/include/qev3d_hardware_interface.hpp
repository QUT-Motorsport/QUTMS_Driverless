// Copyright 2021 ros2_control Development Team
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

#ifndef QEV3D_HARDWARE_INTERFACE_HPP_
#define QEV3D_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware/canbus/include/component_canbus_translator.hpp"

//#include "canbus/can_translator.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace qev3d_hardware_interface {
struct JointValue {
    double position{0.0};
    double velocity{0.0};
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
class Qev3dHardwareInterface : public hardware_interface::SystemInterface {
    struct Config {
        std::string steering_joint = "";
        std::string drive_joint = "";
        double hw_start_sec;
        double hw_stop_sec;
    };

   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Qev3dHardwareInterface);

    // Initialize the hardware interface with the given hardware information
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    // Configure the hardware interface with the given lifecycle state
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    // Cleanup the hardware interface with the given lifecycle state
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    // Activate the hardware interface with the given lifecycle state
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    // Deactivate the hardware interface with the given lifecycle state
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    // Read the current state of the hardware
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // Write the command to the hardware
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    // Export command interfaces
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Export state interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

   private:
    // Parameters for the simulation
    double hw_start_sec_;  // Time in seconds to start the hardware
    double hw_stop_sec_;   // Time in seconds to stop the hardware

    // Joint names
    std::string steering_joint_;  // Name of the steering joint
    std::string drive_joint_;     // Name of the drive joint

    // CAN translator for CANbus communication
    // std::shared_ptr<canbus::CANTranslator> can_translator_node_;

    // Config parameters
    Config config_;
};

}  // namespace qev3d_hardware_interface

#endif  // QEV3D_HARDWARE_INTERFACE_HPP_
