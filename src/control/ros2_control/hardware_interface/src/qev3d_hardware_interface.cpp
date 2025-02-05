#include "qev3d_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qev3d_hardware_interface {

// pass in the hardware info (command, state interfaces and joints) in the qev3d_ros2_control.xacro
hardware_interface::CallbackReturn Qev3dHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    /* For now, info_ is a member variable of the class Qev3dHardwareInterface, storing:
        info_.joints = {
            {
                name: "virtual_front_wheel_joint",
                state_interfaces: ["position"],
                command_interfaces: ["position"]
            },
            {
                name: "virtual_rear_wheel_joint",
                state_interfaces: ["position", "velocity"],
                command_interfaces: ["velocity"]
            }
        }

        info_.hardware_parameters = {
            {
                name: "wheel_radius",
                value: "0.03"
            },
            {
                name: "wheel_base",
                value: "0.1"
            }
        }
    */

    // Check if the parent class hardware_interface::SystemInterface can be initialized
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if the number of joints is correct based on the mode of operation
    if (info_.joints.size() != 2) {
        RCLCPP_ERROR(get_logger(),
                     "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
                     "because the number of joints %ld is not 2.",
                     info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // This code checks if there are exactly 2 joints. It then iterates through the joints to identify and validate them
    // as either steering or drive joints. Steering joints must have one position command interface and one position
    // state interface. Drive joints must have one velocity command interface and two state interfaces (velocity and
    // position). If any joint does not meet the expected criteria, it logs an error and returns an error status.
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        bool joint_is_steering = joint.name.find("front") != std::string::npos;

        // Steering joints have a position command interface and a position state interface
        if (joint_is_steering) {
            steering_joint_ = joint.name;
            RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                             joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                             joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                             joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());
            drive_joint_ = joint.name;

            // Drive joints have an velocity command interface and velocity and position state interfaces
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                             joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.", joint.name.c_str(),
                             joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                             joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
                             joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }

    config_.drive_joint = info_.hardware_parameters["virtual_rear_wheel_joint"];
    config_.steering_joint = info_.hardware_parameters["virtual_front_wheel_joint"];
    config_.hw_start_sec = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    config_.hw_stop_sec = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    // ...implementation...

    // can_translator_node_.set_interface();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Implementation for cleanup state
    RCLCPP_INFO(get_logger(), "Cleaning up hardware interface.");
    // ...additional cleanup code...
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    // ...implementation...
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    // ...implementation...
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Qev3dHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // ...implementation...

    // can_translator_node_.canmsg_timer();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Qev3dHardwareInterface::write(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) {
    // ...implementation...

    // can_translator_node_.canmsg_callback();
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> Qev3dHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(steering_joint_, hardware_interface::HW_IF_POSITION, &command.position));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(drive_joint_, hardware_interface::HW_IF_VELOCITY, &command.velocity));
    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> Qev3dHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(steering_joint_, hardware_interface::HW_IF_POSITION, &state.position));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(drive_joint_, hardware_interface::HW_IF_VELOCITY, &state.velocity));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(drive_joint_, hardware_interface::HW_IF_POSITION, &state.position));
    return state_interfaces;
}

}  // namespace qev3d_hardware_interface
