#include "qev3d_ros2_control/qev3d_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qev3d_ros2_control {

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
                     "Qev3dHardwareInterface::on_init() - Failed to initialize, "
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

    // Initialize the configuration parameters for the hardware interface
    config_.drive_joint = info_.joints[1].name;     // info_.joints[1] is the drive joint (virtual_rear_wheel_joint)
    config_.steering_joint = info_.joints[0].name;  // info_.joints[0] is the steering joint (virtual_front_wheel_joint)
    config_.hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    config_.hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Configure the hardware interface with the given lifecycle state
hardware_interface::CallbackReturn Qev3dHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    for (int i = 0; i < config_.hw_start_sec_; i++) {
        RCLCPP_INFO(get_logger(), "%.1f seconds left... [ on_configure() ]",
                    static_cast<float>(config_.hw_start_sec_ - i));
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Reset values when configuring hardware
    for (const auto& [name, descr] : joint_state_interfaces_) {
        set_state(name, 0.0);
    }
    for (const auto& [name, descr] : joint_command_interfaces_) {
        set_command(name, 0.0);
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup the hardware interface with the given lifecycle state
hardware_interface::CallbackReturn Qev3dHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    // Implementation for cleanup state
    RCLCPP_INFO(get_logger(), "Cleaning up hardware interface.");
    // ...additional cleanup code...
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Activate the hardware interface with the given lifecycle state
hardware_interface::CallbackReturn Qev3dHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (auto i = 0; i < config_.config_.hw_start_sec_; i++) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "%.1f seconds left.. [ on_activate() ].", config_.hw_start_sec_ - i);
    }

    // command and state should be equal when starting
    for (const auto& [name, descr] : joint_command_interfaces_) {
        set_command(name, get_state(name));
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivate the hardware interface with the given lifecycle state
hardware_interface::CallbackReturn Qev3dHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    for (auto i = 0; i < config_.hw_stop_sec_; i++) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "%.1f seconds left... [ on_deactivate() ]", config_.hw_stop_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Read the current state of the hardware
hardware_interface::return_type Qev3dHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    // update states from commands and integrate velocity to position
    set_state(steering_joint_ + "/" + hardware_interface::HW_IF_POSITION,
              get_command(steering_joint_ + "/" + hardware_interface::HW_IF_POSITION));

    set_state(drive_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
              get_command(drive_joint_ + "/" + hardware_interface::HW_IF_VELOCITY));
    set_state(drive_joint_ + "/" + hardware_interface::HW_IF_POSITION,
              get_state(drive_joint_ + "/" + hardware_interface::HW_IF_POSITION) +
                  get_command(drive_joint_ + "/" + hardware_interface::HW_IF_VELOCITY) * period.seconds());

    std::stringstream ss;
    ss << "Reading states:";

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "position: " << get_state(steering_joint_ + "/" + hardware_interface::HW_IF_POSITION) << " for joint '"
       << steering_joint_ << "'" << std::endl
       << "\t"
       << "position: " << get_state(drive_joint_ + "/" + hardware_interface::HW_IF_POSITION) << " for joint '"
       << drive_joint_ << "'" << std::endl
       << "\t"
       << "velocity: " << get_state(drive_joint_ + "/" + hardware_interface::HW_IF_VELOCITY) << " for joint '"
       << drive_joint_ << "'";

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

// Write the command to the hardware
hardware_interface::return_type Qev3dHardwareInterface::write(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    std::stringstream ss;
    ss << "Writing commands:";

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "position: " << get_command(steering_joint_ + "/" + hardware_interface::HW_IF_POSITION) << " for joint '"
       << steering_joint_ << "'" << std::endl
       << "\t"
       << "velocity: " << get_command(drive_joint_ + "/" + hardware_interface::HW_IF_VELOCITY) << " for joint '"
       << drive_joint_ << "'";

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
}

}  // namespace qev3d_ros2_control
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(qev3d_ros2_control::Qev3dHardwareInterface, hardware_interface::SystemInterface)
