#include "qev3d_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qev3d_hardware_interface {

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
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
    // state interface. Drive joints must have one effort command interface and two state interfaces (velocity and
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
            traction_joint_ = joint.name;

            // Drive joints have an effort command interface and velocity and position state interfaces
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                             joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
                RCLCPP_FATAL(get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
                             joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
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

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Qev3dHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state) {
    // ...implementation...
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
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Qev3dHardwareInterface::write(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) {
    // ...implementation...
    return hardware_interface::return_type::OK;
}

}  // namespace qev3d_hardware_interface
