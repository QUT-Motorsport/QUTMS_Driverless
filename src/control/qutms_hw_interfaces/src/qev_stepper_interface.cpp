#include "qutms_hw_interfaces/qev_stepper_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "CAN_VCU.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace qutms_hw_interfaces {

// Constants from steering actuator package
static constexpr uint16_t HOME_OFFSET = 0x607C;
static constexpr uint16_t PROFILE_VELOCITY = 0x6081;
static constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
static constexpr uint16_t PROFILE_DECELERATION = 0x6084;
static constexpr uint16_t QUICK_STOP_DECELERATION = 0x6085;
static constexpr uint16_t MAX_ACCELERATION = 0x60C5;
static constexpr uint16_t MAX_DECELERATION = 0x60C6;
static constexpr uint16_t MODE_OF_OPERATION = 0x6060;
static constexpr uint16_t TARGET_POSITION = 0x607A;
static constexpr uint16_t CONTROL_WORD = 0x6040;
static constexpr uint16_t STATUS_WORD = 0x6041;
static constexpr uint16_t POSITION_ACTUAL_VAL = 0x6064;

static constexpr uint16_t NRTSO_VAL = 0b0000000000000000;
static constexpr uint16_t SOD_VAL = 0b0000000001000000;
static constexpr uint16_t RTSO_VAL = 0b0000000000100001;
static constexpr uint16_t SO_VAL = 0b0000000000100011;
static constexpr uint16_t OE_VAL = 0b0000000000100111;
static constexpr uint16_t QSA_VAL = 0b0000000000000111;
static constexpr uint16_t FRA_VAL = 0b0000000000001111;
static constexpr uint16_t F_VAL = 0b0000000000001000;

static const std::map<uint16_t, c5e_state> states = {
    {NRTSO_VAL, {"Not ready to switch on", 0b0000000001001111, NRTSO_VAL, 0b0000}},
    {SOD_VAL, {"Switch on disabled", 0b0000000001001111, SOD_VAL, 0b0000}},
    {RTSO_VAL, {"Ready to switch on", 0b0000000001101111, RTSO_VAL, 0b0110}},
    {SO_VAL, {"Switched on", 0b0000000001101111, SO_VAL, 0b0111}},
    {OE_VAL, {"Operation enabled", 0b0000000001101111, OE_VAL, 0b1111}},
    {QSA_VAL, {"Quick stop active", 0b0000000001101111, QSA_VAL, 0b0000}},
    {FRA_VAL, {"Fault reaction active", 0b0000000001001111, FRA_VAL, 0b0000}},
    {F_VAL, {"Fault", 0b0000000001001111, F_VAL, 0b0000}},
};

static const uint16_t MODE_ABSOLUTE = 0b00101111;
static const uint16_t TRIGGER_MOTION = 0b00010000;
static const uint16_t FAULT_RESET = 0b10000000;

static c5e_state parse_state(uint16_t status_word) {
    for (const auto &[key, actuator_state] : states) {
        if ((status_word & actuator_state.mask) == actuator_state.state_id) {
            return actuator_state;
        }
    }
    return states.at(F_VAL);
}

hardware_interface::CallbackReturn QevStepperInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Read parameters from URDF xacro
    can_interface_name_ =
        info_.hardware_parameters.count("can_interface") ? info_.hardware_parameters.at("can_interface") : "can0";
    node_id_ = info_.hardware_parameters.count("node_id")
                   ? static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("node_id")))
                   : 0x70;
    max_position_ = info_.hardware_parameters.count("max_position")
                        ? std::stoi(info_.hardware_parameters.at("max_position"))
                        : 7500;
    velocity_ = info_.hardware_parameters.count("velocity")
                    ? static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("velocity")))
                    : 10000;
    acceleration_ = info_.hardware_parameters.count("acceleration")
                        ? static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("acceleration")))
                        : 2000;

    steering_ang_received_ = false;
    current_position_ = 0;
    joint_position_state_ = std::numeric_limits<double>::quiet_NaN();
    joint_position_command_ = std::numeric_limits<double>::quiet_NaN();
    motor_temp_ = 0.0;
    inverter_temp_ = 0.0;
    fault_code_ = 0.0;
    dc_voltage_ = 0.0;

    current_status_word_ = 0;
    current_state_ = states.at(NRTSO_VAL);
    desired_state_ = states.at(RTSO_VAL);

    // Initialize ROS 2 Node for diagnostics and commands
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=qev_stepper_interface_node"});
    node_ = rclcpp::Node::make_shared("_", options);

    auto pub = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
    diagnostics_pub_ = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(pub);

    if (!socket_can_) {
        socket_can_ = std::make_unique<SocketCAN>();
    }

    RCLCPP_INFO(rclcpp::get_logger("QevStepperInterface"), "QEV Stepper Interface initialized. CAN: %s, Node ID: 0x%X",
                can_interface_name_.c_str(), node_id_);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QevStepperInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    if (!socket_can_->setup(can_interface_name_, rclcpp::get_logger("QevStepperInterface"))) {
        RCLCPP_ERROR(rclcpp::get_logger("QevStepperInterface"), "Failed to setup SocketCAN on %s",
                     can_interface_name_.c_str());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QevStepperInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "motor_temp", &motor_temp_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[0].name, "inverter_temp", &inverter_temp_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "fault_code", &fault_code_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "dc_voltage", &dc_voltage_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QevStepperInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_command_));
    return command_interfaces;
}

hardware_interface::CallbackReturn QevStepperInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    desired_state_ = states.at(OE_VAL);
    RCLCPP_INFO(rclcpp::get_logger("QevStepperInterface"), "QEV Stepper Interface activated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn QevStepperInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    desired_state_ = states.at(RTSO_VAL);
    RCLCPP_INFO(rclcpp::get_logger("QevStepperInterface"), "QEV Stepper Interface deactivated.");
    return CallbackReturn::SUCCESS;
}

void QevStepperInterface::sdo_write(uint16_t index, uint8_t sub_index, uint8_t *data, size_t data_size) {
    driverless_msgs::msg::Can msg;
    msg.id = 0x600 + node_id_;
    msg.id_type = false;
    msg.dlc = 8;
    msg.data.resize(8, 0x00);

    uint8_t free = static_cast<uint8_t>(4 - data_size);
    msg.data[0] = 0x23 | static_cast<uint8_t>((free << 2) & 0x1E);
    msg.data[1] = index & 0xFF;
    msg.data[2] = (index >> 8) & 0xFF;
    msg.data[3] = sub_index;
    for (size_t i = 0; i < data_size; i++) {
        msg.data[i + 4] = data[i];
    }

    socket_can_->tx(&msg, rclcpp::get_logger("QevStepperInterface"));
}

void QevStepperInterface::sdo_read(uint16_t index, uint8_t sub_index) {
    driverless_msgs::msg::Can msg;
    msg.id = 0x600 + node_id_;
    msg.id_type = false;
    msg.dlc = 8;
    msg.data.resize(8, 0x00);
    msg.data[0] = 0x40;
    msg.data[1] = index & 0xFF;
    msg.data[2] = (index >> 8) & 0xFF;
    msg.data[3] = sub_index;

    socket_can_->tx(&msg, rclcpp::get_logger("QevStepperInterface"));
}

void QevStepperInterface::configure_c5e() {
    this->sdo_write(PROFILE_VELOCITY, 0, (uint8_t *)&velocity_, 4);
    this->sdo_write(PROFILE_ACCELERATION, 0, (uint8_t *)&acceleration_, 4);
    this->sdo_write(PROFILE_DECELERATION, 0, (uint8_t *)&acceleration_, 4);
    this->sdo_write(QUICK_STOP_DECELERATION, 0, (uint8_t *)&acceleration_, 4);
    this->sdo_write(MAX_ACCELERATION, 0, (uint8_t *)&acceleration_, 4);
    this->sdo_write(MAX_DECELERATION, 0, (uint8_t *)&acceleration_, 4);
}

void QevStepperInterface::target_position(int32_t target) {
    if (current_state_.state_id != OE_VAL) {
        return;
    }

    this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&MODE_ABSOLUTE, 2);
    this->sdo_write(TARGET_POSITION, 0, (uint8_t *)&target, 4);

    uint16_t trigger_control_method = MODE_ABSOLUTE | TRIGGER_MOTION;
    this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&trigger_control_method, 2);
}

hardware_interface::return_type QevStepperInterface::read(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/) {
    // Request status word
    this->sdo_read(STATUS_WORD, 0);

    // Read all incoming frames
    auto frames = socket_can_->rx(rclcpp::get_logger("QevStepperInterface"), node_->get_clock());
    bool has_fault = false;
    std::string fault_reason = "";

    uint32_t bootup_id = 0x700 + node_id_;
    uint32_t emcy_id = 0x80 + node_id_;
    uint32_t pos_id = 0x280 + node_id_;
    uint32_t srv_id = 0x580 + node_id_;

    for (const auto &msg : *frames) {
        if (msg.id == VCU_TransmitSteering_ID) {
            int16_t steering0_raw = 0;
            int16_t steering1_raw = 0;
            uint16_t adc0 = 0;
            uint16_t adc1 = 0;
            Parse_VCU_TransmitSteering(msg.data.data(), &steering0_raw, &steering1_raw, &adc0, &adc1);
            double steering_deg = steering0_raw / 10.0;
            steering_ang_received_ = true;
            joint_position_state_ = steering_deg * (M_PI / 180.0);
        } else if (msg.id == emcy_id) {
            uint16_t error_code = static_cast<uint16_t>((msg.data[1] << 8) | msg.data[0]);
            fault_code_ = static_cast<double>(error_code);
            RCLCPP_ERROR(rclcpp::get_logger("QevStepperInterface"), "C5E Emergency code: 0x%X", error_code);
            has_fault = true;
            fault_reason = "Emergency Code 0x" + std::to_string(error_code);
        } else if (msg.id == bootup_id) {
            RCLCPP_INFO(rclcpp::get_logger("QevStepperInterface"), "C5E Booted Up");
        } else if (msg.id == pos_id) {
            uint32_t raw_pos = 0;
            for (int i = 0; i < 4; i++) {
                raw_pos |= static_cast<uint32_t>(msg.data[static_cast<size_t>(i)]) << (8 * i);
            }
            int32_t val = static_cast<int32_t>(raw_pos);
            current_position_ = val;
        } else if (msg.id == srv_id) {
            uint16_t object_id = static_cast<uint16_t>(((msg.data[2] & 0xFF) << 8) | (msg.data[1] & 0xFF));
            if (object_id == STATUS_WORD) {
                uint16_t status_word = static_cast<uint16_t>((msg.data[5] << 8) | msg.data[4]);
                current_status_word_ = status_word;
                current_state_ = parse_state(status_word);

                // Run state transitions
                if (current_state_.state_id == F_VAL) {
                    has_fault = true;
                    fault_reason = "Fault State";
                    if (fault_code_ == 0.0) {
                        fault_code_ = 1.0;
                    }
                    // Attempt auto-reset if OE is desired
                    if (desired_state_.state_id == OE_VAL) {
                        this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&FAULT_RESET, 2);
                    }
                } else {
                    fault_code_ = 0.0;
                    if (current_state_.state_id == RTSO_VAL && desired_state_.state_id == OE_VAL) {
                        uint16_t cw = states.at(SO_VAL).control_word;
                        this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&cw, 2);
                    } else if (current_state_.state_id == SO_VAL && desired_state_.state_id == OE_VAL) {
                        uint16_t cw = states.at(OE_VAL).control_word;
                        this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&cw, 2);
                        this->configure_c5e();
                    } else if (current_state_.state_id != desired_state_.state_id &&
                               desired_state_.state_id == RTSO_VAL) {
                        uint16_t cw = states.at(RTSO_VAL).control_word;
                        this->sdo_write(CONTROL_WORD, 0, (uint8_t *)&cw, 2);
                    }
                }
            }
        }
    }

    publish_diagnostics(has_fault, fault_reason);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type QevStepperInterface::write(const rclcpp::Time & /*time*/,
                                                           const rclcpp::Duration & /*period*/) {
    double command_ticks = std::isnan(joint_position_command_) ? 0.0 : joint_position_command_;
    int32_t target_ticks = static_cast<int32_t>(command_ticks);
    target_ticks = std::clamp(target_ticks, -static_cast<int32_t>(max_position_), static_cast<int32_t>(max_position_));
    this->target_position(target_ticks);
    return hardware_interface::return_type::OK;
}

void QevStepperInterface::publish_diagnostics(bool has_fault, const std::string &reason) {
    if (diagnostics_pub_ && diagnostics_pub_->trylock()) {
        auto &diag_msg = diagnostics_pub_->msg_;
        diag_msg.header.stamp = node_->now();
        diag_msg.status.clear();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Steering: Stepper Motor Controller";
        status.hardware_id = std::to_string(node_id_);

        if (has_fault) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Fault Active: " + reason;
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Operational state: " + current_state_.name;
        }

        status.values.clear();
        diagnostic_msgs::msg::KeyValue state_val;
        state_val.key = "Current State";
        state_val.value = current_state_.name;
        status.values.push_back(state_val);

        diagnostic_msgs::msg::KeyValue status_word_val;
        status_word_val.key = "Status Word";
        status_word_val.value = std::to_string(current_status_word_);
        status.values.push_back(status_word_val);

        diag_msg.status.push_back(status);
        diagnostics_pub_->unlockAndPublish();
    }
}

}  // namespace qutms_hw_interfaces

PLUGINLIB_EXPORT_CLASS(qutms_hw_interfaces::QevStepperInterface, hardware_interface::SystemInterface)
