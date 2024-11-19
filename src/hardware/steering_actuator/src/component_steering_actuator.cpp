#include "component_steering_actuator.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace steering_actuator {

SteeringActuator::SteeringActuator(const rclcpp::NodeOptions &options) : Node("steering_actuator_node", options) {
    // Steering parameters
    this->declare_parameter<int>("velocity", 10000);
    this->declare_parameter<int>("acceleration", 2000);
    this->declare_parameter<int>("max_position", 7500);

    this->update_parameters(rcl_interfaces::msg::ParameterEvent());

    control_method_ = MODE_ABSOLUTE;

    sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sensor_cb_opt = rclcpp::SubscriptionOptions();
    sensor_cb_opt.callback_group = sensor_cb_group_;
    auto control_cb_opt = rclcpp::SubscriptionOptions();
    control_cb_opt.callback_group = control_cb_group_;

    // Create subscriber to topic "canbus_rosbound"
    canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "can/canopen_rosbound", QOS_ALL, std::bind(&SteeringActuator::can_callback, this, _1), sensor_cb_opt);

    // Create subscriber to topic "steering_angle"
    steer_ang_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "vehicle/steering_angle", QOS_ALL, std::bind(&SteeringActuator::steering_angle_callback, this, _1),
        sensor_cb_opt);

    // Create subscriber to topic "driving_command"
    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1),
        control_cb_opt);

    // Create subscriber to topic AS status
    state_sub_ = this->create_subscription<driverless_msgs::msg::AVStateStamped>(
        "system/av_state", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1), control_cb_opt);

    // Create state request and config timers
    state_request_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&SteeringActuator::c5e_state_request_callback, this), control_cb_group_);

    // Create publisher to topic "canbus_carbound"
    can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("can/canbus_carbound", QOS_ALL);

    // Create publisher to topic "encoder_reading"
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32>("vehicle/encoder_reading", QOS_ALL);

    // Create publisher to topic "encoder_reading"
    step_target_pub_ = this->create_publisher<std_msgs::msg::Int32>("vehicle/stepper_request", QOS_ALL);

    // Param callback
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_cb_handle_ = param_event_handler_->add_parameter_event_callback(
        std::bind(&SteeringActuator::update_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "---Steering Actuator Node Initialised---");
}

void SteeringActuator::update_parameters(const rcl_interfaces::msg::ParameterEvent &event) {
    (void)event;

    current_velocity_ = this->get_parameter("velocity").as_int();
    current_acceleration_ = this->get_parameter("acceleration").as_int();
    max_position_ = this->get_parameter("max_position").as_int();
}

void SteeringActuator::configure_c5e() {
    this->send_steering_data(PROFILE_VELOCITY, (uint8_t *)&current_velocity_, 4);
    this->send_steering_data(PROFILE_ACCELERATION, (uint8_t *)&current_acceleration_, 4);
    this->send_steering_data(PROFILE_DECELERATION, (uint8_t *)&current_acceleration_, 4);
    this->send_steering_data(QUICK_STOP_DECELERATION, (uint8_t *)&current_acceleration_, 4);
    this->send_steering_data(MAX_ACCELERATION, (uint8_t *)&current_acceleration_, 4);
    this->send_steering_data(MAX_DECELERATION, (uint8_t *)&current_acceleration_, 4);
    // this->send_steering_data(HOME_OFFSET, (uint8_t *)&offset_, 4);

    RCLCPP_INFO(this->get_logger(), "Configured C5E");
}

void SteeringActuator::c5e_state_request_callback() {
    this->read_steering_data(STATUS_WORD);
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Requesting state");
}

// Check State to enable or disable motor
void SteeringActuator::as_state_callback(const driverless_msgs::msg::AVStateStamped::SharedPtr msg) {
    if (msg->state == driverless_msgs::msg::AVStateStamped::DRIVING && g2g_) {
        // Enable motor
        motor_enabled_ = true;
    } else {
        // Disable motor
        motor_enabled_ = false;
    }
}

void SteeringActuator::ros_state_callback(const driverless_msgs::msg::ROSStateStamped::SharedPtr msg) {
    g2g_ = msg->good_to_go;
}

// Get steering angle reading
void SteeringActuator::steering_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!steering_ang_received_ && initial_enc_saved_) {
        steering_ang_received_ = true;
        offset_ = int32_t(-105 * msg->data) - initial_enc_;
        RCLCPP_INFO(this->get_logger(), "Steering angle received, offset: %d", offset_);
    }

    RCLCPP_DEBUG(this->get_logger(), "Current angle: %f", msg->data);
}

// Get desired steering angle to update steering
void SteeringActuator::driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    if (!motor_enabled_ && !steering_ang_received_) return;
    // Use 1 equation for both left and right turns
    int32_t target = int32_t(-105 * msg->drive.steering_angle) - offset_;
    // Clamp target to max
    target = std::max(std::min(target, max_position_ - offset_), -max_position_ - offset_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Target: %f = %d", msg->drive.steering_angle,
                         target);
    this->target_position(target);

    std_msgs::msg::Int32::UniquePtr step_targ_msg(new std_msgs::msg::Int32());
    step_targ_msg->data = -target;  // Flip to match steering angle signs, remove offset
    step_target_pub_->publish(std::move(step_targ_msg));
}

// Receive message from CAN
void SteeringActuator::can_callback(const driverless_msgs::msg::Can::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "CAN ID: %x", msg->id);
    // Message ID from the steering actuator
    if (msg->id == C5E_EMCY_ID) {
        // first two bytes are the error code
        uint16_t error_code = (msg->data[1] << 8 | msg->data[0]);
        RCLCPP_ERROR(this->get_logger(), "Emergency error code: %x", error_code);

    } else if (msg->id == C5E_BOOT_UP_ID) {
        RCLCPP_INFO(this->get_logger(), "C5E Booted Up");

    } else if (msg->id == C5E_POS_ID) {
        // first 4 bytes are the position (int32)
        uint32_t data = 0;
        // iterate to get the 4 bytes
        for (int i = 0; i < 4; i++) {
            data |= msg->data[i] << (8 * i);
        }

        // message received for position revolution
        int32_t val = (int32_t)data;
        current_enc_revolutions_ = val;
        if (!initial_enc_saved_) {
            RCLCPP_INFO(this->get_logger(), "Position received: %d", val);
            initial_enc_ = val;
            initial_enc_saved_ = true;
        }

        std_msgs::msg::Int32::UniquePtr enc_msg(new std_msgs::msg::Int32());
        enc_msg->data = -current_enc_revolutions_;
        encoder_pub_->publish(std::move(enc_msg));

    } else if (msg->id == C5E_SRV_ID) {
        // objects returned from sdo read
        uint16_t object_id = (msg->data[2] & 0xFF) << 8 | (msg->data[1] & 0xFF);

        if (msg->data[0] == 0x80) {
            // acknowledgements of sdo write: 0x60 -> success ack, 0x80 -> error ack
            RCLCPP_INFO(this->get_logger(), "ACK object: %x, %x", object_id, msg->data[0]);
            return;
        }

        if (object_id == STATUS_WORD) {
            uint16_t status_word = (msg->data[5] << 8 | msg->data[4]);
            current_state_ = parse_state(status_word);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Status word: %s",
                                  std::bitset<16>(status_word).to_string().c_str());
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Current state: %s",
                                  current_state_.name.c_str());

            if (motor_enabled_) {
                if (current_state_ == states[RTSO]) {
                    desired_state_ = states[SO];
                } else if (current_state_ == states[SO]) {
                    desired_state_ = states[OE];
                    // default params
                    this->configure_c5e();
                } else if (current_state_ == states[OE]) {
                    // stay here -> no transition
                } else {
                    desired_state_ = states[RTSO];
                }
            } else {
                // disabled transitions
                desired_state_ = states[RTSO];
            }

            // Print current and desired states
            RCLCPP_DEBUG(this->get_logger(), "Desired state: %s", desired_state_.name.c_str());

            // State transition stage via state map definitions
            if (current_state_ != desired_state_) {
                RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.name.c_str());
                RCLCPP_INFO(this->get_logger(), "Sending state: %s", desired_state_.name.c_str());
                this->send_steering_data(CONTROL_WORD, (uint8_t *)&desired_state_.control_word, 2);
            }
        }
    }
}

void SteeringActuator::target_position(int32_t target) {
    if (current_state_ != states[OE]) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Not enabled");
        return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "Enabled, Targeting");

    this->send_steering_data(CONTROL_WORD, (uint8_t *)&control_method_, 2);
    this->send_steering_data(TARGET_POSITION, (uint8_t *)&target, 4);
    uint16_t trigger_control_method = control_method_ | TRIGGER_MOTION;
    this->send_steering_data(CONTROL_WORD, (uint8_t *)&trigger_control_method, 2);
}

void SteeringActuator::send_steering_data(uint16_t obj_index, uint8_t *data, size_t data_size) {
    uint32_t id;                                                                    // Packet id out
    uint8_t out[8];                                                                 // Data out
    sdo_write(C5E_NODE_ID, obj_index, 0x00, (uint8_t *)data, data_size, &id, out);  // Control Word
    can_pub_->publish(std::move(_d_2_f(id, 0, out, sizeof(out))));
}

void SteeringActuator::read_steering_data(uint16_t obj_index) {
    uint32_t id;                                       // Packet id out
    uint8_t out[8];                                    // Data out
    sdo_read(C5E_NODE_ID, obj_index, 0x00, &id, out);  // Control Word
    can_pub_->publish(std::move(_d_2_f(id, 0, out, sizeof(out))));
}
}  // namespace steering_actuator

RCLCPP_COMPONENTS_REGISTER_NODE(steering_actuator::SteeringActuator);
