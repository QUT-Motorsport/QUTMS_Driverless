#include "component_steering_actuator.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace steering_actuator {

void SteeringActuator::update_parameters(const rcl_interfaces::msg::ParameterEvent &event) {
    (void)event;

    current_velocity_ = this->get_parameter("velocity").as_int();
    current_acceleration_ = this->get_parameter("acceleration").as_int();
    centre_range_ = this->get_parameter("centre_range").as_double();
    centre_angle_ = this->get_parameter("centre_angle").as_double();
    settling_iter_ = this->get_parameter("settling_iter").as_int();
    max_position_ = this->get_parameter("max_position").as_int();
    Kp_ = this->get_parameter("Kp").as_double();
    Ki_ = this->get_parameter("Ki").as_double();
    Kd_ = this->get_parameter("Kd").as_double();

    RCLCPP_INFO(this->get_logger(), "max_position %d", max_position_);
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
    std_msgs::msg::Bool::UniquePtr msg(new std_msgs::msg::Bool());
    msg->data = centred_;
    steering_ready_pub_->publish(std::move(msg));
}

// Check State to enable or disable motor
void SteeringActuator::as_state_callback(const driverless_msgs::msg::State::SharedPtr msg) {
    if (msg->state == driverless_msgs::msg::State::DRIVING || msg->state == driverless_msgs::msg::State::ACTIVATE_EBS) {
        // Enable motor
        motor_enabled_ = true;
    } else {
        // Disable motor
        motor_enabled_ = false;
        centred_ = false;
        settled_count_ = 0;
        initial_enc_saved_ = false;
        steering_ang_received_ = false;
    }
}

// Get steering angle reading
void SteeringActuator::steering_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_steering_angle_ = msg->data;
    if (!steering_ang_received_) steering_ang_received_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "Steering angle received");
    RCLCPP_DEBUG(this->get_logger(), "Current angle: %f", msg->data);
}

// Get desired steering angle to update steering
void SteeringActuator::driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    if (!motor_enabled_ || !centred_) return;

    double requested_steering_angle = msg->drive.steering_angle;
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Requested angle: %f",
    // msg->drive.steering_angle); turning left eqn: -83.95x - 398.92 turning right eqn: -96.19x - 83.79
    int32_t target;
    if (requested_steering_angle > centre_angle_) {
        target = int32_t(-86.45 * requested_steering_angle - 398.92) - offset_;
    } else {
        target = int32_t(-94.58 * requested_steering_angle - 83.79) - offset_;
    }
    target = std::max(std::min(target, max_position_ - offset_), -max_position_ - offset_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Target: %f = %d", requested_steering_angle,
                         target);
    this->target_position(target);
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
            RCLCPP_INFO_ONCE(this->get_logger(), "Position received");
            // initial_enc_ = val;
            initial_enc_saved_ = true;
        }

        std_msgs::msg::Int32::UniquePtr enc_msg(new std_msgs::msg::Int32());
        enc_msg->data = -current_enc_revolutions_;
        encoder_pub_->publish(std::move(enc_msg));
    } else if (msg->id == C5E_SRV_ID) {
        // objects returned from sdo read
        uint16_t object_id = (msg->data[2] & 0xFF) << 8 | (msg->data[1] & 0xFF);

        // if (msg->data[0] == 0x60 || msg->data[0] == 0x80) {
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

// Update Steering
void SteeringActuator::pre_op_centering() {
    if (!motor_enabled_ || !steering_ang_received_ || centred_ || current_state_ != states[OE])
        return;  // if multithread doesn't work

    RCLCPP_INFO_ONCE(this->get_logger(), "Centering");
    // center steering
    // splitting into threads means steering angle will be updated separately to this function
    if (settled_count_ < settling_iter_) {  // if multithread doesn't work
        double error = current_steering_angle_ - centre_angle_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error: %f", error);
        if (abs(error) < centre_range_) {
            // motor has settled enough
            settled_count_ += 1;
        } else {
            // reset settled count to debounce initial overshoots
            settled_count_ = 0;
        }
        // left hand down is +, rhd is -
        double target = Kp_ * error;  // P commands to send to plant
        this->target_position(target);
        return;  // if multithread doesn't work
    }
    offset_ = initial_enc_ - current_enc_revolutions_;
    centred_ = true;
    // save offset and configure c5e for different motion profile
    current_velocity_ = this->get_parameter("velocity").as_int();
    current_acceleration_ = this->get_parameter("acceleration").as_int();
    control_method_ = MODE_ABSOLUTE;
    RCLCPP_INFO_ONCE(this->get_logger(), "Centered, offset: %d", offset_);
    this->configure_c5e();
    return;
}

void SteeringActuator::target_position(int32_t target) {
    if (current_state_ != states[OE]) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Not enabled");
        return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "Enabled, Targeting");

    this->send_steering_data(CONTROL_WORD, (uint8_t *)&control_method_, 2);
    // print bitset control word
    // RCLCPP_INFO(this->get_logger(), "Control word: %s",
    // std::bitset<16>(control_method_).to_string().c_str());
    this->send_steering_data(TARGET_POSITION, (uint8_t *)&target, 4);
    uint16_t trigger_control_method = control_method_ | TRIGGER_MOTION;
    this->send_steering_data(CONTROL_WORD, (uint8_t *)&trigger_control_method, 2);
    // RCLCPP_INFO(this->get_logger(),"Trigger control word: %s",
    // std::bitset<16>(trigger_control_method).to_string().c_str());
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

SteeringActuator::SteeringActuator(const rclcpp::NodeOptions &options) : Node("steering_actuator_node", options) {
    // Steering parameters
    this->declare_parameter<int>("velocity", 10000);
    this->declare_parameter<int>("velocity_centering", 100);
    this->declare_parameter<int>("acceleration", 2000);
    this->declare_parameter<int>("acceleration_centering", 100);
    this->declare_parameter<float>("centre_range", 5.0);
    this->declare_parameter<float>("centre_angle", -2.0);
    this->declare_parameter<int>("settling_iter", 20);
    this->declare_parameter<int>("max_position", 7500);
    // PID controller parameters
    this->declare_parameter<double>("Kp", 1.0);
    this->declare_parameter<float>("Ki", 0.0);
    this->declare_parameter<float>("Kd", 0.0);

    sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sensor_cb_opt = rclcpp::SubscriptionOptions();
    sensor_cb_opt.callback_group = sensor_cb_group_;
    auto control_cb_opt = rclcpp::SubscriptionOptions();
    control_cb_opt.callback_group = control_cb_group_;

    // Create subscriber to topic "canbus_rosbound"
    canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "/can/canopen_rosbound", QOS_ALL, std::bind(&SteeringActuator::can_callback, this, _1), sensor_cb_opt);

    // Create subscriber to topic "steering_angle"
    steer_ang_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/vehicle/steering_angle", QOS_ALL, std::bind(&SteeringActuator::steering_angle_callback, this, _1),
        sensor_cb_opt);

    // Create subscriber to topic "driving_command"
    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1),
        control_cb_opt);

    // Create subscriber to topic AS status
    state_sub_ = this->create_subscription<driverless_msgs::msg::State>(
        "/system/as_status", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1), control_cb_opt);

    steering_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&SteeringActuator::pre_op_centering, this), sensor_cb_group_);

    // Create state request and config timers
    state_request_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&SteeringActuator::c5e_state_request_callback, this), control_cb_group_);

    // Create publisher to topic "canbus_carbound"
    can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", QOS_ALL);

    // Create publisher to topic "encoder_reading"
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle/encoder_reading", QOS_ALL);

    // Create publisher to topic "steering_state"
    steering_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system/steering_ready", QOS_ALL);

    // Param callback
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_cb_handle_ = param_event_handler_->add_parameter_event_callback(
        std::bind(&SteeringActuator::update_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "---Steering Actuator Node Initialised---");
}

}  // namespace steering_actuator

RCLCPP_COMPONENTS_REGISTER_NODE(steering_actuator::SteeringActuator);
