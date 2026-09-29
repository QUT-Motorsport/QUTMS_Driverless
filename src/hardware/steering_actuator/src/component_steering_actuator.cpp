#include "component_steering_actuator.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace steering_actuator {

SteeringActuator::SteeringActuator(const rclcpp::NodeOptions &options) : Node("steering_actuator_node", options) {
    // Motor parameters
    this->declare_parameter<int>("motor_id", 1);
    this->declare_parameter<double>("speed_rpm", 3276.7);
    this->declare_parameter<double>("current_limit_a", 3.0);
    this->declare_parameter<double>("scale", 1.0);
    this->declare_parameter<double>("steer_offset_deg", 0.0);
    this->declare_parameter<bool>("invert", false);
    this->declare_parameter<double>("max_position", 100.0);
    this->declare_parameter<double>("command_timeout_s", 0.5);
    this->declare_parameter<bool>("require_driving", true);
    this->declare_parameter<double>("rate_hz", 50.0);

    this->update_parameters(rcl_interfaces::msg::ParameterEvent());
    last_cmd_time_ = this->now();

    sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sensor_cb_opt = rclcpp::SubscriptionOptions();
    sensor_cb_opt.callback_group = sensor_cb_group_;
    auto control_cb_opt = rclcpp::SubscriptionOptions();
    control_cb_opt.callback_group = control_cb_group_;

    motor_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "can/steering_rosbound", QOS_ALL, std::bind(&SteeringActuator::motor_callback, this, _1), sensor_cb_opt);

    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1),
        control_cb_opt);

    as_state_sub_ = this->create_subscription<driverless_msgs::msg::AVStateStamped>(
        "system/av_state", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1), control_cb_opt);
    ros_state_sub_ = this->create_subscription<driverless_msgs::msg::ROSStateStamped>(
        "system/ros_state", QOS_ALL, std::bind(&SteeringActuator::ros_state_callback, this, _1), control_cb_opt);

    double rate_hz = this->get_parameter("rate_hz").as_double();
    send_timer_ = this->create_wall_timer(std::chrono::microseconds((int64_t)(1e6 / rate_hz)),
                                          std::bind(&SteeringActuator::send_timer_callback, this), control_cb_group_);

    can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("can/canbus_carbound", QOS_ALL);
    target_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering_motor/target", QOS_ALL);
    position_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering_motor/position", QOS_ALL);
    current_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering_motor/current", QOS_ALL);
    temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/steering_motor/temperature", QOS_ALL);
    error_pub_ = this->create_publisher<std_msgs::msg::Int32>("vehicle/steering_motor/error", QOS_ALL);

    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_cb_handle_ = param_event_handler_->add_parameter_event_callback(
        std::bind(&SteeringActuator::update_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "---Steering Actuator Node Initialised--- ENCOS id=%d speed=%.1f rpm current=%.1f A "
                "max=%.1f deg require_driving=%d",
                motor_id_, speed_rpm_, current_limit_a_, max_position_, require_driving_);
}

SteeringActuator::~SteeringActuator() {
    if (sending_) {
        this->send_frame(encos::STOP_FRAME, sizeof(encos::STOP_FRAME));
    }
}

void SteeringActuator::update_parameters(const rcl_interfaces::msg::ParameterEvent &event) {
    (void)event;
    std::lock_guard<std::mutex> lock(mutex_);
    motor_id_ = this->get_parameter("motor_id").as_int();
    speed_rpm_ = this->get_parameter("speed_rpm").as_double();
    current_limit_a_ = this->get_parameter("current_limit_a").as_double();
    scale_ = this->get_parameter("scale").as_double();
    steer_offset_deg_ = this->get_parameter("steer_offset_deg").as_double();
    invert_ = this->get_parameter("invert").as_bool();
    max_position_ = this->get_parameter("max_position").as_double();
    command_timeout_s_ = this->get_parameter("command_timeout_s").as_double();
    require_driving_ = this->get_parameter("require_driving").as_bool();
}

void SteeringActuator::as_state_callback(const driverless_msgs::msg::AVStateStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    av_driving_ = msg->state == driverless_msgs::msg::AVStateStamped::DRIVING;
}

void SteeringActuator::ros_state_callback(const driverless_msgs::msg::ROSStateStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    g2g_ = msg->good_to_go;
}

void SteeringActuator::driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    float target;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        double sign = invert_ ? -1.0 : 1.0;
        target = sign * scale_ * (msg->drive.steering_angle + steer_offset_deg_);
        target = std::clamp<float>(target, -max_position_, max_position_);
        target_deg_ = target;
        have_target_ = true;
        last_cmd_time_ = this->now();
    }
    RCLCPP_DEBUG(this->get_logger(), "Target: %f = %f deg", msg->drive.steering_angle, target);

    std_msgs::msg::Float32::UniquePtr target_msg(new std_msgs::msg::Float32());
    target_msg->data = target;
    target_pub_->publish(std::move(target_msg));
}

void SteeringActuator::send_timer_callback() {
    bool go;
    float target, speed, current;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool fresh = have_target_ && (this->now() - last_cmd_time_).seconds() < command_timeout_s_;
        bool allowed = !require_driving_ || (av_driving_ && g2g_);
        go = fresh && allowed;
        target = target_deg_;
        speed = speed_rpm_;
        current = current_limit_a_;
    }

    if (go) {
        if (!sending_) RCLCPP_INFO(this->get_logger(), "Steering enabled");
        sending_ = true;
        uint8_t out[8];
        encos::pack_servo_position(target, speed, current, 2, out);
        this->send_frame(out, sizeof(out));
    } else if (sending_) {
        sending_ = false;
        this->send_frame(encos::STOP_FRAME, sizeof(encos::STOP_FRAME));
        RCLCPP_INFO(this->get_logger(), "Steering disabled (no fresh command)");
    }
}

void SteeringActuator::motor_callback(const driverless_msgs::msg::Can::SharedPtr msg) {
    encos::Feedback fb;
    if (!encos::decode_reply(msg->data.data(), msg->data.size(), fb)) return;

    if (fb.error != encos::ERROR_NONE) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Motor error: %s",
                              encos::error_text(fb.error).c_str());
    }
    std_msgs::msg::Int32::UniquePtr err_msg(new std_msgs::msg::Int32());
    err_msg->data = fb.error;
    error_pub_->publish(std::move(err_msg));

    if (!fb.has_position) return;
    std_msgs::msg::Float32::UniquePtr pos_msg(new std_msgs::msg::Float32());
    pos_msg->data = fb.position_deg;
    position_pub_->publish(std::move(pos_msg));
    std_msgs::msg::Float32::UniquePtr cur_msg(new std_msgs::msg::Float32());
    cur_msg->data = fb.current_a;
    current_pub_->publish(std::move(cur_msg));
    std_msgs::msg::Float32::UniquePtr temp_msg(new std_msgs::msg::Float32());
    temp_msg->data = fb.temperature_c;
    temperature_pub_->publish(std::move(temp_msg));
}

void SteeringActuator::send_frame(const uint8_t *data, uint8_t dlc) {
    uint8_t out[8] = {0};
    memcpy(out, data, dlc);
    can_pub_->publish(this->_d_2_f(motor_id_, false, out, dlc));
}

}  // namespace steering_actuator

RCLCPP_COMPONENTS_REGISTER_NODE(steering_actuator::SteeringActuator);
