#include "component_velocity_controller.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace velocity_controller {

VelocityController::VelocityController(const rclcpp::NodeOptions& options) : Node("velocity_controller_node", options) {
    // PID controller parameters
    this->declare_parameter<float>("Kp", 0.05);
    this->declare_parameter<float>("Ki", 0);
    this->declare_parameter<float>("max_integral_torque", 0);
    this->declare_parameter<float>("histerisis_kick_ms", 0);
    this->declare_parameter<float>("histerisis_reset_ms", 0);
    this->declare_parameter<float>("min_time_to_max_accel_sec", 2.0);

    this->update_parameters(rcl_interfaces::msg::ParameterEvent());

    if (Kp_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "Please provide parameters!");
    }

    // State updates (these could be in a msg filter)
    av_state_sub_ = this->create_subscription<driverless_msgs::msg::AVStateStamped>(
        "system/av_state", QOS_LATEST, std::bind(&VelocityController::av_state_callback, this, _1));
    ros_state_sub_ = this->create_subscription<driverless_msgs::msg::ROSStateStamped>(
        "system/ros_state", QOS_LATEST, std::bind(&VelocityController::ros_state_callback, this, _1));

    // Ackermann
    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/driving_command", QOS_ALL, std::bind(&VelocityController::ackermann_callback, this, _1));

    // imu updates
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "imu/velocity", QOS_LATEST, std::bind(&VelocityController::twist_callback, this, _1));

    // Control loop -> 10ms so runs at double speed heartbeats are sent at
    controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(loop_ms_),
                                                std::bind(&VelocityController::controller_callback, this));

    // Acceleration command publisher (to Supervisor so it can be sent in the DVL heartbeat)
    can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("can/canbus_carbound", 10);

    // Acceleration command publisher (to Supervisor so it can be sent in the DVL heartbeat)
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("vehicle/velocity", 10);

    // Param callback
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_cb_handle_ = param_event_handler_->add_parameter_event_callback(
        std::bind(&VelocityController::update_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "---Velocity Controller Node Initialised---");
}

void VelocityController::update_parameters(const rcl_interfaces::msg::ParameterEvent& event) {
    (void)event;

    this->get_parameter("Kp", Kp_);
    this->get_parameter("Ki", Ki_);
    this->get_parameter("max_integral_torque", max_integral_torque_);
    this->get_parameter("histerisis_kickin_ms", histerisis_kickin_ms_);
    this->get_parameter("histerisis_reset_ms", histerisis_reset_ms_);
    this->get_parameter("min_time_to_max_accel_sec", min_time_to_max_accel_sec_);
    max_accel_per_tick_ = loop_ms_ / (1000 * min_time_to_max_accel_sec_);

    RCLCPP_INFO(this->get_logger(), "Kp: %f, max_accel_per_tick: %f", Kp_, max_accel_per_tick_);
}

void VelocityController::av_state_callback(const driverless_msgs::msg::AVStateStamped::SharedPtr msg) {
    state_ = msg;
    // enabled if driving and not in inspection mission
    if (msg->state == driverless_msgs::msg::AVStateStamped::DRIVING && g2g_) {
        motors_enabled_ = true;
    }
}

void VelocityController::ros_state_callback(const driverless_msgs::msg::ROSStateStamped::SharedPtr msg) {
    g2g_ = msg->good_to_go;
}

void VelocityController::ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    target_ackermann_ = msg;
    received_ackermann_ = true;
}

void VelocityController::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // get magnitude of linear velocity
    avg_velocity_ = sqrt(pow(msg->twist.linear.x, 2) + pow(msg->twist.linear.y, 2));
    received_velocity_ = true;

    // publish velocity
    std_msgs::msg::Float32::UniquePtr velocity_msg(new std_msgs::msg::Float32());
    velocity_msg->data = avg_velocity_;
    velocity_pub_->publish(std::move(velocity_msg));
}

void VelocityController::controller_callback() {
    if (!motors_enabled_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Motors not enabled, awaiting State::DRIVING");
        return;
    }
    if (!received_velocity_ || !received_ackermann_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for target and current velocities");
        return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "Motors enabled, Received target and current velocities\n - Starting control loop");

    // calculate error
    float error = target_ackermann_->drive.speed - avg_velocity_;
    integral_error_ += error;

    // clip the integral error based on max_integral_torque_
    if (integral_error_ < 0) {
        integral_error_ = 0;
    } else if (integral_error_ > (max_integral_torque_ / Ki_)) {
        integral_error_ = max_integral_torque_ / Ki_;
    }

    if (avg_velocity_ < histerisis_reset_ms_) {
        integral_error_ = 0;
    }

    // calculate control variable
    float p_term = Kp_ * error;
    float i_term = Ki_ * integral_error_;

    float accel = p_term;
    if (avg_velocity_ > histerisis_kickin_ms_) {
        accel += i_term;
    }

    if ((accel - prev_accel_) > max_accel_per_tick_) {
        accel = prev_accel_ + max_accel_per_tick_;
    }

    // limit output accel to be between -1 (braking) and 1 (accel)
    if (accel > 1) {
        accel = 1;
    } else if (accel < -1) {
        accel = -1;
    }

    if (state_->mission == driverless_msgs::msg::AVStateStamped::INSPECTION) {
        accel = 0.11;  // THIS COULD BE A PARAM, TODO
    }

    // create control ackermann based off desired and calculated acceleration
    Torque_Request_t torque_request;
    torque_request.torque = accel * 100;  // convert to percentage
    auto torque_heartbeat = Compose_Torque_Request_Heartbeat(&torque_request);
    this->can_pub_->publish(
        std::move(this->_d_2_f(torque_heartbeat.id, true, torque_heartbeat.data, sizeof(torque_heartbeat.data))));

    prev_accel_ = accel;
}

}  // namespace velocity_controller

RCLCPP_COMPONENTS_REGISTER_NODE(velocity_controller::VelocityController)
