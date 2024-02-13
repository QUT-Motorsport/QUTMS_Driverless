#include "node_yaw_controller.hpp"

void YawController::shutdown_callback(const driverless_msgs::msg::State msg) {
    if (msg.state == driverless_msgs::msg::State::START || msg.state == driverless_msgs::msg::State::SELECT_MISSION ||
        msg.state == driverless_msgs::msg::State::ACTIVATE_EBS || msg.state == driverless_msgs::msg::State::FINISHED ||
        msg.state == driverless_msgs::msg::State::EMERGENCY) {
        rclcpp::shutdown();
    }
}

void YawController::imu_callback(const sensor_msgs::msg::Imu msg) {
    tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw = yaw;
}

void YawController::yaw_rate_callback() {
    float error = target_yaw_rate - current_yaw_rate;
    integral_yaw_rate += error;

    if (abs(error) < integral_kickin) {
        integral_yaw_rate = 0;
    }

    // clip the integral error based on max_integral_steering
    float max_integral_error = max_integral_steering / Ki_yaw_rate;
    if (integral_yaw_rate > max_integral_error) {
        integral_yaw_rate = max_integral_error;
    } else if (integral_yaw_rate < -max_integral_error) {
        integral_yaw_rate = -max_integral_error;
    }

    RCLCPP_INFO(this->get_logger(), "integral_yaw_rate: %f", integral_yaw_rate);

    // calculate control variable
    float p_term = Kp_yaw_rate * error;
    float i_term = Ki_yaw_rate * integral_yaw_rate;

    ackermann_msgs::msg::AckermannDriveStamped driving_command;
    driving_command.header.stamp = now();
    driving_command.drive.speed = target_velocity;
    driving_command.drive.steering_angle = p_term + i_term;

    // publish accel
    this->driving_command_pub->publish(driving_command);
}

void YawController::update_parameters(const rcl_interfaces::msg::ParameterEvent& event) {
    (void)event;

    this->get_parameter("target_velocity", this->target_velocity);
    this->get_parameter("Kp_yaw", this->Kp_yaw);
    this->get_parameter("Ki_yaw", this->Ki_yaw);
    this->get_parameter("Kp_yaw_rate", this->Kp_yaw_rate);
    this->get_parameter("Ki_yaw_rate", this->Ki_yaw_rate);
    this->get_parameter("max_integral_steering", this->max_integral_steering);
    this->get_parameter("integral_kickin", this->integral_kickin);

    RCLCPP_INFO(this->get_logger(),
                "target_velocity: %f Kp_yaw: %f Ki_yaw: %f Kp_yaw_rate: %f Ki_yaw_rate: %f max_integral_steering: "
                "%f integral_kickin: %f",
                target_velocity, Kp_yaw, Ki_yaw, Kp_yaw_rate, Ki_yaw_rate, max_integral_steering, integral_kickin);
}

YawController::YawController() : Node("yaw_controller_node") {
    this->declare_parameter<float>("target_velocity", 0);
    this->declare_parameter<float>("Kp_yaw", 0);
    this->declare_parameter<float>("Ki_yaw", 0);
    this->declare_parameter<float>("Kp_yaw_rate", 0);
    this->declare_parameter<float>("Ki_yaw_rate", 0);
    this->declare_parameter<float>("max_integral_steering", 0);
    this->declare_parameter<float>("integral_kickin", 0);

    this->update_parameters(rcl_interfaces::msg::ParameterEvent());

    this->shutdown_sub = this->create_subscription<driverless_msgs::msg::State>(
        "/system/as_status", QOS_LATEST, std::bind(&YawController::shutdown_callback, this, _1));
    this->velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "imu/velocity", QOS_LATEST, std::bind(&YawController::velocity_callback, this, _1));
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", QOS_LATEST,
                                                                     std::bind(&YawController::imu_callback, this, _1));
    this->yaw_timer =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&YawController::yaw_callback, this));
    this->yaw_rate_timer =
        this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&YawController::yaw_rate_callback, this));

    this->driving_command_pub =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawController>());
    rclcpp::shutdown();
    return 0;
}
