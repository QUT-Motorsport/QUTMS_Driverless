#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class YawController : public rclcpp::Node {
   private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr shutdown_sub;

    rclcpp::TimerBase::SharedPtr yaw_timer;
    rclcpp::TimerBase::SharedPtr yaw_rate_timer;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driving_command_pub;

    float target_yaw = 0;
    float current_yaw = 0;
    float integral_yaw = 0;

    float target_yaw_rate = 0;
    float current_yaw_rate = 0;
    float integral_yaw_rate = 0;

    float Kp_yaw = 0;
    float Ki_yaw = 0;
    float Kp_yaw_rate = 0;
    float Ki_yaw_rate = 0;
    float max_integral_steering = 0;
    float target_velocity = 0;
    float integral_kickin = 0;

    void shutdown_callback(const driverless_msgs::msg::State msg) {
        if (msg.state == driverless_msgs::msg::State::ACTIVATE_EBS ||
            msg.state == driverless_msgs::msg::State::FINISHED || msg.state == driverless_msgs::msg::State::EMERGENCY) {
            rclcpp::shutdown();
        }
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped msg) { current_yaw_rate = msg.twist.angular.z; }

    void imu_callback(const sensor_msgs::msg::Imu msg) {
        tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw = yaw;
    }

    void yaw_callback() {
        // pass
    }

    void yaw_rate_callback() {
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

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event) {
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

   public:
    YawController() : Node("yaw_controller") {
        this->declare_parameter<float>("target_velocity", 0);
        this->declare_parameter<float>("Kp_yaw", 0);
        this->declare_parameter<float>("Ki_yaw", 0);
        this->declare_parameter<float>("Kp_yaw_rate", 0);
        this->declare_parameter<float>("Ki_yaw_rate", 0);
        this->declare_parameter<float>("max_integral_steering", 0);
        this->declare_parameter<float>("integral_kickin", 0);

        this->update_parameters(rcl_interfaces::msg::ParameterEvent());

        this->shutdown_sub = this->create_subscription<driverless_msgs::msg::State>(
            "as_status", 10, std::bind(&YawController::shutdown_callback, this, _1));
        this->velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "imu/velocity", 10, std::bind(&YawController::velocity_callback, this, _1));
        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&YawController::imu_callback, this, _1));
        this->yaw_timer =
            this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&YawController::yaw_callback, this));
        this->yaw_rate_timer =
            this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&YawController::yaw_rate_callback, this));

        this->driving_command_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("driving_command", 10);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawController>());
    rclcpp::shutdown();
    return 0;
}
