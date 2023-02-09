#include <iostream>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_msgs/msg/motor_rpm.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Velocity_Controller : public rclcpp::Node {
   private:
    float Kp_vel = 0;
    float Ki_vel = 0;
    float max_integral_torque = 0;
    float histerisis_kickin_ms = 0;
    float histerisis_reset_ms = 0;

    float integral_error = 0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr accel_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<driverless_msgs::msg::MotorRPM>::SharedPtr motorRPM_sub;

    ackermann_msgs::msg::AckermannDriveStamped target_ackermann;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_cb_handle;

    // velocity of each wheel in m/s
    float motor_velocities[4];

    const int MOTOR_COUNT = 4;
    const float WHEEL_RADIUS = 0.4064;

    rclcpp::TimerBase::SharedPtr controller_timer;

    rclcpp::Time _internal_status_time;

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) { this->target_ackermann = msg; }

    void rpm_callback(const driverless_msgs::msg::MotorRPM msg) {
        if (msg.index < MOTOR_COUNT) {
            // valid wheel, so convert from rpm to m/s

            // convert from eRPM
            float motorRPM = msg.rpm / (21.0 * 4.50);

            // convert to m/s
            motor_velocities[msg.index] = motorRPM * M_PI * this->WHEEL_RADIUS / 60;
        }
    }

    void controller_callback() {
        // PID controller for velocity here

        // average all 4 motor velocities to use as 'current'
        float av_velocity = 0;

        for (int i = 0; i < MOTOR_COUNT; i++) {
            av_velocity += this->motor_velocities[i];
        }

        av_velocity = av_velocity / MOTOR_COUNT;

        // calculate error
        float error = this->target_ackermann.drive.speed - av_velocity;
        this->integral_error += error;

        // clip the integral error based on max_integral_torque
        if (this->integral_error < 0) {
            this->integral_error = 0;
        } else if (this->integral_error > (this->max_integral_torque / this->Ki_vel)) {
            this->integral_error = this->max_integral_torque / this->Ki_vel;
        }

        if (av_velocity < this->histerisis_reset_ms) {
            this->integral_error = 0;
        }

        // calculate control variable
        float p_term = this->Kp_vel * error;
        float i_term = this->Ki_vel * integral_error;

        float accel = p_term;
        if (av_velocity > this->histerisis_kickin_ms) {
            accel += i_term;
        }

        // limit output accel to be between -1 (braking) and 1 (accel)
        if (accel > 1) {
            accel = 1;
        } else if (accel < -1) {
            accel = -1;
        }

        // create control ackermann based off desired and calculated acceleration
        ackermann_msgs::msg::AckermannDriveStamped accel_cmd;
        accel_cmd.header.stamp = this->now();
        accel_cmd.drive = this->target_ackermann.drive;
        accel_cmd.drive.acceleration = accel;

        // publish accel
        this->accel_pub->publish(accel_cmd);
    }

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event) {
        (void)event;

        this->get_parameter("Kp_vel", this->Kp_vel);
        this->get_parameter("Ki_vel", this->Ki_vel);
        this->get_parameter("max_integral_torque", this->max_integral_torque);
        this->get_parameter("histerisis_kickin_ms", this->histerisis_kickin_ms);
        this->get_parameter("histerisis_reset_ms", this->histerisis_reset_ms);

        RCLCPP_INFO(this->get_logger(),
                    "Kp: %f Ki: %f max_integral_torque: %f histerisis_kickin_ms: %f histerisis_reset_ms: %f", Kp_vel,
                    Ki_vel, max_integral_torque, histerisis_kickin_ms, histerisis_reset_ms);
    }

   public:
    Velocity_Controller() : Node("velocity_controller_node") {
        // PID controller parameters
        this->declare_parameter<float>("Kp_vel", 0);
        this->declare_parameter<float>("Ki_vel", 0);
        this->declare_parameter<float>("max_integral_torque", 0);
        this->declare_parameter<float>("histerisis_kick_ms", 0);
        this->declare_parameter<float>("histerisis_reset_ms", 0);

        this->update_parameters(rcl_interfaces::msg::ParameterEvent());

        // Configure logger level
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // Ackermann
        this->ackermann_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/driving_command", 10, std::bind(&Velocity_Controller::ackermann_callback, this, _1));

        // Velocity updates
        this->motorRPM_sub = this->create_subscription<driverless_msgs::msg::MotorRPM>(
            "/vehicle/motor_rpm", 10, std::bind(&Velocity_Controller::rpm_callback, this, _1));

        // Control loop -> 10ms so runs at double speed heartbeats are sent at
        this->controller_timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                                         std::bind(&Velocity_Controller::controller_callback, this));

        // Acceleration command publisher
        this->accel_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/accel_command", 10);

        // Param callback

        this->param_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);
        this->param_cb_handle = this->param_event_handler->add_parameter_event_callback(
            std::bind(&Velocity_Controller::update_parameters, this, std::placeholders::_1));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Velocity_Controller>());
    rclcpp::shutdown();
    return 0;
}
