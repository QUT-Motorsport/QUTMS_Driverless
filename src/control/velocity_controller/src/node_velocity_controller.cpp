#include <iostream>

#include "driverless_common/common.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Velocity_Controller : public rclcpp::Node {
   private:
    float Kp = 0;
    float Ki = 0;
    float max_integral_torque = 0;
    float histerisis_kickin_ms = 0;
    float histerisis_reset_ms = 0;

    float integral_error = 0;

    // wheel params
    const int MOTOR_COUNT = 4;
    const float WHEEL_RADIUS = 0.4064;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr accel_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub;
    rclcpp::TimerBase::SharedPtr controller_timer;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_cb_handle;

    // Enable motors logic
    driverless_msgs::msg::State state;
    bool motors_enabled = false;
    bool received_velocity = false;
    bool received_ackermann = false;

    float avg_velocity;
    ackermann_msgs::msg::AckermannDriveStamped target_ackermann;

   public:
    Velocity_Controller() : Node("velocity_controller_node") {
        // PID controller parameters
        this->declare_parameter<float>("Kp", 1.0);
        this->declare_parameter<float>("Ki", 0);
        this->declare_parameter<float>("max_integral_torque", 0);
        this->declare_parameter<float>("histerisis_kick_ms", 0);
        this->declare_parameter<float>("histerisis_reset_ms", 0);

        this->update_parameters(rcl_interfaces::msg::ParameterEvent());

        if (this->Kp == 0) {
            RCLCPP_ERROR(this->get_logger(), "Please provide parameters!");
        }

        // State updates
        this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
            "/system/as_status", QOS_LATEST, std::bind(&Velocity_Controller::state_callback, this, _1));

        // Ackermann
        this->ackermann_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/driving_command", QOS_ALL, std::bind(&Velocity_Controller::ackermann_callback, this, _1));

        // Velocity updates
        this->velocity_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/velocity", QOS_LATEST, std::bind(&Velocity_Controller::velocity_callback, this, _1));

        // Control loop -> 10ms so runs at double speed heartbeats are sent at
        this->controller_timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                                         std::bind(&Velocity_Controller::controller_callback, this));

        // Acceleration command publisher (to Supervisor so it can be sent in the DVL heartbeat)
        this->accel_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/accel_command", 10);

        // Param callback
        this->param_event_handler = std::make_shared<rclcpp::ParameterEventHandler>(this);
        this->param_cb_handle = this->param_event_handler->add_parameter_event_callback(
            std::bind(&Velocity_Controller::update_parameters, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "---Velocity Controller Node Initialised---");
    }

    void update_parameters(const rcl_interfaces::msg::ParameterEvent& event) {
        (void)event;

        this->get_parameter("Kp", this->Kp);
        this->get_parameter("Ki", this->Ki);
        this->get_parameter("max_integral_torque", this->max_integral_torque);
        this->get_parameter("histerisis_kickin_ms", this->histerisis_kickin_ms);
        this->get_parameter("histerisis_reset_ms", this->histerisis_reset_ms);

        RCLCPP_DEBUG(this->get_logger(),
                     "Kp: %f Ki: %f max_integral_torque: %f histerisis_kickin_ms: %f histerisis_reset_ms: %f", this->Kp,
                     this->Ki, this->max_integral_torque, this->histerisis_kickin_ms, this->histerisis_reset_ms);
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) { 
        this->target_ackermann = msg; 
        this->received_ackermann = true;
    }

    void velocity_callback(const std_msgs::msg::Float32 msg) { 
        this->avg_velocity = msg.data; 
        this->received_velocity = true;
    }

    void state_callback(const driverless_msgs::msg::State msg) {
        this->state = msg;
        if (msg.state == driverless_msgs::msg::State::DRIVING) {
            this->motors_enabled = true;
        } else {
            this->motors_enabled = false;
        }
    }

    void controller_callback() {
        if (!this->motors_enabled) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Motors not enabled, awaiting State::DRIVING");
            return;
        }
        if (!this->received_velocity || !this->received_ackermann ) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for target and current velocities");
            return;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Motors enabled, Received target and current velocities\n - Starting control loop");

        // calculate error
        float error = this->target_ackermann.drive.speed - this->avg_velocity;
        this->integral_error += error;

        // clip the integral error based on max_integral_torque
        if (this->integral_error < 0) {
            this->integral_error = 0;
        } else if (this->integral_error > (this->max_integral_torque / this->Ki)) {
            this->integral_error = this->max_integral_torque / this->Ki;
        }

        if (this->avg_velocity < this->histerisis_reset_ms) {
            this->integral_error = 0;
        }

        // calculate control variable
        float p_term = this->Kp * error;
        float i_term = this->Ki * this->integral_error;

        float accel = p_term;
        if (this->avg_velocity > this->histerisis_kickin_ms) {
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
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Velocity_Controller>());
    rclcpp::shutdown();
    return 0;
}
