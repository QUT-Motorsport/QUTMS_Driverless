#include <iostream>

#include "CAN_DVL.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can_interface.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class TractiveSystemController : public rclcpp::Node, public CanInterface {
   private:
    // Private speed
    DVL_HeartbeatState_t state;

    // Can publisher and subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann;
    rclcpp::Subscription<driverless_msgs::msg::RES>::SharedPtr res;

    // Called when a new can message is recieved
    void canbus_callback(const driverless_msgs::msg::Can msg) {
        switch (msg.id) {
            default: {
            } break;
        }
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDrive msg) {
        // Update speed request in heartbeat
        this->state.torqueRequest = msg.acceleration;  // Chaos
    }

    void res_callback(const driverless_msgs::msg::RES msg) {
        // Update estop state from res
        // TODO: Change this to the state from the DVL state machine.

        this->state.flags._DVL_Flags.DVL_ESTOP = msg.estop;
    }

    void heartbeat_callback() {
        // Generate the DVL heartbeat
        auto heartbeat = Compose_DVL_Heartbeat(&this->state);

        // Publish the heartbeat
        // TODO: Should this be done in the DVL state machine?
        this->can_pub->publish(this->_d_2_f(heartbeat.id, true, heartbeat.data));
    }

    rclcpp::TimerBase::SharedPtr timer_;

   public:
    TractiveSystemController() : Node("tractive_system_controller") {
        // Configure logger level
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // Can
        RCLCPP_DEBUG(this->get_logger(), "Initalising can interactants...");
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&TractiveSystemController::canbus_callback, this, _1));

        this->res = this->create_subscription<driverless_msgs::msg::RES>(
            "res_status", 10, std::bind(&TractiveSystemController::res_callback, this, _1));

        if (this->can_pub == nullptr || this->can_sub == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create can topic interactants");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Done");
        }

        // Ackermann
        RCLCPP_DEBUG(this->get_logger(), "Initalising ackermann interactants...");
        this->ackermann = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "ackermann", 10, std::bind(&TractiveSystemController::ackermann_callback, this, _1));
        RCLCPP_DEBUG(this->get_logger(), "Done");

        // Heartbeat
        RCLCPP_DEBUG(this->get_logger(), "Creating heartbeat timer...");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                               std::bind(&TractiveSystemController::heartbeat_callback, this));
        RCLCPP_DEBUG(this->get_logger(), "Done");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TractiveSystemController>());
    rclcpp::shutdown();
    return 0;
}
