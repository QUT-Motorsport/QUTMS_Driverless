#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can2etherenet_adapter.hpp"
#include "steering_control.hpp"
#include <algorithm>

using std::placeholders::_1;

class Steering : public rclcpp::Node
{
private:
std::shared_ptr<Can2Ethernet> c;
std::shared_ptr<SteeringControl> steering;

void topic_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const{
        float cappedAngle = std::fmax(std::fmin(msg->steering_angle, M_PI), -M_PI);
        int32_t steeringDemandStepper = cappedAngle * 5000/M_PI;

        RCLCPP_INFO(this->get_logger(), "Stepper: %i", steeringDemandStepper);
        RCLCPP_INFO(this->get_logger(), "Radians: %f", cappedAngle);
        this->steering->targetPosition(steeringDemandStepper);
        this->steering->reachedTarget();
    }
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;

public:
    Steering() : Node("steering") {
        this->subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("steering", 10, std::bind(&Steering::topic_callback, this, _1));
        this->c = std::make_shared<Can2Ethernet>("192.168.2.8", 20005);
        this->steering = std::make_shared<SteeringControl>(c);
	    this->steering->setAcceleration(std::make_pair<uint32_t, uint32_t>(10000, 10000));
    }
    
    ~Steering(){
        this->steering->shutdown();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Steering>());
    rclcpp::shutdown();
    return 0;
}
