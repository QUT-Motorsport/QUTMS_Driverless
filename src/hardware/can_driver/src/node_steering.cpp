#include <algorithm>
#include <memory>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can2etherenet_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "steering_control.hpp"

using std::placeholders::_1;

class Steering : public rclcpp::Node {
   private:
	std::shared_ptr<Can2Ethernet> c;
	std::shared_ptr<SteeringControl> steering;

	void topic_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const {
		float cappedAngle = std::fmax(std::fmin(msg->steering_angle, M_PI), -M_PI);
		int32_t steeringDemandStepper = cappedAngle * 5000 / M_PI;

		RCLCPP_INFO(this->get_logger(), "Stepper: %i", steeringDemandStepper);
		RCLCPP_INFO(this->get_logger(), "Radians: %f", cappedAngle);
		this->steering->target_position(steeringDemandStepper);
		this->steering->reached_target();
	}
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;

   public:
	Steering() : Node("steering") {
        this->declare_parameter<std::string>("ip", "");
		this->declare_parameter<int>("port", 0);
		
		// Defaults
		this->declare_parameter<int>("d_acceleration", 0);
		this->declare_parameter<int>("d_current", 0);
		this->declare_parameter<int>("d_limits", 0);
		this->declare_parameter<int>("d_velocity", 0);

		this->subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			"steering", 10, std::bind(&Steering::topic_callback, this, _1));

		std::string _ip;
		int _port;
		this->get_parameter("ip", _ip);
		this->get_parameter("port", _port);

		uint32_t _d_acceleration;
		uint32_t _d_current;
		uint32_t _d_limits;
		int32_t _d_velocity;
		this->get_parameter("d_acceleration", _d_acceleration);
		this->get_parameter("d_current", _d_current);
		this->get_parameter("d_limits", _d_limits);
		this->get_parameter("d_velocity", _d_velocity);

		if(_ip == "") {
			RCLCPP_ERROR(this->get_logger(), "Please provide a rosparam yaml file!");
			rclcpp::shutdown();
		}

		this->c = std::make_shared<Can2Ethernet>(_ip, _port);
		
		c5e_config_t config;
		config.default_accelerations = _d_acceleration;
		config.default_current = _d_current;
		config.default_limits = _d_limits;
		config.default_velocity = _d_velocity;

		this->steering = std::make_shared<SteeringControl>(c, config);
		this->steering->set_acceleration(std::make_pair<uint32_t, uint32_t>(10000, 10000));
	}

	~Steering() { this->steering->shutdown(); }
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Steering>());
	rclcpp::shutdown();
	return 0;
}
