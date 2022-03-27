#include <iostream>

#include "can_interface.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "rclcpp/rclcpp.hpp"

const int RES_NODE_ID = 0x011;

using std::placeholders::_1;

class RESReceiver : public rclcpp::Node, public CanInterface {
   private:
	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
	rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_pub;

	driverless_msgs::msg::RES res_status;

	rclcpp::Time _internal_status_time;

	void canbus_callback(const driverless_msgs::msg::Can msg) {
		switch (msg.id) {
			case (0x700 + RES_NODE_ID): {
				// Wake up message from RES Reciever
				// Order start up of RES Reciever
				uint8_t p[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
				this->can_pub->publish(this->_d_2_f(0x00, false, p));

			} break;

			case (0x180 + RES_NODE_ID): {
				// RES Reciever Status Packet
				this->_internal_status_time = this->now();	// Debug information

				// Store in local state
				this->res_status.estop = msg.data[9] & (1 << 0);
				this->res_status.sw_k2 = msg.data[0] & (1 << 1);
				this->res_status.bt_k3 = msg.data[0] & (1 << 2);
				this->res_status.radio_quality = msg.data[6];
				this->res_status.loss_of_signal_shutdown_notice = msg.data[7] & (1 << 6);

				// Publish state
				this->res_pub->publish(this->res_status);
			} break;

			default:
				break;
		}
	}

   public:
	RESReceiver() : Node("res") {
		RCLCPP_INFO(this->get_logger(), "starting res node setup...");
		this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
		this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_rosbound", 10, std::bind(&RESReceiver::canbus_callback, this, _1));
		this->res_pub = this->create_publisher<driverless_msgs::msg::RES>("res_status", 10);
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RESReceiver>());
	rclcpp::shutdown();
	return 0;
}