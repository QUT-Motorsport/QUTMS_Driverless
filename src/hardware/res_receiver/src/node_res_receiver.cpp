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
				/*
				RES has reported in, request state change to enable it

				Doing so will result in the RES reporting PDOs
				2000 - 20007 every 30ms with the ID 0x180 + RES_NODE_ID

				Byte 0 = state command (start up)
				Byte 1 = Node ID (0x00 = All Nodes)
				*/

				uint8_t p[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
				this->can_pub->publish(this->_d_2_f(0x00, false, p));

			} break;

			case (0x180 + RES_NODE_ID): {
				// RES Reciever Status Packet
				this->_internal_status_time = this->now();	// Debug information

				// Store in local state
				this->res_status.estop = msg.data[0] & (1 << 0);						   // ESTOP = PDO 2000 Bit 0
				this->res_status.sw_k2 = msg.data[0] & (1 << 1);						   // K2 = PDO 2000 Bit 1
				this->res_status.bt_k3 = msg.data[0] & (1 << 2);						   // K3 = PDO 2000 Bit 2
				this->res_status.radio_quality = msg.data[6];							   // Radio Quality = PDO 2006
				this->res_status.loss_of_signal_shutdown_notice = msg.data[7] & (1 << 6);  // LoSSN = PDO 2007 Bit 6

				// Publish state
				this->res_pub->publish(this->res_status);

				// Log RES state
				// RCLCPP_INFO(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
				// 			this->res_status.sw_k2, this->res_status.bt_k3, this->res_status.estop,
				// 			this->res_status.radio_quality);
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
		RCLCPP_INFO(this->get_logger(), "res node setup complete");

		RCLCPP_INFO(this->get_logger(), "Attemping to start res");
		uint8_t p[8] = {0x80, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		this->can_pub->publish(this->_d_2_f(0x00, false, p));
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		uint8_t p2[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		this->can_pub->publish(this->_d_2_f(0x00, false, p2));
		RCLCPP_INFO(this->get_logger(), "res startup complete");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RESReceiver>());
	rclcpp::shutdown();
	return 0;
}