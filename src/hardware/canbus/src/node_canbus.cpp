#include <algorithm>
#include <bitset>
#include <iostream>

#include "can2etherenet_adapter.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

const int FRAME_LENGTH = 13;
const int FI_RESERVED_MASK = 0x30;

class CanBus : public rclcpp::Node {
   private:
	std::shared_ptr<Can2Ethernet> c;

	void canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const {
		this->c->tx(msg->id, msg->id_type, msg->data.data());
	}

	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr subscription_;

	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr publisher_;

	void canmsg_timer_callback() {
		auto res = this->c->rx();
		if (res != nullptr) {
			for (size_t i = 0; i < res->size() / FRAME_LENGTH; i++) {
				std::vector<char> packet(FRAME_LENGTH);
				std::copy(res->begin() + (FRAME_LENGTH * i), res->begin() + (FRAME_LENGTH * (i + 1)), packet.begin());

				if (this->validate_frame(std::make_shared<std::vector<char>>(packet))) {
					uint32_t id = packet.at(1) << 24 | packet.at(2) << 16 | packet.at(3) << 8 | (packet.at(4) & 0xFF);
					std::vector<uint8_t> data_bytes(8);
					for (int i = 0; i < 8; i++) {
						data_bytes.at(i) = packet.at(5 + i);
					}

					char frame_information = packet.at(0);
					bool extended, remote;
					int dlc;

					this->parse_frame_information(frame_information, &extended, &remote, &dlc);

					driverless_msgs::msg::Can msg;
					msg.id = id;
					msg.id_type = extended;
					msg.dlc = dlc;
					msg.data = data_bytes;
					this->publisher_->publish(msg);
				}
			}
		}
	}

	rclcpp::TimerBase::SharedPtr timer_;

   public:
	CanBus() : Node("canbus") {
		// Can2Ethernet parameters
		this->declare_parameter<std::string>("ip", "");
		this->declare_parameter<int>("port", 0);

		this->subscription_ = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_carbound", 10, std::bind(&CanBus::canmsg_callback, this, _1));

		this->publisher_ = this->create_publisher<driverless_msgs::msg::Can>("canbus_rosbound", 10);

		std::string _ip;
		int _port;
		this->get_parameter("ip", _ip);
		this->get_parameter("port", _port);

		if (_ip == "") {
			RCLCPP_ERROR(this->get_logger(), "Please provide a rosparam yaml file!");
			rclcpp::shutdown();
		}

		RCLCPP_INFO(this->get_logger(), "Creating Connection on %s:%i...", _ip.c_str(), _port);

		this->c = std::make_shared<Can2Ethernet>(_ip, _port);

		RCLCPP_INFO(this->get_logger(), "done!");
		RCLCPP_INFO(this->get_logger(), "Creating Timer...");
		this->timer_ =
			this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CanBus::canmsg_timer_callback, this));
		RCLCPP_INFO(this->get_logger(), "done!");
	}

	bool validate_frame(std::shared_ptr<std::vector<char>> frame) {
		if (frame->size() < FRAME_LENGTH) {
			return false;
		}

		char frame_information = frame->at(0);
		bool extended, remote;
		int dlc;

		if (!this->parse_frame_information(frame_information, &extended, &remote, &dlc)) {
			return false;
		}

		std::vector<char> id_bytes = std::vector<char>(frame->begin() + 1, frame->begin() + 4);
		std::vector<char> data_bytes = std::vector<char>(frame->begin() + 5, frame->begin() + 8);

		for (int i = dlc; i < 8; i++) {
			if (data_bytes[i] != 0) {
				return false;
			}
		}

		return true;
	}

	bool parse_frame_information(char input, bool* extended, bool* remote, int* dlc) {
		*dlc = input & 0xF;
		*remote = (input & 0x40) == 0x40;
		*extended = (input & 0x80) == 0x80;

		if ((input & FI_RESERVED_MASK) != 0) {
			return false;
		}

		if (*dlc > 8) {
			return false;
		}

		return true;
	}

	~CanBus() { this->c->~Can2Ethernet(); }
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CanBus>());
	rclcpp::shutdown();
	return 0;
}