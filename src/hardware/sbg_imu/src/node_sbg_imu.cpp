#include <iostream>

#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_can.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/temperature.hpp"

using std::placeholders::_1;

class SBGIMU : public rclcpp::Node {
   private:
	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;

	// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data;
	// rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp;

	void canbus_callback(const driverless_msgs::msg::Can msg) {
		switch (msg.id) {
			case SBG_ECAN_MSG_STATUS_01_ID: {
				uint32_t id;
				uint32_t time_stamp;
				uint16_t general_status;
				uint16_t clock_status;
				Parse_SBG_ECAN_MSG_STATUS_01(const_cast<uint8_t *>(msg.data.data()), &id, &time_stamp, &general_status,
											 &clock_status);
			} break;
			case SBG_ECAN_MSG_IMU_ACCEL_ID: {
				float x, y, z;
				uint32_t id;
				Parse_SBG_ECAN_MSG_IMU_ACCEL(const_cast<uint8_t *>(msg.data.data()), &id, &x, &y, &z);
				RCLCPP_INFO(this->get_logger(), "XYZ: [%f, %f, %f]", x, y, z);
			} break;
		}
	}

   public:
	SBGIMU() : Node("sbg") {
		RCLCPP_INFO(this->get_logger(), "starting sbg node setup...");
		this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
		this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_rosbound", 10, std::bind(&SBGIMU::canbus_callback, this, _1));

		// this->imu_data = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
		// this->imu_temp = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);

		RCLCPP_INFO(this->get_logger(), "done");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SBGIMU>());
	rclcpp::shutdown();
	return 0;
}