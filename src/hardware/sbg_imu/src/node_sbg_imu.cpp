#include <iostream>

#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_can.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using std::placeholders::_1;

class SBGIMU : public rclcpp::Node {
   private:
	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp;

	rclcpp::TimerBase::SharedPtr imu_data_timer;
	rclcpp::TimerBase::SharedPtr imu_temp_timer;

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

	std::map<std::string, int> param_defaults = {{"imu_info", 1},
												 {"imu_accelerometer", 100},
												 {"imu_gyrometer", 100},
												 {"imu_delta_velocity", 100},
												 {"imu_delta_angle", 100},

												 {"orientation_info", 1},
												 {"orientation_quaternion", 100},
												 {"orientation_euler", 50},
												 {"orientation_orientation_acc", 50},

												 {"navigation_position", 50},
												 {"navigation_altitude", 10},
												 {"navigation_position_acc", 10},
												 {"navigation_velocity", 10},
												 {"navigation_velocity_acc", 10},
												 {"navigation_velocity_body", 10},

												 {"automotive_track_slip_curve", 50},

												 {"gps_1_velocity_info", 1},
												 {"gps_1_velocity", 10},
												 {"gps_1_velocity_acc", 10},
												 {"gps_1_course", 10},
												 {"gps_1_position_info", 1},
												 {"gps_1_position", 10},
												 {"gps_1_altitude", 10},
												 {"gps_1_position_acc", 10}};

   public:
	SBGIMU() : Node("sbg") {
		// declare and set all params
		this->declare_parameters("", param_defaults);

		std::vector<std::string> param_keys;

		std::transform(param_defaults.begin(), param_defaults.end(), std::back_inserter(param_keys),
					   [](const std::map<std::string, int>::value_type &pair) { return pair.first; });

		this->get_parameters(param_keys);

		RCLCPP_INFO(this->get_logger(), "starting sbg node setup...");
		this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
		this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_rosbound", 10, std::bind(&SBGIMU::canbus_callback, this, _1));

		this->imu_data = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
		this->imu_temp = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);

		RCLCPP_INFO(this->get_logger(), "done");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SBGIMU>());
	rclcpp::shutdown();
	return 0;
}