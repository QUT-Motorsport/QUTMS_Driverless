#include <iostream>

#include "driverless_msgs/msg/can.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_can.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

using std::placeholders::_1;

class SBGIMU : public rclcpp::Node {
   private:
	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr imu_vel;
	rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr imu_utc;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr imu_sat;

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
	std::vector<std::string> param_keys;

	// Buffer messages for each of the topics we publish
	sensor_msgs::msg::Imu imu_buffer_msg;
	uint8_t imu_bufffer_bitcode;

	sensor_msgs::msg::Temperature temperature_buffer_msg;
	uint8_t temperature_buffer_bitcode;

	geometry_msgs::msg::TwistStamped velocity_buffer_msg;
	uint8_t velocity_buffer_bitcode;

	sensor_msgs::msg::TimeReference utc_buffer_msg;
	uint8_t utc_buffer_bitcode;

	sensor_msgs::msg::NavSatFix sat_buffer_msg;
	uint8_t sat_buffer_bitcode;

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
				geometry_msgs::msg::Vector3 lin;
				Parse_SBG_ECAN_MSG_IMU_ACCEL(const_cast<uint8_t *>(msg.data.data()), &id, &x, &y, &z);

				this->imu_buffer_msg.linear_acceleration.x = static_cast<double>(x);
				this->imu_buffer_msg.linear_acceleration.y = static_cast<double>(y);
				this->imu_buffer_msg.linear_acceleration.z = static_cast<double>(z);

				this->velocity_buffer_msg.twist.linear.x = static_cast<double>(x);
				this->velocity_buffer_msg.twist.linear.y = static_cast<double>(y);
				this->velocity_buffer_msg.twist.linear.z = static_cast<double>(z);

				// Set bitcode, if we have filled all information, send message
				this->imu_bufffer_bitcode |= 1;
				this->velocity_buffer_bitcode |= 1;

				if (this->imu_bufffer_bitcode == 7) {
					this->imu_bufffer_bitcode = 0;
					this->imu_data->publish(this->imu_buffer_msg);
				}

				if (this->velocity_buffer_bitcode == 3) {
					this->velocity_buffer_bitcode = 0;
					this->imu_vel->publish(this->velocity_buffer_msg);
				}
			} break;

			case SBG_ECAN_MSG_IMU_GYRO_ID: {
				float d_a_x, d_a_y, d_a_z;
				uint32_t id;
				geometry_msgs::msg::Vector3 ang;
				Parse_SBG_ECAN_MSG_IMU_GYRO(const_cast<uint8_t *>(msg.data.data()), &id, &d_a_x, &d_a_y, &d_a_z);

				this->imu_buffer_msg.angular_velocity.x = static_cast<double>(d_a_x);
				this->imu_buffer_msg.angular_velocity.y = static_cast<double>(d_a_y);
				this->imu_buffer_msg.angular_velocity.z = static_cast<double>(d_a_z);

				this->velocity_buffer_msg.twist.angular.x = static_cast<double>(d_a_x);
				this->velocity_buffer_msg.twist.angular.y = static_cast<double>(d_a_y);
				this->velocity_buffer_msg.twist.angular.z = static_cast<double>(d_a_z);

				// Set bitcode, if we have filled all information, send message
				this->imu_bufffer_bitcode |= 2;
				this->velocity_buffer_bitcode |= 2;

				if (this->imu_bufffer_bitcode == 7) {
					this->imu_bufffer_bitcode = 0;
					this->imu_data->publish(this->imu_buffer_msg);
				}

				if (this->velocity_buffer_bitcode == 3) {
					this->velocity_buffer_bitcode = 0;
					this->imu_vel->publish(this->velocity_buffer_msg);
				}
			} break;

			case SBG_ECAN_MSG_EKF_QUAT_ID: {
				float x, y, z, w;
				uint32_t id;
				Parse_SBG_ECAN_MSG_EKF_QUAT(const_cast<uint8_t *>(msg.data.data()), &id, &x, &y, &z, &w);

				this->imu_buffer_msg.orientation.x = x;
				this->imu_buffer_msg.orientation.y = y;
				this->imu_buffer_msg.orientation.z = z;
				this->imu_buffer_msg.orientation.w = w;

				// Set bitcode, if we have filled all information, send message
				this->imu_bufffer_bitcode = this->imu_bufffer_bitcode | 4;
				if (this->imu_bufffer_bitcode == 7) {
					this->imu_bufffer_bitcode = 0;
					this->imu_data->publish(this->imu_buffer_msg);
				}
			} break;

			case SBG_ECAN_MSG_IMU_INFO_ID: {
				uint32_t time_stamp;
				uint16_t imu_status;
				float temperature;
				uint32_t id;
				Parse_SBG_ECAN_MSG_IMU_INFO(const_cast<uint8_t *>(msg.data.data()), &id, &time_stamp, &imu_status,
											&temperature);
				this->temperature_buffer_msg.temperature = static_cast<double>(temperature);

				// Set bitcode, if we have filled all information, send message
				this->temperature_buffer_bitcode = this->temperature_buffer_bitcode | 1;
				if (this->temperature_buffer_bitcode == 1) {
					this->temperature_buffer_bitcode = 0;
					this->imu_temp->publish(this->temperature_buffer_msg);
				}
			} break;

			case SBG_ECAN_MSG_UTC_1_ID: {
				uint8_t year, month, day, hour, min, sec;
				uint16_t micro_sec;
				uint32_t id;
				Parse_SBG_ECAN_MSG_UTC_1(const_cast<uint8_t *>(msg.data.data()), &id, &year, &month, &day, &hour, &min,
										 &sec, &micro_sec);

				this->utc_buffer_msg.time_ref.sec;
			} break;
		}
	}

   public:
	SBGIMU() : Node("sbg") {
		// declare and set all params
		this->declare_parameters("", this->param_defaults);

		std::transform(this->param_defaults.begin(), this->param_defaults.end(), std::back_inserter(this->param_keys),
					   [](const std::map<std::string, int>::value_type &pair) { return pair.first; });

		std::vector<rclcpp::Parameter> _params = this->get_parameters(this->param_keys);

		RCLCPP_INFO(this->get_logger(), "starting sbg node setup...");
		this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
		this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_rosbound", 10, std::bind(&SBGIMU::canbus_callback, this, _1));

		this->imu_data = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
		this->imu_temp = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);
		this->imu_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("/imu/velocity", 10);
		this->imu_utc = this->create_publisher<sensor_msgs::msg::TimeReference>("imu/utc_ref", 10);
		this->imu_sat = this->create_publisher<sensor_msgs::msg::NavSatFix>("imu/nav_sat_fix", 10);

		RCLCPP_INFO(this->get_logger(), "done");
	}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SBGIMU>());
	rclcpp::shutdown();
	return 0;
}