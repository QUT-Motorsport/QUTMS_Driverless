#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "canopen.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

const int C5_E_ID = 0x70;

typedef struct c5e_config {
	int32_t default_velocity;
	uint32_t default_accelerations;
	uint32_t default_limits;
	uint32_t default_current;

	std::string to_string() {
		std::stringstream ss;
		ss << "vel: " << this->default_velocity << " acc: " << this->default_accelerations
		   << " lims: " << this->default_limits << " curr: " << this->default_current << std::endl;
		return ss.str();
	}
} c5e_config_t;

class SteeringActuator : public rclcpp::Node {
   private:
	int32_t target;
	int32_t velocity;
	int32_t current;
	std::pair<uint32_t, uint32_t> accelerations;
	std::pair<int32_t, int32_t> limits;
	c5e_config_t defaults;
	driverless_msgs::msg::Can _frame;

	rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
	rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr steering_sub;

	void canbus_callback(const driverless_msgs::msg::Can msg) { this->_frame = msg; }

	void steering_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
		float cappedAngle = std::fmax(std::fmin(msg->steering_angle, M_PI), -M_PI);
		int32_t steeringDemandStepper = cappedAngle * 5000 / M_PI;

		RCLCPP_INFO(this->get_logger(), "Stepper: %i", steeringDemandStepper);
		RCLCPP_INFO(this->get_logger(), "Radians: %f", cappedAngle);
		this->target_position(steeringDemandStepper);
	}

	rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;
		result.reason = "success";

		for (const auto &param : parameters) {
			if (param.get_name() == "d_acceleration") {
				RCLCPP_INFO(this->get_logger(), "Setting acceleration to %li.", param.as_int());
				this->set_acceleration(std::make_pair<uint32_t, uint32_t>(param.as_int(), param.as_int()));

			} else if (param.get_name() == "d_velocity") {
				RCLCPP_INFO(this->get_logger(), "Setting velocity to %li.", param.as_int());
				this->set_velocity(param.as_int());
			} else {
				RCLCPP_ERROR(this->get_logger(), "Do not set current and limits on the fly. Request ignored.");
				result.successful = false;
				result.reason = "Do not set current and limits on the fly.";
			}
		}

		return result;
	}

	OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

	driverless_msgs::msg::Can _d_2_f(uint32_t id, bool is_extended, uint8_t *data) {
		driverless_msgs::msg::Can frame;
		frame.id = id;
		frame.id_type = is_extended;
		std::vector<uint8_t> v;
		v.assign(data, data + 8);
		frame.data = v;
		return frame;
	}

   public:
	SteeringActuator() : Node("steering") {
		// Defaults
		this->declare_parameter<int>("d_acceleration", 0);
		this->declare_parameter<int>("d_current", 0);
		this->declare_parameter<int>("d_limits", 0);
		this->declare_parameter<int>("d_velocity", 0);

		this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
		this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
			"canbus_rosbound", 10, std::bind(&SteeringActuator::canbus_callback, this, _1));
		this->steering_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
			"steering", 10, std::bind(&SteeringActuator::steering_callback, this, _1));

		this->parameter_callback_handle =
			this->add_on_set_parameters_callback(std::bind(&SteeringActuator::parameter_callback, this, _1));

		uint32_t _d_acceleration;
		uint32_t _d_current;
		uint32_t _d_limits;
		int32_t _d_velocity;
		this->get_parameter("d_acceleration", _d_acceleration);
		this->get_parameter("d_current", _d_current);
		this->get_parameter("d_limits", _d_limits);
		this->get_parameter("d_velocity", _d_velocity);

		c5e_config_t config;
		config.default_accelerations = _d_acceleration;
		config.default_current = _d_current;
		config.default_limits = _d_limits;
		config.default_velocity = _d_velocity;

		RCLCPP_INFO(this->get_logger(), "Using c5e_config: %s", config.to_string().c_str());

		this->velocity = config.default_velocity;
		this->accelerations =
			std::make_pair<int32_t, int32_t>(config.default_accelerations, config.default_accelerations);
		this->limits = std::make_pair<int32_t, int32_t>(config.default_limits, -config.default_limits);

		this->current = config.default_current;
		this->target = 0;

		this->setup();
	}

	void setup() {
		/* To initialise the controller to a usable state, we must set the:
		Target Position = 0
		Min Position Limit = -DEFAULT_LIMITS
		Max Position Limit = DEFAULT_LIMITS
		Home Offset = 0
		Motion Profile Type = trapezoidal ramp
		Profile Velocity = DEFAULT_VELOCITY
		End Velocity = 0
		Profile Acceleration = DEFAULT_ACCELERATIONS
		Profile Deceleration = DEFAULT_ACCELERATIONS
		Quick Stop Deceleration = DEFAUL_ACCELERATIONS
		Max Acceleration = DEFAULT_ACCELERATIONS
		MAX Deceleration = DEFAULT_ACCELERATIONS
		Mode of Operation = 1 (Profile Position)
	*/

		uint32_t id;	 // Packet id out
		uint8_t out[8];	 // Data out

		std::cout << "Performing C5-E Setup... ";

		sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x607D, 0x01, (uint8_t *)&this->limits.second, 4, &id, out);	 // Min Limit
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x607D, 0x02, (uint8_t *)&this->limits.first, 4, &id, out);	// Max Limit
		this->can_pub->publish(_d_2_f(id, 0, out));

		int32_t ho = 0;
		sdo_write(C5_E_ID, 0x607C, 0x00, (uint8_t *)&ho, 4, &id, out);	// Home Offset
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6086, 0x00, (uint8_t *)&ho, 2, &id, out);	// Motion Profile Type (trap)
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);	// Profile Velocity
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6082, 0x00, (uint8_t *)&ho, 4, &id, out);	// End Velocity
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Profile Accelerataion
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);	// Profile Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
				  out);	 // Quick Stop Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Max Acceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);	// Max Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		int8_t ppm = 1;
		sdo_write(C5_E_ID, 0x6060, 0x00, (uint8_t *)&ppm, 1, &id, out);	 // Modes of Operation
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		uint16_t control_word = 6;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Shutdown
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		control_word = 7;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Switched On
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		control_word = 15;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Op Enabled
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::cout << "Done (Operation Enabled)" << std::endl;
	}

	void target_position(int32_t target) {
		// Set Target
		this->target = target;

		uint32_t id;	 // Packet id out
		uint8_t out[8];	 // Data out

		uint16_t control_word = 47;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
		this->can_pub->publish(_d_2_f(id, 0, out));

		// Set Control Word
		control_word = 63;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
		this->can_pub->publish(_d_2_f(id, 0, out));
	}

	void target_position(int32_t target, int32_t velocity) {
		this->target = target;

		uint32_t id;	 // Packet id out
		uint8_t out[8];	 // Data out

		sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
		this->can_pub->publish(_d_2_f(id, 0, out));

		this->velocity = velocity;

		sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);	// Profile Velocity
		this->can_pub->publish(_d_2_f(id, 0, out));

		uint16_t control_word = 47;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
		this->can_pub->publish(_d_2_f(id, 0, out));

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		// Set Control Word
		control_word = 63;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
		this->can_pub->publish(_d_2_f(id, 0, out));
	}

	void set_velocity(int32_t velocity) {
		this->velocity = velocity;

		uint32_t id;	 // Packet id out
		uint8_t out[8];	 // Data out

		sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);	// Profile Velocity
		this->can_pub->publish(_d_2_f(id, 0, out));
	}

	void set_acceleration(std::pair<uint32_t, uint32_t> accelerations) {
		this->accelerations = accelerations;

		uint32_t id;	 // Packet id out
		uint8_t out[8];	 // Data out

		sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
				  out);	 // Profile Accelerataion
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t *)&this->accelerations.second, 4, &id,
				  out);	 // Profile Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
				  out);	 // Quick Stop Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Max Acceleration
		this->can_pub->publish(_d_2_f(id, 0, out));

		sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);	// Max Deceleration
		this->can_pub->publish(_d_2_f(id, 0, out));
	}

	void shutdown() {
		uint32_t id;
		uint8_t out[8];

		uint16_t control_word = 6;
		sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Shutdown
		this->can_pub->publish(_d_2_f(id, 0, out));
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	void set_c5e_config(c5e_config_t config) {
		this->defaults.default_accelerations = config.default_accelerations;
		this->defaults.default_current = config.default_current;
		this->defaults.default_limits = config.default_limits;
		this->defaults.default_velocity = config.default_velocity;
	}

	c5e_config_t get_c5e_config() { return this->defaults; }
};

std::function<void(int)> handler;
void signal_handler(int signal) { handler(signal); }

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	// Hack
	auto x = std::make_shared<SteeringActuator>();
	signal(SIGINT, signal_handler);
	handler = [x](int signal) {
		for (int i = 0; i < 10; i++) {
			x->shutdown();
		}
		RCLCPP_INFO(x->get_logger(), "Motor shutdown, exiting node.");
		rclcpp::shutdown();
		return signal;
	};

	rclcpp::spin(x);
	// rclcpp::shutdown();
	return 0;
}