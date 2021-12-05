#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include "can2etherenet_adapter.hpp"
#include "canopen.hpp"
#include "rclcpp/rclcpp.hpp"

#define C5_E_ID 0x70  // C5-E-2-9 Controller Node ID

typedef struct c5e_config {
	int32_t default_velocity;
	uint32_t default_accelerations;
	uint32_t default_limits;
	uint32_t default_current;

	std::string to_string() {
		std::stringstream ss;
		ss << "vel: " << this->default_velocity << " acc: " << this->default_accelerations << " lims: " << this->default_limits << " curr: " << this->default_current << std::endl;
		return ss.str();
	}
} c5e_config_t;

class SteeringControl {
   private:
	std::shared_ptr<Can2Ethernet> can;
	int32_t target;
	int32_t velocity;
	uint32_t current;
	std::pair<uint32_t, uint32_t> accelerations;
	std::pair<int32_t, int32_t> limits;
	c5e_config_t defaults;

   public:
	SteeringControl(std::shared_ptr<Can2Ethernet> can, c5e_config_t config);
	~SteeringControl();

	// Target a position with configured velocity and accelerations
	void target_position(int32_t target);
	// Target a position with custom velocity and accelerations, does not set
	// internal values
	void target_position(int32_t target, int32_t velocity);
	// Set the limits for the motor
	void set_limits(std::pair<int32_t, int32_t> limits);
	// Set the internal velocity limit for the motor
	void set_velocity(int32_t velocity);
	// Set hte internal acceleration limits for the motor
	void set_acceleration(std::pair<uint32_t, uint32_t> accelerations);
	// Shutdown the controller
	void shutdown();

	// Check if the target reached bit has been set
	bool reached_target();

	// set and get default config
	void set_c5e_config(c5e_config_t config);
	c5e_config_t get_c5e_config();
};