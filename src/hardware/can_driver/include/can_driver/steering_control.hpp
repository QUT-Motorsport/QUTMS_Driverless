#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include "can2etherenet_adapter.hpp"
#include "canopen.hpp"
#include "rclcpp/rclcpp.hpp"

#define C5_E_ID 0x70  // C5-E-2-9 Controller Node ID

#define DEFAULT_VELOCITY 1000		// 1000 Encoder Counts / s
#define DEFAULT_ACCELERATIONS 5000	// 5000 Encoder Counts / s^2
#define DEFAULT_LIMITS 5000			// 5000, -5000 Encoder Counts
#define DEFAULT_CURRENT 5000		// 5000mA

class SteeringControl {
   private:
	std::shared_ptr<Can2Ethernet> can;
	int32_t target;
	int32_t velocity;
	uint32_t current;
	std::pair<uint32_t, uint32_t> accelerations;
	std::pair<int32_t, int32_t> limits;

   public:
	SteeringControl(std::shared_ptr<Can2Ethernet> can);
	~SteeringControl();

	// Target a position with configured velocity and accelerations
	void targetPosition(int32_t target);
	// Target a position with custom velocity and accelerations, does not set
	// internal values
	void targetPosition(int32_t target, int32_t velocity);
	// Set the limits for the motor
	void setLimits(std::pair<int32_t, int32_t> limits);
	// Set the internal velocity limit for the motor
	void setVelocity(int32_t velocity);
	// Set hte internal acceleration limits for the motor
	void setAcceleration(std::pair<uint32_t, uint32_t> accelerations);
	// Shutdown the controller
	void shutdown();

	bool reachedTarget();
};