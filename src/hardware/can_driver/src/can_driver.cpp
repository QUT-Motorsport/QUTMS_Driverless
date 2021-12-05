#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can2etherenet_adapter.hpp"
#include "steering_control.hpp"

int main(int argc, char **argv) {
	(void)argc;
	(void)argv;
	std::shared_ptr<Can2Ethernet> c = std::make_shared<Can2Ethernet>("192.168.0.8", 20001);

	c5e_config_t config;
	config.default_accelerations = 5000;
	config.default_current = 5000;
	config.default_limits = 4500;
	config.default_velocity = 1000;

	std::shared_ptr<SteeringControl> steering = std::make_shared<SteeringControl>(c, config);
	steering->set_acceleration(std::make_pair<uint32_t, uint32_t>(20000, 20000));
	for (;;) {
		steering->target_position(-5000, 10000);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		// while(!steering->reachedTarget());
		steering->target_position(5000, 10000);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// while(!steering->reachedTarget());
	}
	return 0;
}
