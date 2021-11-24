#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "can2etherenet_adapter.hpp"
#include "steering_control.hpp"

int main(int argc, char **argv) {
	(void)argc;
	(void)argv;
	std::shared_ptr<Can2Ethernet> c = std::make_shared<Can2Ethernet>("192.168.0.8", 20001);
	std::shared_ptr<SteeringControl> steering = std::make_shared<SteeringControl>(c);
	// steering->targetPosition(1000);
	// sleep(2);
	for (;;) {
		steering->targetPosition(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		steering->targetPosition(-5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	return 0;
}
