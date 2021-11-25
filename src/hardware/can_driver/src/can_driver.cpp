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
	std::shared_ptr<SteeringControl> steering = std::make_shared<SteeringControl>(c);
	steering->setAcceleration(std::make_pair<uint32_t, uint32_t>(10000, 10000));
	for (;;) {
		steering->targetPosition(-5000, 100);
		while(!steering->reachedTarget());
		steering->targetPosition(5000, 100);
		while(!steering->reachedTarget());
	}
	return 0;
}
