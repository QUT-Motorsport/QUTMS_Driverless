#pragma once

#include <iostream>

#include "driverless_msgs/msg/can.hpp"

class CanInterface {
   protected:
	// Convert a data array, bool and id information to a canbus frame (data_2_frame)
	driverless_msgs::msg::Can _d_2_f(uint32_t id, bool is_extended, uint8_t *data);
};