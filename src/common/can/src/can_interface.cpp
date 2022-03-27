#include "can_interface.hpp"

driverless_msgs::msg::Can CanInterface::_d_2_f(uint32_t id, bool is_extended, uint8_t *data) {
	driverless_msgs::msg::Can frame;
	frame.id = id;
	frame.id_type = is_extended;
	std::vector<uint8_t> v;
	v.assign(data, data + 8);
	frame.data = v;
	return frame;
}