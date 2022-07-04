#include "canopen.hpp"

void sdo_read(uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t* can_packet_id, uint8_t* out) {
	*can_packet_id = 0x600 + node_id;

	out[0] = 0x40;
	out[1] = index & 0xFF;
	out[2] = (index >> 8) & 0xFF;
	out[3] = sub_index;

	out[4] = 0x00;
	out[5] = 0x00;
	out[6] = 0x00;
	out[7] = 0x00;
}

void sdo_write(uint8_t node_id, uint16_t index, uint8_t sub_index, uint8_t* data, size_t data_size,
			   uint32_t* can_packet_id, uint8_t* out) {
	*can_packet_id = 0x600 + node_id;
	uint8_t free = 4 - data_size;
	out[0] = 35;
	out[0] = out[0] | ((free << 2) & 0x1E);

	out[1] = index & 0xFF;
	out[2] = (index >> 8) & 0xFF;
	out[3] = sub_index;

	for (size_t i = 0; i < data_size; i++) {
		out[i + 4] = data[i];
	}
}