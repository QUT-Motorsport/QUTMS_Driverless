#pragma once

#include <stddef.h>
#include <stdint.h>

void sdo_read(uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t *can_packet_id, uint8_t *out);
void sdo_write(uint8_t node_id, uint16_t index, uint8_t sub_index, uint8_t *data, size_t data_size,
			   uint32_t *can_packet_id, uint8_t *out);