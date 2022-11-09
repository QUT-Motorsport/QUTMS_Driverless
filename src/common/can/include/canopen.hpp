#pragma once

#include <stddef.h>
#include <stdint.h>

#include <map>

void sdo_read(uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t *can_packet_id, uint8_t *out);
void sdo_write(uint8_t node_id, uint16_t index, uint8_t sub_index, uint8_t *data, size_t data_size,
               uint32_t *can_packet_id, uint8_t *out);

std::map<uint8_t, size_t> can_open_size_map = {
    {1, 0x4F},
    {2, 0x4B},
    {3, 0x47},
    {4, 0x43},
};
