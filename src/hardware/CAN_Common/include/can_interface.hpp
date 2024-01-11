#pragma once

#include <iostream>

#include "driverless_msgs/msg/can.hpp"

// CAN Open nodes
// RES is on Node 0x011
// FSG Rules: "The Node-ID has to be set to 0x011 at the competition. Only in severe cases,
// there will be an exception. Please give a detail problem description with the request"

#define RES_NODE_ID 0x011
#define RES_BOOT_UP_ID 0x700 + RES_NODE_ID
#define RES_HEARTBEAT_ID 0x180 + RES_NODE_ID

// C5E Steering motor controller is on Node 0x070
// configured via rotary switches 0x0 - 0x7
#define C5E_NODE_ID 0x70
#define C5E_BOOT_UP_ID 0x700 + C5E_NODE_ID
#define C5E_EMCY_ID 0x80 + C5E_NODE_ID
#define C5E_STATUS_ID 0x180 + C5E_NODE_ID
#define C5E_TX_PDO1_ID 0x200 + C5E_NODE_ID
#define C5E_TX_PDO2_ID 0x300 + C5E_NODE_ID
#define C5E_POS_ID 0x280 + C5E_NODE_ID
#define C5E_SRV_ID 0x580 + C5E_NODE_ID

class CanInterface {
   protected:
    void copy_data(const std::vector<uint8_t> &vec, uint8_t *dest, size_t n);
    // Convert a data array, bool and id information to a canbus frame (data_2_frame)
    driverless_msgs::msg::Can _d_2_f(uint32_t id, bool is_extended, uint8_t *data, uint8_t dlc);
};
