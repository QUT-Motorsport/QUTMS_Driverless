#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string>

#include <memory>
#include <vector>

#include "driverless_msgs/msg/can.hpp"
#include "udp_client.hpp"
#include "tcp_client.hpp"

const int CAN_MSG_LEN = (4 + 2 + 8);

class TritiumCAN {
    std::shared_ptr<UDPClient> rxClient;
    std::vector<std::shared_ptr<UDPClient>> txClients;
    std::shared_ptr<TCPClient> tcpClient;

    bool isConnected;
    bool txInitial;
    bool rxInitial;

   public:
    TritiumCAN();

    bool setup(std::string ip);

    void tx(driverless_msgs::msg::Can *msg);
    std::shared_ptr<std::vector<driverless_msgs::msg::Can>> rx();
    bool process_can_msg(uint8_t *data, driverless_msgs::msg::Can* msg);
    std::shared_ptr<std::vector<uint8_t>> compose_tritium_tcp_header();
    std::shared_ptr<std::vector<uint8_t>> compose_tritium_packet(std::vector<driverless_msgs::msg::Can> msgs);
    std::shared_ptr<std::vector<uint8_t>> compose_tritum_can_bytes(driverless_msgs::msg::Can msg);

    ~TritiumCAN();
};
