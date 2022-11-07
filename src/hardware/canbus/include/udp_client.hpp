#pragma once

#include <stdbool.h>
#include <arpa/inet.h>
#include <string>
#include <memory>
#include <vector>

#include <stdint.h>
#include <stdbool.h>

const int UDP_RECV_SIZE = 4096;

class UDPClient
{
private:
    int sock;

    in_addr_t address;
    int port;

public:
    UDPClient();
    bool setup(in_addr_t address, int port, bool loopback);
    bool join_multicast_group(in_addr_t group_addr, in_addr_t local_addr);

    bool send_data(std::shared_ptr<std::vector<uint8_t>> data, in_addr_t dest_addr);
    std::shared_ptr<std::vector<uint8_t>> recieve_data();

    bool _close();
    ~UDPClient() { this->_close(); }
};