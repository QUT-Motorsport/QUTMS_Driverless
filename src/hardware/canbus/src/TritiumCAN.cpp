#include "TritiumCAN.hpp"

#include <ifaddrs.h>
#include <netdb.h>

#include <iostream>
#include <string>

std::string groupAddr("239.255.60.60");

TritiumCAN::TritiumCAN() { this->isConnected = false; }

bool TritiumCAN::setup(std::string ip) {
    int port = 4876;
    std::string localLoopbackAddr("127.0.0.1");

    this->rxClient = std::make_shared<UDPClient>();
    this->rxClient->setup(htonl(INADDR_ANY), port, false);

    this->tcpClient = std::make_shared<TCPClient>();
    this->tcpClient->setup(ip, port);

    this->txInitial = false;

    // look through all valid interfaces and join multicast group on each
    this->txClients.clear();

    struct ifaddrs *interfaceAddrLL;
    getifaddrs(&interfaceAddrLL);
    char host[NI_MAXHOST];

    for (struct ifaddrs *ifCurrent = interfaceAddrLL; ifCurrent != NULL; ifCurrent = ifCurrent->ifa_next) {
        sockaddr *ifAddrRaw = ifCurrent->ifa_addr;
        if (ifAddrRaw == NULL) {
            continue;
        }

        // check if IPv4
        if (ifAddrRaw->sa_family == AF_INET) {
            sockaddr_in *ifAddr = (sockaddr_in *)ifAddrRaw;
            if (inet_addr(localLoopbackAddr.c_str()) != ifAddr->sin_addr.s_addr) {
                if (getnameinfo(ifAddrRaw, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST) ==
                    0) {
                    std::string hostname(host);
                    std::cout << "Found interface: " << hostname << std::endl;

                    rxClient->join_multicast_group(inet_addr(groupAddr.c_str()), ifAddr->sin_addr.s_addr);

                    if (ifAddr->sin_addr.s_addr == inet_addr("192.168.2.1")) {
                        // create tx client for this interface
                        auto txClient = std::make_shared<UDPClient>();
                        txClient->setup(ifAddr->sin_addr.s_addr, port, true);
                        txClient->join_multicast_group(inet_addr(groupAddr.c_str()), ifAddr->sin_addr.s_addr);

                        txClients.push_back(txClient);
                        break;
                    }
                }
            }
        }
    }

    freeifaddrs(interfaceAddrLL);

    this->isConnected = true;
    return true;
}

void TritiumCAN::tx(driverless_msgs::msg::Can *msg) {
    // std::vector<driverless_msgs::msg::Can> msgs;
    // msgs.push_back(*msg);

    if (!this->txInitial) {
        auto headerData = this->compose_tritium_tcp_header();

        bool result = this->tcpClient->send_data(headerData);

        if (result) {
            this->txInitial = true;
        }
    }

    auto msgData = this->compose_tritum_can_bytes(*msg);
    this->tcpClient->send_data(msgData);

    /*
        auto data = this->compose_tritium_packet(msgs);

        for (auto &&udpTxClient : this->txClients) {
            udpTxClient->send_data(data, inet_addr(groupAddr.c_str()));
        }
    */
}

std::shared_ptr<std::vector<uint8_t>> TritiumCAN::compose_tritium_tcp_header() {
    auto result = std::make_shared<std::vector<uint8_t>>();

    uint32_t fwdId = 0x00;
    uint32_t fwdRange = 0x00;  // 0x1FFFFFFF;

    for (int i = 0; i < 4; i++) {
        result->push_back((fwdId >> ((3 - i) * 8)) & 0xFF);
    }

    for (int i = 0; i < 4; i++) {
        result->push_back((fwdRange >> ((3 - i) * 8)) & 0xFF);
    }

    result->push_back(0);

    uint64_t protocolVersion = 0x547269FDD6;
    uint16_t busNumber = 13;

    // TODO: do this based on actual mac address?
    uint64_t clientId = 0x00155DAE73CE;

    for (int i = 0; i < 5; i++) {
        result->push_back((protocolVersion >> ((4 - i) * 8)) & 0xFF);
    }

    for (int i = 0; i < 2; i++) {
        result->push_back((busNumber >> ((1 - i) * 8)) & 0xFF);
    }

    result->push_back(0);
    for (int i = 0; i < 7; i++) {
        result->push_back((clientId >> ((6 - i) * 8)) & 0xFF);
    }

    return result;
}

std::shared_ptr<std::vector<uint8_t>> TritiumCAN::compose_tritium_packet(std::vector<driverless_msgs::msg::Can> msgs) {
    auto result = std::make_shared<std::vector<uint8_t>>();

    result->push_back(0);

    uint64_t protocolVersion = 0x547269FDD6;
    uint16_t busNumber = 13;

    // TODO: do this based on actual mac address?
    uint64_t clientId = 0x00155DAE73CE;

    for (int i = 0; i < 5; i++) {
        result->push_back((protocolVersion >> ((4 - i) * 8)) & 0xFF);
    }

    for (int i = 0; i < 2; i++) {
        result->push_back((busNumber >> ((1 - i) * 8)) & 0xFF);
    }

    result->push_back(0);
    for (int i = 0; i < 7; i++) {
        result->push_back((clientId >> ((6 - i) * 8)) & 0xFF);
    }

    for (auto &&msg : msgs) {
        auto msgBytes = this->compose_tritum_can_bytes(msg);
        result->insert(result->end(), msgBytes->begin(), msgBytes->end());
    }

    return result;
}

std::shared_ptr<std::vector<uint8_t>> TritiumCAN::compose_tritum_can_bytes(driverless_msgs::msg::Can msg) {
    auto result = std::make_shared<std::vector<uint8_t>>();

    for (int i = 0; i < 4; i++) {
        result->push_back((msg.id >> ((3 - i) * 8)) & 0xFF);
    }

    bool RTR = false;
    uint8_t flags = (RTR ? (1 << 1) : 0) | (msg.id_type ? (1 << 0) : 0);
    result->push_back(flags);
    result->push_back(msg.dlc);

    for (int i = 0; i < msg.dlc; i++) {
        result->push_back(msg.data.data()[i]);
    }

    for (int i = msg.dlc; i < 8; i++) {
        result->push_back(0);
    }

    return result;
}

std::shared_ptr<std::vector<driverless_msgs::msg::Can>> TritiumCAN::rx() {
    auto msgs = std::make_shared<std::vector<driverless_msgs::msg::Can>>();

    // auto rxData = this->tcpClient->recieve_data();

    auto rxData = this->rxClient->recieve_data();

    while (rxData != nullptr) {
        if (rxData->size() < 16) {
            break;
        }

        int numCANBytes = rxData->size() - 16;
        int numCAN = numCANBytes / CAN_MSG_LEN;

        // std::cout << ", Num CAN: " << numCAN << std::endl;

        for (int i = 0; i < numCAN; i++) {
            uint8_t data[CAN_MSG_LEN];
            std::copy(rxData->begin() + 16 + i * CAN_MSG_LEN, rxData->begin() + 16 + (i + 1) * CAN_MSG_LEN, data);

            driverless_msgs::msg::Can msg;
            bool result = this->process_can_msg(data, &msg);
            if (result) {
                msgs->push_back(msg);
            } else {
                std::cout << "RX - INVALID CAN PACKET " << std::endl;
            }
        }

        // check queue for more stuff
        rxData = this->rxClient->recieve_data();
    }

    // uint64_t protocolVersion = 0;

    // for (int i = 0; i < 5; i++) {
    //     protocolVersion |= ((uint64_t)rxData->data()[1 + i]) << ((4 - i) * 8);
    // }
    // std::cout << "Protocol Version: " << std::hex << protocolVersion << std::dec;

    // uint16_t busNumber = (rxData->data()[6] << 8) | (rxData->data()[7]);
    // std::cout << ", Bus Number: " << busNumber;

    // uint64_t clientIdentifier = 0;
    // for (int i = 0; i < 7; i++) {
    //     clientIdentifier |= ((uint64_t)rxData->data()[9 + i]) << ((6 - i) * 8);
    // }
    // std::cout << ", Client Identifier: " << std::hex << clientIdentifier << std::dec;

    return msgs;
}

bool TritiumCAN::process_can_msg(uint8_t *data, driverless_msgs::msg::Can *msg) {
    // // std::cout << "CAN: ";
    // for (int i = 0; i < CAN_MSG_LEN; i++) {
    // }
    //     std::cout << std::hex << (int)data[i] << " ";
    // std::cout << std::dec << std::endl;

    uint32_t canID = 0;
    for (int i = 0; i < 4; i++) {
        canID |= ((uint32_t)data[0 + i]) << ((3 - i) * 8);
    }
    msg->id = canID;

    uint8_t flags = data[4];
    // bool tritiumHB = (flags & (1 << 7)) > 0;
    // bool settings = (flags & (1 << 6)) > 0;
    // bool rtr = (flags & (1 << 1)) > 0;
    bool extended = (flags & (1 << 0)) > 0;

    if (extended) {
        if (((canID & 0xE0000000) != 0)) {
            // extended, so only lowest 29 bits should have values, top 3 bits should be 0
            std::cout << "bad extended id" << std::endl;
            return false;
        }
    } else {
        if (((canID & (~0x7FF)) != 0)) {
            // standard, so only lowest 11 bits should have values, top 21 bits should be 0
            std::cout << "bad standard id" << std::endl;
            return false;
        }
    }

    msg->id_type = extended;

    uint8_t DLC = data[5];

    if (DLC > 8) {
        // DLC cannot be greater than 8
        std::cout << "bad DLC" << std::endl;
        return false;
    }

    msg->dlc = DLC;

    // TODO: why aren't all the bytes 0 lol
    /*
        if (DLC < 8) {
            for (int i = DLC; i < 8; i++) {
                if (data[6 + i] != 0) {
                    // all bytes after dlc amount should be 0
                    std::cout << "bad data" << std::endl;
                    return false;
                }
            }
        }
    */
    std::vector<uint8_t> msgData;
    for (int i = 0; i < msg->dlc; i++) {
        msgData.push_back(data[6 + i]);
    }
    msg->data = msgData;

    // std::cout << "RX - ID: " << std::hex << msg->id << std::dec << ", Flags: " << std::hex << (int)flags << std::dec
    //           << ", DLC: " << (int)msg->dlc << ", Data: [";
    // for (int i = 0; i < DLC; i++) {
    //     std::cout << std::hex << (int)msg->data[i] << std::dec << ", ";
    // }
    // std::cout << "]" << std::endl;

    return true;
}

TritiumCAN::~TritiumCAN() {}
