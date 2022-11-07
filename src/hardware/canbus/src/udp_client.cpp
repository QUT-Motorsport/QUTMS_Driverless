#include "udp_client.hpp"

#include <iostream>
#include <stdlib.h>
#include <memory>
#include <string.h>
#include <unistd.h>

UDPClient::UDPClient()
{
    this->sock = -1;
    this->port = 0;
    this->address = 0;
}

bool UDPClient::setup(in_addr_t address, int port, bool loopback)
{
    this->address = address;
    this->port = port;

    if (this->sock == -1)
    {
        this->sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (this->sock == -1)
        {
            std::cout << "Failed to create socket." << std::endl;
            return false;
        }
    }

    // set mode to reuse address
    int val = 1;
    if (setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0)
    {
        std::cout << "Failed to set SO_REUSEADDR." << std::endl;
        return false;
    }

    if (loopback)
    {
        if (setsockopt(this->sock, IPPROTO_IP, IP_MULTICAST_LOOP, &val, sizeof(val)) != 0)
        {
            std::cout << "Failed to set IP_MULTICAST_LOOP." << std::endl;
            return false;
        }
    }

    // bind to requested address
    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(address));
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = address;
    bindAddr.sin_port = htons(port);

    char buffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &bindAddr.sin_addr, buffer, INET_ADDRSTRLEN);

    std::string bindAddrStr(buffer);

    if (bind(this->sock, (sockaddr *)&bindAddr, sizeof(bindAddr)) != 0)
    {
        std::cout << "Unable to bind socket to: " << bindAddrStr << std::endl;
        return false;
    }
    else
    {
        std::cout << "Socket bound to: " << bindAddrStr << std::endl;
    }

    return true;
}

bool UDPClient::_close()
{
    if (this->sock != -1)
    {
        close(this->sock);
        this->sock = -1;
    }

    return true;
}

bool UDPClient::join_multicast_group(in_addr_t group_addr, in_addr_t local_addr)
{
    // join multicast
    struct ip_mreqn mreq;
    mreq.imr_multiaddr.s_addr = group_addr;
    mreq.imr_address.s_addr = local_addr;
    mreq.imr_ifindex = 0;

    if (setsockopt(this->sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0)
    {
        std::cout << "Failed to join multicast." << std::endl;
        return false;
    }

    return true;
}

bool UDPClient::send_data(std::shared_ptr<std::vector<uint8_t>> data, in_addr_t dest_addr)
{
    if (this->sock != -1)
    {
        struct sockaddr_in destEP;
        memset(&destEP, 0, sizeof(destEP));
        destEP.sin_addr.s_addr = dest_addr;
        destEP.sin_port = htons(this->port);
        destEP.sin_family = AF_INET;

        int num = sendto(this->sock, data->data(), data->size(), 0, (sockaddr *)&destEP, sizeof(destEP));
        if (num < 0)
        {
            std::cout << "Failed to send on socket: " << this->sock << std::endl;
            return false;
        }

        return true;
    }
    else
    {
        std::cout << "Please setup before trying to send." << std::endl;
        return false;
    }
}

std::shared_ptr<std::vector<uint8_t>> UDPClient::recieve_data()
{
    uint8_t buf[UDP_RECV_SIZE];
    if (this->sock != -1)
    {
        struct sockaddr_in recvAddr;
        memset(&recvAddr, 0, sizeof(recvAddr));
        recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        recvAddr.sin_port = htons(this->port);
        recvAddr.sin_family = AF_INET;

        socklen_t addrLen = sizeof(recvAddr);

        int num = recvfrom(sock, buf, UDP_RECV_SIZE, 0, (sockaddr *)&recvAddr, &addrLen);

        if (num < 0)
        {
            std::cout << "Failed to recieve on socket: " << this->sock << std::endl;
            return nullptr;
        }
        else
        {
            auto rxData = std::make_shared<std::vector<uint8_t>>(buf, buf + num);
            return rxData;
        }
    }
    else
    {
        std::cout << "Please setup before trying to recieve." << std::endl;
        return nullptr;
    }
}