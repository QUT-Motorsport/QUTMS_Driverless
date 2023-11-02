#include "SocketCAN.hpp"

#include <ifaddrs.h>
#include <linux/can.h>
#include <net/if.h>
#include <netdb.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

SocketCAN::SocketCAN() {
    this->isConnected = false;
    this->sock = -1;
}

bool SocketCAN::setup(std::string interface, rclcpp::Logger logger) {
    // create socket
    if (this->sock == -1) {
        this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->sock == -1) {
            // std::cout << "CAN - Failed to create socket." << std::endl;
            RCLCPP_ERROR(logger, "CAN - Failed to create socket.");
            return false;
        }
    }

    // get interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface.c_str());
    ioctl(this->sock, SIOCGIFINDEX, &ifr);

    // bind socket to interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(this->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        // std::cout << "CAN - Failed to bind interface to socket." << std::endl;
        RCLCPP_ERROR(logger, "CAN - Failed to bind interface to socket.");
        return false;
    }

    this->isConnected = true;
    return true;
}

void SocketCAN::compose_socketcan_frame(driverless_msgs::msg::Can *msg, struct can_frame *frame) {
    frame->can_id = msg->id;
    if (msg->id_type) {
        // set extended bit
        frame->can_id |= CAN_EFF_FLAG;
    }

    frame->can_dlc = msg->dlc;
    for (uint8_t i = 0; i < msg->dlc; i++) {
        frame->data[i] = msg->data.data()[i];
    }
}

bool SocketCAN::parse_socketcan_frame(struct can_frame *frame, driverless_msgs::msg::Can *msg) {
    msg->id_type = (frame->can_id & CAN_EFF_FLAG) != 0;

    if (msg->id_type != 0) {
        msg->id = frame->can_id & CAN_EFF_MASK;
    } else {
        msg->id = frame->can_id & CAN_SFF_MASK;
    }

    msg->dlc = frame->can_dlc;
    std::vector<uint8_t> msgData;
    for (uint8_t i = 0; i < msg->dlc; i++) {
        msgData.push_back(frame->data[i]);
    }
    msg->data = msgData;

    return true;
}

void SocketCAN::tx(driverless_msgs::msg::Can *msg, rclcpp::Logger logger) {
    if (this->isConnected) {
        struct can_frame frame;
        compose_socketcan_frame(msg, &frame);

        if (write(this->sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            // std::cout << "CAN - Failed TX." << std::endl;
            RCLCPP_WARN(logger, "CAN - Failed TX.");
        }
    }
}

std::shared_ptr<std::vector<driverless_msgs::msg::Can>> SocketCAN::rx(rclcpp::Logger logger,
                                                                      rclcpp::Clock::SharedPtr clock) {
    auto msgs = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can rxMsg;

    if (this->isConnected) {
        // only need to try Rx if we're connected
        // this function polls the socket for data, but will block if there's no data to recieve
        // can't know ahead of time how many messages to read, so will just read to oversized buffer

        // use DONTWAIT flag to make this non blocking
        ssize_t rxLen = recv(this->sock, this->rxBuf, SCAN_RECV_SIZE, MSG_DONTWAIT);
        RCLCPP_DEBUG_THROTTLE(logger, *clock, 500, "CAN - RX %ld bytes", rxLen);

        if (rxLen > 0) {
            size_t len = rxLen;
            for (size_t offset = 0; offset < len; offset += sizeof(struct can_frame)) {
                if ((offset + sizeof(struct can_frame)) <= len) {
                    // there is a full frame here, read it

                    // convert appropriate bytes from socket into a can_frame
                    struct can_frame *frame = (struct can_frame *)&(this->rxBuf[offset]);
                    if (parse_socketcan_frame(frame, &rxMsg)) {
                        msgs->push_back(rxMsg);
                    }
                }
            }
        }
    }

    return msgs;
}

SocketCAN::~SocketCAN() {
    if (this->sock != -1) {
        close(this->sock);
        this->isConnected = false;
        this->sock = -1;
    }
}
