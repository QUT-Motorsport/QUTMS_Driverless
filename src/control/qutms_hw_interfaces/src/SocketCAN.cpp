#include "qutms_hw_interfaces/SocketCAN.hpp"

#include <errno.h>
#include <ifaddrs.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netdb.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCAN::SocketCAN() {
    this->isConnected = false;
    this->sock = -1;
}

SocketCAN::~SocketCAN() { this->deconstruct(); }

bool SocketCAN::setup(std::string interface, rclcpp::Logger logger) {
    // create socket
    if (this->sock == -1) {
        this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->sock == -1) {
            int err = errno;
            RCLCPP_ERROR(logger, "CAN - Failed to create socket: %s (errno: %d)", strerror(err), err);
            return false;
        }
    }

    // Enable CAN error frames reception
    can_err_mask_t err_mask = CAN_ERR_MASK;
    if (setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        int err = errno;
        RCLCPP_WARN(logger, "CAN - Failed to set error frame filter: %s (errno: %d)", strerror(err), err);
    }

    // get interface index
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        RCLCPP_ERROR(logger, "CAN - Failed to get interface index for %s: %s (errno: %d)", interface.c_str(),
                     strerror(err), err);
        return false;
    }

    // bind socket to interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(this->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        int err = errno;
        RCLCPP_ERROR(logger, "CAN - Failed to bind interface %s to socket: %s (errno: %d)", interface.c_str(),
                     strerror(err), err);
        return false;
    }

    this->isConnected = true;
    RCLCPP_INFO(logger, "CAN - Successfully connected and bound to %s", interface.c_str());
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
            int err = errno;
            RCLCPP_WARN(logger, "CAN - Failed TX: %s (errno: %d)", strerror(err), err);
        }
    }
}

std::shared_ptr<std::vector<driverless_msgs::msg::Can>> SocketCAN::rx(rclcpp::Logger logger,
                                                                      rclcpp::Clock::SharedPtr clock) {
    auto msgs = std::make_shared<std::vector<driverless_msgs::msg::Can>>();

    if (this->isConnected) {
        // use DONTWAIT flag to make this non blocking
        ssize_t rxLen = recv(this->sock, this->rxBuf, SCAN_RECV_SIZE, MSG_DONTWAIT);
        RCLCPP_DEBUG_THROTTLE(logger, *clock, 500, "CAN - RX %ld bytes", rxLen);

        if (rxLen > 0) {
            size_t len = static_cast<size_t>(rxLen);
            for (size_t offset = 0; offset < len; offset += sizeof(struct can_frame)) {
                if ((offset + sizeof(struct can_frame)) <= len) {
                    struct can_frame *frame = (struct can_frame *)&(this->rxBuf[offset]);

                    // Check for CAN error frames
                    if (frame->can_id & CAN_ERR_FLAG) {
                        uint32_t err_class = frame->can_id & CAN_ERR_MASK;
                        if (err_class & CAN_ERR_BUSOFF) {
                            RCLCPP_ERROR(logger, "CAN Error: Bus-Off event detected! Interface state is BUS-OFF.");
                        }
                        if (err_class & CAN_ERR_ACK) {
                            RCLCPP_ERROR(logger,
                                         "CAN Error: No ACK received (Acknowledge Error). Check cable connection & "
                                         "motor power.");
                        }
                        if (err_class & CAN_ERR_CRTL) {
                            RCLCPP_WARN(logger, "CAN Error: Controller error (status: 0x%02X)", frame->data[1]);
                        }
                        if (err_class & CAN_ERR_TX_TIMEOUT) {
                            RCLCPP_ERROR(logger, "CAN Error: TX Timeout!");
                        }
                        if (err_class & CAN_ERR_LOSTARB) {
                            RCLCPP_WARN(logger, "CAN Error: Lost arbitration at bit %d", frame->data[0]);
                        }
                        continue;
                    }

                    driverless_msgs::msg::Can rxMsg;
                    if (parse_socketcan_frame(frame, &rxMsg)) {
                        msgs->push_back(rxMsg);
                    }
                }
            }
        }
    }

    return msgs;
}

void SocketCAN::deconstruct() {
    if (this->sock != -1) {
        close(this->sock);
        this->isConnected = false;
        this->sock = -1;
    }
}
