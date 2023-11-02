#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"

const int SCAN_RECV_SIZE = 4096;

class SocketCAN {
   private:
    bool isConnected;
    int sock;
    uint8_t rxBuf[SCAN_RECV_SIZE];

   public:
    SocketCAN();

    bool setup(std::string interface, rclcpp::Logger logger);

    void tx(driverless_msgs::msg::Can *msg, rclcpp::Logger logger);
    std::shared_ptr<std::vector<driverless_msgs::msg::Can>> rx(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

    void compose_socketcan_frame(driverless_msgs::msg::Can *msg, struct can_frame *frame);
    bool parse_socketcan_frame(struct can_frame *frame, driverless_msgs::msg::Can *msg);

    ~SocketCAN();
};
