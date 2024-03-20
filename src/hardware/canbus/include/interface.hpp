#pragma once

#include <memory>
#include <string>
#include <vector>

#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"

class CANInterface {
   public:
    CANInterface(){};
    virtual bool setup(std::string interface, rclcpp::Logger logger) = 0;

    virtual void tx(driverless_msgs::msg::Can *msg, rclcpp::Logger logger) = 0;
    virtual std::shared_ptr<std::vector<driverless_msgs::msg::Can>> rx(rclcpp::Logger logger,
                                                                       rclcpp::Clock::SharedPtr clock) = 0;

    virtual void deconstruct() = 0;
};
