#pragma once

#include <rclcpp_lifecycle/state.hpp>
#include "pure_pursuit.hpp"

class PurePursuitNode : public rclcpp::Node {
private:
    std::unique_ptr<PurePursuit> pp;
    void state_callback(driverless_msgs::msg::State const &msg);
public:
    PurePursuitNode();
    void initialise();
};
