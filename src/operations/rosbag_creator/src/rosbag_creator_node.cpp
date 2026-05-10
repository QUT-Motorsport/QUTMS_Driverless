#include "rclcpp/rclcpp.hpp"
#include "rosbag_creator/rosbag2_recorder_component.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rosbag_creator::Rosbag2RecorderComponent>(options);
    RCLCPP_INFO(node->get_logger(), "rosbag_creator_node (standalone) started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
