#include "component_velocity_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<velocity_controller::VelocityController>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
