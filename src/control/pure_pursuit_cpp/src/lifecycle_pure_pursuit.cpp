#include <lifecycle_pure_pursuit.hpp>

PurePursuitLifecycle::PurePursuitLifecycle() :
        rclcpp_lifecycle::LifecycleNode("pure_pursuit_cpp_node", rclcpp::NodeOptions().use_intra_process_comms(false)) {
}

// Can't call shared_from_this() is a constructor :(
void PurePursuitLifecycle::initialise() {
    auto buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    pp = std::make_unique<PurePursuit>(shared_from_this(), buffer,
                                       std::make_shared<tf2_ros::TransformListener>(*buffer));
    RCLCPP_INFO(get_logger(), "---Pure Pursuit (C++) Node Initialised---");
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_configure(rclcpp_lifecycle::State const &) {
    pp->driving_command_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);

    RCLCPP_INFO(get_logger(), "On configure");
    return CallbackReturn::SUCCESS;
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_activate(rclcpp_lifecycle::State const &) {
    pp->path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/midline_path", QOS_LATEST, std::bind(&PurePursuit::path_callback, pp.get(), _1));
    pp->timer =
            this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PurePursuit::timer_callback, pp.get()));
    pp->driving_command_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);
    RCLCPP_INFO(get_logger(), "On activate");
    return CallbackReturn::SUCCESS;
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_deactivate(rclcpp_lifecycle::State const &) {
    if (pp->path_sub) pp->path_sub.reset();
    if (pp->timer) pp->timer->reset();
    if (pp->driving_command_pub) pp->driving_command_pub.reset();
    RCLCPP_INFO(get_logger(), "On deactivate");
    return CallbackReturn::SUCCESS;
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_cleanup(rclcpp_lifecycle::State const &) {
    if (pp->path_sub) pp->path_sub.reset();
    if (pp->timer) pp->timer->reset();
    if (pp->driving_command_pub) pp->driving_command_pub.reset();
    RCLCPP_INFO(get_logger(), "On cleanup");
    return CallbackReturn::SUCCESS;
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_shutdown(rclcpp_lifecycle::State const &) {
    if (pp->path_sub) pp->path_sub.reset();
    if (pp->timer) pp->timer->reset();
    if (pp->driving_command_pub) pp->driving_command_pub.reset();
    RCLCPP_INFO(get_logger(), "On shutdown");
    return CallbackReturn::SUCCESS;
}

PurePursuitLifecycle::CallbackReturn PurePursuitLifecycle::on_error(rclcpp_lifecycle::State const &) {
    RCLCPP_INFO(get_logger(), "On error");
    return CallbackReturn::SUCCESS;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitLifecycle>();
    node->initialise();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
