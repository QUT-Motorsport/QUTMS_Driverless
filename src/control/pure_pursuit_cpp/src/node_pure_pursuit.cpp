#include <node_pure_pursuit.hpp>

void PurePursuitNode::state_callback(driverless_msgs::msg::State const &msg) {
    if (msg.state == driverless_msgs::msg::State::DRIVING) {
        pp->driving = true;
        RCLCPP_INFO_ONCE(get_logger(), "Ready to drive");
    }
    if (msg.lap_count == 0) {
        pp->following = false;
        pp->vel_max = this->get_parameter("discovery_vel_max").as_double();
        pp->vel_min = this->get_parameter("discovery_vel_min").as_double();
        pp->lookahead = this->get_parameter("discovery_lookahead").as_double();
    }
    if (msg.lap_count > 0 && !pp->following) {
        pp->following = true;
        pp->vel_max = this->get_parameter("vel_max").as_double();
        pp->vel_min = this->get_parameter("vel_min").as_double();
        pp->lookahead = this->get_parameter("lookahead").as_double();
        RCLCPP_INFO(get_logger(), "Discovery lap completed, commencing following");
    }
}

// Can't call shared_from_this() is a constructor :(
void PurePursuitNode::initialise() {
    auto buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    pp = std::make_unique<PurePursuit>(shared_from_this(), buffer,
                                       std::make_shared<tf2_ros::TransformListener>(*buffer));

    pp->path_sub = this->create_subscription<nav_msgs::msg::Path>("/planning/midline_path", QOS_LATEST,
                                                                  std::bind(&PurePursuit::path_callback, pp.get(), _1));
    pp->state_sub = this->create_subscription<driverless_msgs::msg::State>(
        "/system/as_status", QOS_LATEST, std::bind(&PurePursuitNode::state_callback, this, _1));
    pp->timer =
        this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PurePursuit::timer_callback, pp.get()));
    pp->driving_command_pub =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);
    pp->rvwp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/control/rvwp", 10);

    RCLCPP_INFO(get_logger(), "---Pure Pursuit (C++) Node Initialised---");
}

PurePursuitNode::PurePursuitNode() : Node("pure_pursuit_cpp_node") {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    node->initialise();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
