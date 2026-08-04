#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class SineNode : public rclcpp::Node {
   public:
    SineNode() : Node("sine_node"), count_(0.0) {
        publisher_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/steering_position_controller/commands", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&SineNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "--- Sine Controller Test Node Initialized ---");
    }

   private:
    void timer_callback() {
        count_ += 0.01;
        // -80 degrees in radians is -80 * M_PI / 180
        double angle_rad = std::sin(count_ * M_PI) * (-80.0 * M_PI / 180.0);

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.push_back(angle_rad);
        publisher_->publish(msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published steering command: %.4f rad",
                             angle_rad);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SineNode>());
    rclcpp::shutdown();
    return 0;
}
