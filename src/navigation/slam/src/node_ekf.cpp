#include <eigen3/Eigen/Dense>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "rclcpp/rclcpp.hpp"

class EKFNode : public rclcpp::Node {
    private:
        Eigen::MatrixXf x;	// state
        Eigen::MatrixXf u;  // control commands

        void control_callback(ackermann_msgs::msg::AckermannDrive msg) {
            // TODO
        }
    public:
        EKFNode() : Node("ekf_node") {

        }
};

int main(int argc, char ** argv) {
	(void) argc;
	(void) argv;

	return 0;
}
