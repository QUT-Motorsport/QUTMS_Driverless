#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "rclcpp/rclcpp.hpp"


// x, y, orientation (and 0 landmarks)
#define INITAL_STATE_SIZE 3


double compute_dt(builtin_interfaces::msg::Time start_, builtin_interfaces::msg::Time end_) {
    uint32_t sec_dt = end_.sec - start_.sec;
    uint32_t nsec_dt = end_.nanosec - start_.nanosec;

    return (double)sec_dt + (double)nsec_dt/1e9;
}


void motion_model(
    double dt,
    double forward_vel,
    double theta_dot,
    const Eigen::MatrixXd& mu,
    Eigen::MatrixXd& pred_mu_out,
    Eigen::MatrixXd& jacobian_out  // G_x
) {
    // this is g()
    double x = mu(0, 0);
    double y = mu(1, 0);
    double theta = mu(2, 0);

    // predict new x, y, theta position
    pred_mu_out(0, 0) = x + forward_vel*dt*cos(theta);  // x'
    pred_mu_out(1, 0) = y + forward_vel*dt*sin(theta);  // y'
    pred_mu_out(2, 0) = theta + theta_dot*dt;           // theta'

    // compute jacobian for the robot state (G_x)
    jacobian_out = Eigen::MatrixXd::Identity(INITAL_STATE_SIZE, INITAL_STATE_SIZE);
    jacobian_out(0, 2) = -forward_vel*dt*sin(theta);
    jacobian_out(1, 2) = forward_vel*dt*cos(theta);
    jacobian_out(2, 2) = 2;
}

void update_pred_motion_cov(
    const Eigen::MatrixXd& motion_jacobian,  // G_x
    const Eigen::MatrixXd& cov,
    Eigen::MatrixXd& pred_cov_out
) {
    // G = ( G_x  0 )
    //     ( 0    I )
    // TODO: INITAL_STATE_SIZE needs to be dynamic current state size here
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(INITAL_STATE_SIZE, INITAL_STATE_SIZE);
    G.topLeftCorner(INITAL_STATE_SIZE, INITAL_STATE_SIZE) = motion_jacobian;

    pred_cov_out = G * cov * G.transpose();
}


class EKFNode : public rclcpp::Node {
    private:
        Eigen::MatrixXd pred_mu;	 // predicted state (mean, μ)
        Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑)

        Eigen::MatrixXd mu;  // final state (mean, μ)
        Eigen::MatrixXd cov;  // final state (covariance, ∑)

        std::optional<builtin_interfaces::msg::Time> last_vel_update;

    public:
        EKFNode() : Node("ekf_node") {
            pred_mu = Eigen::MatrixXd::Zero(INITAL_STATE_SIZE, 1);
            pred_cov = 0.1 * Eigen::MatrixXd::Identity(INITAL_STATE_SIZE, INITAL_STATE_SIZE);

            mu = pred_mu;
            cov = pred_cov;
        }

        // public for testing
        void control_callback(ackermann_msgs::msg::AckermannDrive msg) {
            // TODO
            (void) msg;
        }

        void velocity_callback(geometry_msgs::msg::TwistStamped msg) {
            // catch first call where we have no "last update time"
            if(!this->last_vel_update.has_value()) {
                this->last_vel_update = msg.header.stamp;
                return;
            }

            double dt = compute_dt(last_vel_update.value(), msg.header.stamp);
            this->last_vel_update = msg.header.stamp;

            // u vector doesnt really need to be constructed, but the concept lives here as [msg.twist.linear.x, msg.twist.angular.z]
            Eigen::MatrixXd motion_jacobian = Eigen::MatrixXd::Zero(INITAL_STATE_SIZE, INITAL_STATE_SIZE);  // G_x
            motion_model(dt, msg.twist.linear.x, msg.twist.angular.z, this->mu, this->pred_mu, motion_jacobian);
            update_pred_motion_cov(motion_jacobian, this->cov, this->pred_cov);
        }

        void print_matricies() {
            std::cout << "pred_mu:\n" << this->pred_mu << "\n" << std::endl;
            std::cout << "pred_cov:\n" << this->pred_cov << "\n" << std::endl;
            std::cout << "mu:\n" << this->mu << "\n" << std::endl;
            std::cout << "cov:\n" << this->cov << "\n" << std::endl;
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    EKFNode ekf_node = EKFNode();

    ekf_node.print_matricies();

    builtin_interfaces::msg::Time time1;
    builtin_interfaces::msg::Time time2;

    time1.sec = 1;
    time2.sec = 3;

    geometry_msgs::msg::TwistStamped test_msg;
    test_msg.header.stamp = time1;
    ekf_node.velocity_callback(test_msg);

    test_msg.header.stamp = time2;
    test_msg.twist.linear.x = 2;  // m/s
    test_msg.twist.angular.z = 0.5;  // rad/s
    ekf_node.velocity_callback(test_msg);

    ekf_node.print_matricies();

	return 0;
}
