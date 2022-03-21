#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>
#include <memory>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/cone.hpp"

// This is here to make message_filters build (https://stackoverflow.com/a/30851225)
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "rclcpp/rclcpp.hpp"


// x, y, orientation
#define CAR_STATE_SIZE 3


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
    jacobian_out = Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);
    jacobian_out(0, 2) = -forward_vel*dt*sin(theta);
    jacobian_out(1, 2) = forward_vel*dt*cos(theta);
    jacobian_out(2, 2) = 2;
}

void update_pred_motion_cov(
    int state_size,
    const Eigen::MatrixXd& motion_jacobian,  // G_x
    const Eigen::MatrixXd& cov,
    Eigen::MatrixXd& pred_cov_out
) {
    // G = ( G_x  0 )
    //     ( 0    I )
    // TODO: INITAL_STATE_SIZE needs to be dynamic current state size here
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(state_size, state_size);
    G.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = motion_jacobian;

    pred_cov_out = G * cov * G.transpose();
}


class EKFNode : public rclcpp::Node {
    private:
        Eigen::MatrixXd pred_mu;	 // predicted state (mean, μ)
        Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑)

        Eigen::MatrixXd mu;  // final state (mean, μ)
        Eigen::MatrixXd cov;  // final state (covariance, ∑)

        int state_size = CAR_STATE_SIZE;

        std::optional<builtin_interfaces::msg::Time> last_sensed_control_update;

    public:
        EKFNode() : Node("ekf_node") {

            // set up approximate time sycronised imu and gss subscribers
            message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub(this, "imu");
            message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vel_sub(this, "gss");
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped> approximate_policy;
            message_filters::Synchronizer<approximate_policy>syncApproximate(approximate_policy(10), imu_sub, vel_sub);
            syncApproximate.registerCallback(&EKFNode::sensed_control_callback, this);

            pred_mu = Eigen::MatrixXd::Zero(this->state_size, 1);
            pred_cov = 0.1 * Eigen::MatrixXd::Identity(this->state_size, this->state_size);

            mu = pred_mu;
            cov = pred_cov;
        }

        // public for testing
        void control_callback(ackermann_msgs::msg::AckermannDrive msg) {
            // TODO
            (void) msg;
        }

        void sensed_control_callback(
            const sensor_msgs::msg::Imu::SharedPtr imu_msg,
            const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg
        ) {
            // catch first call where we have no "last update time"
            if(!this->last_sensed_control_update.has_value()) {
                this->last_sensed_control_update = imu_msg->header.stamp;
                return;
            }

            double dt = compute_dt(last_sensed_control_update.value(), imu_msg->header.stamp);
            this->last_sensed_control_update = imu_msg->header.stamp;

            // u vector doesnt really need to be constructed, but the concept lives here as:
            // [vel_msg->twist.linear.x, imu_msg->angular_velocity.z]
            Eigen::MatrixXd motion_jacobian = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, CAR_STATE_SIZE);  // G_x
            motion_model(
                dt,
                vel_msg->twist.linear.x,
                imu_msg->angular_velocity.z,
                this->pred_mu,
                this->pred_mu,
                motion_jacobian
            );
            std::cout << "motion jacobian:\n" << motion_jacobian << "\n" << std::endl;
            update_pred_motion_cov(this->state_size, motion_jacobian, this->pred_cov, this->pred_cov);
        }

        void cone_detection_callback(driverless_msgs::msg::ConeDetectionStamped msg) {
            // TODO
            (void) msg;
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

    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    geometry_msgs::msg::TwistStamped::SharedPtr vel_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    std::cout << "\n ---------- 1. \n" << std::endl;
    imu_msg->header.stamp.sec = 1;
    vel_msg->header.stamp.sec = 1;
    ekf_node.sensed_control_callback(imu_msg, vel_msg);
    ekf_node.print_matricies();

    std::cout << "\n ---------- 2. \n" << std::endl;
    imu_msg->header.stamp.sec = 2;
    imu_msg->angular_velocity.z = 0.5;  // rad/s
    vel_msg->header.stamp.sec = 2;
    vel_msg->twist.linear.x = 2;  // m/s
    ekf_node.sensed_control_callback(imu_msg, vel_msg);
    ekf_node.print_matricies();

    std::cout << "\n ---------- 3. \n" << std::endl;
    imu_msg->header.stamp.sec=3;
    vel_msg->header.stamp.sec=3;
    ekf_node.sensed_control_callback(imu_msg, vel_msg);
    ekf_node.print_matricies();

    std::cout << "\n ---------- 4. \n" << std::endl;
    imu_msg->header.stamp.sec=4;
    vel_msg->header.stamp.sec=4;
    ekf_node.sensed_control_callback(imu_msg, vel_msg);
    ekf_node.print_matricies();

	return 0;
}
