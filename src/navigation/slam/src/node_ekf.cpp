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

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


// USEFUL LINKS:
// https://jihongju.github.io/2018/10/05/slam-05-ekf-slam/
// https://www.youtube.com/watch?v=X30sEgIws0g
// https://github.com/MURDriverless/slam/

// HOW TO MAKE MESSAGE FILTERS BUILD:
// // This is here to make message_filters build (https://stackoverflow.com/a/30851225)
// #define __STDC_FORMAT_MACROS
// #include <inttypes.h>

// #include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"
// #include "message_filters/synchronizer.h"

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped> approximate_policy;



// xdot, ydot, x, y, orientation
#define CAR_STATE_SIZE 5
// x, y
#define LANDMARK_STATE_SIZE 2


void get_state(const Eigen::MatrixXd& mu, double& x, double& y, double& theta) {
    x = mu(2, 0);
    y = mu(3, 0);
    theta = mu(4, 0);
}

void get_full_state(
    const Eigen::MatrixXd& mu,
    double& xdot,
    double& ydot,
    double& x,
    double& y,
    double& theta
) {
    xdot = mu(0, 0);
    ydot = mu(1, 0);
    x = mu(2, 0);
    y = mu(3, 0);
    theta = mu(4, 0);
}


double compute_dt(builtin_interfaces::msg::Time start_, builtin_interfaces::msg::Time end_) {
    uint32_t sec_dt = end_.sec - start_.sec;
    uint32_t nsec_dt = end_.nanosec - start_.nanosec;

    return (double)sec_dt + (double)nsec_dt/1e9;
}


void compute_motion_model(
    double dt,
    double forward_accel,
    double theta_dot,
    const Eigen::MatrixXd& mu,
    Eigen::MatrixXd& pred_mu_out,
    Eigen::MatrixXd& jacobian_out  // G_x
) {
    // this function is g()

    double xdot, ydot, x, y, theta;
    get_full_state(mu, xdot, ydot, x, y, theta);

    double dt2 = pow(dt, 2);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // predict new xdot, ydot, x, y, theta position
    pred_mu_out(0, 0) = xdot + forward_accel*dt*cos_theta;  // xdot'
    pred_mu_out(1, 0) = ydot + forward_accel*dt*sin_theta;  // ydot'
    pred_mu_out(2, 0) = x + 0.5*forward_accel*dt2*cos_theta + xdot*dt;  // x'
    pred_mu_out(3, 0) = y + 0.5*forward_accel*dt2*sin_theta + ydot*dt;  // y'
    pred_mu_out(4, 0) = theta + theta_dot*dt;           // theta'

    // compute jacobian for the robot state (G_x)
    jacobian_out = Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);
    jacobian_out << 1,  0,  0, 0, -forward_accel*dt*sin_theta,
                    0,  1,  0, 0, forward_accel*dt*cos_theta,
                    dt, 0,  1, 0, -0.5*forward_accel*dt2*sin_theta,
                    0,  dt, 0, 1, -0.5*forward_accel*dt2*cos_theta,
                    0,  0,  0, 0,               1;
}

void update_pred_motion_cov(
    const Eigen::MatrixXd& motion_jacobian,  // G_x
    const Eigen::MatrixXd& cov,
    Eigen::MatrixXd& pred_cov_out
) {
    // G = ( G_x  0 )
    //     ( 0    I )
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(cov.rows(), cov.cols());
    G.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = motion_jacobian;

    pred_cov_out = G * cov * G.transpose();
}

std::optional<int> find_associated_landmark_idx(
    const Eigen::MatrixXd& mu,
    double search_x,
    double search_y
) {
    // data association, uses lowest euclidian distance, within a threshold

    double min_distance = 0.3;  // m, threshold
    std::optional<int> idx = {};

    for(int i=CAR_STATE_SIZE; i < mu.rows(); i += LANDMARK_STATE_SIZE) {
        // iterate through the state, in intervals of two, since x and y
        // for each landmark are stored in the state vector
        double i_x = mu(i, 0);
        double i_y = mu(i+1, 0);
        
        double distance = sqrt(pow(search_x - i_x, 2) + pow(search_y - i_y, 2));
        if(distance < min_distance) {
            min_distance = distance;
            idx = i;
        }
    }

    return idx;
}

void compute_expected_z(
    const Eigen::MatrixXd& mu,
    int landmark_idx,
    Eigen::MatrixXd& expected_z_out,
    Eigen::MatrixXd& observation_jacobian
) {
    // this function is h()

    double x, y, theta;
    get_state(mu, x, y, theta);

    double lm_x = mu(landmark_idx);
    double lm_y = mu(landmark_idx+1);

    double dx = lm_x - x;
    double dy = lm_y - y;

    double q = pow(dx, 2) + pow(dy, 2);

    // z = (  range  )
    //     ( bearing )
    expected_z_out(0, 0) = sqrt(q);
    expected_z_out(1, 0) = atan2(dy, dx) - theta;

    Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(CAR_STATE_SIZE + LANDMARK_STATE_SIZE, mu.rows());
    Fx.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);
    Fx(CAR_STATE_SIZE, landmark_idx) = 1;
    Fx(CAR_STATE_SIZE + 1, landmark_idx + 1) = 1;

    Eigen::MatrixXd low_dim_jacobian(2, CAR_STATE_SIZE + LANDMARK_STATE_SIZE);

    low_dim_jacobian(0, 0) = 1/q * -sqrt(q) * dx;
    low_dim_jacobian(0, 1) = 1/q * -sqrt(q) * dy;
    low_dim_jacobian(0, 2) = 0;  // 1/q * 0
    low_dim_jacobian(0, 3) = 1/q * sqrt(q) * dx;
    low_dim_jacobian(0, 4) = 1/q * sqrt(q) * dy;

    low_dim_jacobian(1, 0) = 1/q * dy;
    low_dim_jacobian(1, 1) = 1/q * -dx;
    low_dim_jacobian(1, 2) = -1;  // 1/q * -q
    low_dim_jacobian(1, 3) = 1/q * -dy;
    low_dim_jacobian(1, 4) = 1/q * dx;

    observation_jacobian = low_dim_jacobian * Fx;
}


class EKFNode : public rclcpp::Node {
    private:
        Eigen::MatrixXd pred_mu;	 // predicted state (mean, μ bar)
        Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

        Eigen::MatrixXd mu;  // final state (mean, μ)
        Eigen::MatrixXd cov;  // final state (covariance, ∑)

        std::optional<builtin_interfaces::msg::Time> last_sensed_control_update;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    public:
        EKFNode() : Node("ekf_node")
        {
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, std::bind(&EKFNode::sensed_control_callback, this, _1)
            );

            // initalise state and convariance with just the car state
            // (landmarks will be added when they are detected)
            pred_mu = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, 1);
            pred_cov = 0.1 * Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);

            mu = pred_mu;
            cov = pred_cov;
        }

        // public for testing
        void control_callback(ackermann_msgs::msg::AckermannDrive msg) {
            // TODO
            (void) msg;
        }

        void sensed_control_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
            // catch first call where we have no "last update time"
            if(!this->last_sensed_control_update.has_value()) {
                this->last_sensed_control_update = imu_msg->header.stamp;
                return;
            }

            std::cout << "U: \n" << imu_msg->linear_acceleration.x << " " << imu_msg->angular_velocity.z << "\n" << std::endl;

            double dt = compute_dt(last_sensed_control_update.value(), imu_msg->header.stamp);
            this->last_sensed_control_update = imu_msg->header.stamp;

            // u vector doesnt really need to be constructed, but the concept lives here as:
            // [imu_msg->linear_acceleration.x, imu_msg->angular_velocity.z]
            Eigen::MatrixXd motion_jacobian = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, CAR_STATE_SIZE);  // G_x
            compute_motion_model(
                dt,
                imu_msg->linear_acceleration.x,
                imu_msg->angular_velocity.z,
                this->pred_mu,
                this->pred_mu,
                motion_jacobian
            );
            std::cout << "motion jacobian:\n" << motion_jacobian << "\n" << std::endl;
            update_pred_motion_cov(motion_jacobian, this->pred_cov, this->pred_cov);

            this->print_matricies();
        }

        void cone_detection_callback(driverless_msgs::msg::ConeDetectionStamped msg) {
            // Q = ( σ_r^2  0         )
            //     ( 0      σ_theta^2 )
            Eigen::Matrix2d Q;
            Q << 10,   0,
                 0 ,  10;

            double x, y, theta;
            get_state(this->pred_mu, x, y, theta);

            for(driverless_msgs::msg::Cone cone : msg.cones) {
                // landmark (cone) position in global frame
                double glob_lm_x = x + cone.location.x * cos(theta) - cone.location.y * sin(theta);
                double glob_lm_y = y + cone.location.x * sin(theta) + cone.location.y * cos(theta);

                std::optional<int> associated_idx = find_associated_landmark_idx(this->pred_mu, glob_lm_x, glob_lm_y);

                if(!associated_idx.has_value()) {
                    // new landmark
                    this->pred_mu.conservativeResize(this->pred_mu.rows() + 2, Eigen::NoChange);
                    this->pred_cov.conservativeResize(this->pred_cov.rows() + 2, pred_cov.cols() + 2);

                    int new_lm_idx = this->pred_mu.rows() - 2;

                    this->pred_mu(new_lm_idx, 0) = glob_lm_x;
                    this->pred_mu(new_lm_idx + 1, 0) = glob_lm_y;

                    this->pred_cov(new_lm_idx, new_lm_idx) = 0.5;
                    this->pred_cov(new_lm_idx+1, new_lm_idx+1) = 0.5;

                    associated_idx = new_lm_idx;
                }

                // z = (  range  )
                //     ( bearing )
                Eigen::MatrixXd z(2, 1);
                z(0, 0) = sqrt(pow(cone.location.x, 2) + pow(cone.location.y, 2));
                z(1, 0) = atan2(cone.location.y, cone.location.x);

                Eigen::MatrixXd expected_z(2, 1);
                Eigen::MatrixXd observation_jacobian = Eigen::MatrixXd::Zero(cov.rows(), cov.cols());  // H
                compute_expected_z(this->pred_mu, associated_idx.value(), expected_z, observation_jacobian);
                std::cout << "H:\n" << observation_jacobian << "\n" << std::endl;


                Eigen::MatrixXd K = this->pred_cov*observation_jacobian.transpose()*((observation_jacobian*this->pred_cov*observation_jacobian.transpose() + Q).inverse());
                std::cout << "K:\n" << K << "\n" << std::endl;

                this->pred_mu = this->pred_mu + K*(z - expected_z);
                this->pred_cov = (Eigen::MatrixXd::Identity(K.rows(), observation_jacobian.cols()) - K*observation_jacobian) * this->pred_cov;
            }

            this->mu = this->pred_mu;
            this->cov = this->pred_cov;
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

    auto ekf_node = std::make_shared<EKFNode>();
    // rclcpp::spin(ekf_node);
    // rclcpp::shutdown();

    std::cout << "\n ---------- Inital. \n" << std::endl;
    ekf_node->print_matricies();

    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    std::cout << "\n ---------- 1. \n" << std::endl;
    imu_msg->header.stamp.sec = 1;
    ekf_node->sensed_control_callback(imu_msg);

    std::cout << "\n ---------- 2. \n" << std::endl;
    imu_msg->header.stamp.sec = 2;
    imu_msg->linear_acceleration.x = 0.5;  // m/s/s
    imu_msg->angular_velocity.z = 0;  // rad/s
    ekf_node->sensed_control_callback(imu_msg);

    std::cout << "\n ---------- 3. \n" << std::endl;
    imu_msg->header.stamp.sec = 3;
    imu_msg->linear_acceleration.x = 0;  // m/s/s
    imu_msg->angular_velocity.z = 0;  // rad/s
    ekf_node->sensed_control_callback(imu_msg);

    std::cout << "\n ---------- 4. \n" << std::endl;
    imu_msg->header.stamp.sec = 4;
    imu_msg->linear_acceleration.x = 0;  // m/s/s
    imu_msg->angular_velocity.z = 0;  // rad/s
    ekf_node->sensed_control_callback(imu_msg);

    std::cout << "\n ---------- Measurement \n" << std::endl;
    driverless_msgs::msg::ConeDetectionStamped msg;
    
    driverless_msgs::msg::Cone cone1;
    cone1.location.x = 2;
    cone1.location.y = 2;
    cone1.color = driverless_msgs::msg::Cone::BLUE;

    msg.cones = {cone1};

    ekf_node->cone_detection_callback(msg);
    ekf_node->print_matricies();

	return 0;
}
