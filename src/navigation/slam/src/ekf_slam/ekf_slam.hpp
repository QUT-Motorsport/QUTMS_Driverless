#include <eigen3/Eigen/Dense>
#include <vector>

#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// x, y, orientation
#define CAR_STATE_SIZE 3
// x, y
#define LANDMARK_STATE_SIZE 2

void get_state_from_mu(const Eigen::MatrixXd& mu, double& x, double& y, double& theta);

class EKFslam {
   private:
    Eigen::MatrixXd pred_mu;   // predicted state (mean, μ bar)
    Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

    Eigen::MatrixXd mu;   // final state (mean, μ)
    Eigen::MatrixXd cov;  // final state (covariance, ∑)

    // clang-format off
    const Eigen::Matrix3d R = (
        Eigen::Matrix3d() << 0.01,   0,     0,
                             0,      0.01,  0,
                             0,      0,     0.01
    ).finished();

    // Q = ( σ_r^2  0         )
    //     ( 0      σ_theta^2 )
    const Eigen::Matrix2d Q = (
        Eigen::Matrix2d() << 1,  0,
                             0,  1
    ).finished();
    // clang-format on

   public:
    EKFslam();

    // pred_car_mu is column vector [x, y, theta]^T
    void position_predict(const Eigen::Matrix<double, CAR_STATE_SIZE, 1>& pred_car_mu,
                          const Eigen::Matrix<double, CAR_STATE_SIZE, CAR_STATE_SIZE>& pred_car_cov);
    void correct(const std::vector<driverless_msgs::msg::Cone>& detected_cones);

    const Eigen::MatrixXd& get_pred_mu() { return pred_mu; };
    const Eigen::MatrixXd& get_pred_cov() { return pred_cov; };
    const Eigen::MatrixXd& get_mu() { return mu; };
    const Eigen::MatrixXd& get_cov() { return cov; };

    void get_state(double& x, double& y, double& theta) { get_state_from_mu(this->mu, x, y, theta); };
    void get_pred_state(double& x, double& y, double& theta) { get_state_from_mu(this->pred_mu, x, y, theta); };
};
