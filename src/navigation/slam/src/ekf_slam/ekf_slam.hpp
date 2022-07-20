#include <eigen3/Eigen/Dense>
#include <vector>

#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "utils.hpp"

// x, y, orientation
#define CAR_STATE_SIZE 3
// x, y
#define LANDMARK_STATE_SIZE 2

class EKFslam {
   private:
    Eigen::MatrixXd pred_mu;   // predicted state (mean, μ bar)
    Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

    Eigen::MatrixXd mu;   // final state (mean, μ)
    Eigen::MatrixXd cov;  // final state (covariance, ∑)

    // clang-format off
        const Eigen::Matrix3d R = (
            Eigen::Matrix3d() << 0.01,  0,     0,
                                 0,     0.01,  0,
                                 0,     0,     0.01
        ).finished();

        // Q = ( σ_r^2  0         )
        //     ( 0      σ_theta^2 )
        const Eigen::Matrix2d Q = (
            Eigen::Matrix2d() << 100,  0,
                                 0,    100
        ).finished();
    // clang-format on

   public:
    EKFslam();
    // pred_car_mu is column vector [x, y, theta]^T
    void position_predict(const Eigen::MatrixXd& pred_car_mu, const Eigen::MatrixXd& pred_car_cov);
    void correct(const std::vector<driverless_msgs::msg::Cone>& detected_cones);

    const Eigen::MatrixXd& pred_mu() { return pred_mu; };
    const Eigen::MatrixXd& pred_cov() { return pred_cov; };
    const Eigen::MatrixXd& mu() { return mu; };
    const Eigen::MatrixXd& cov() { return cov; };
};
