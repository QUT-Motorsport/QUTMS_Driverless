#include <eigen3/Eigen/Dense>
#include <vector>

#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// x, y, orientation
#define CAR_STATE_SIZE 3
// x, y
#define LANDMARK_STATE_SIZE 2

typedef struct ConeColourCount {
    int yellow = 0;
    int blue = 0;
    int orange_big = 0;
    int orange_small = 0;
} ConeColourCount_t;

void get_state_from_mu(const Eigen::MatrixXd& mu, double& x, double& y, double& theta);
double wrap_pi(double x);
int get_cone_colour(ConeColourCount_t cone_colour_count);
int landmark_idx_to_cone_idx(int landmark_idx);
int cone_idx_to_landmark_idx(int cone_idx);

class EKFslam {
   private:
    void initalise_new_cone_colour();
    void update_cone_colour(driverless_msgs::msg::Cone cone, int associated_landmark_idx);

    std::vector<ConeColourCount_t> cone_colours;

    Eigen::MatrixXd pred_mu;   // predicted state (mean, μ bar)
    Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

    Eigen::MatrixXd mu;   // final state (mean, μ)
    Eigen::MatrixXd cov;  // final state (covariance, ∑)

    // clang-format off
    const Eigen::Matrix3d R = (
        Eigen::Matrix3d() << 0.05,   0,     0,
                             0,      0.05,  0,
                             0,      0,     0.05
    ).finished();

    // Q = ( σ_r^2  0         )
    //     ( 0      σ_theta^2 )
    const Eigen::Matrix2d Q = (
        Eigen::Matrix2d() << 4,  0,
                             0,  1
    ).finished();
    // clang-format on

   public:
    EKFslam();

    // pred_car_mu is column vector [x, y, theta]^T
    void position_predict(const Eigen::Matrix<double, CAR_STATE_SIZE, 1>& pred_car_mu,
                          const Eigen::Matrix<double, CAR_STATE_SIZE, CAR_STATE_SIZE>& pred_car_cov);
    void position_delta_predict(const double delta_robot_x, const double delta_robot_theta);
    void correct(const std::vector<driverless_msgs::msg::Cone>& detected_cones);

    const Eigen::MatrixXd& get_pred_mu() { return pred_mu; };
    const Eigen::MatrixXd& get_pred_cov() { return pred_cov; };
    const Eigen::MatrixXd& get_mu() { return mu; };
    const Eigen::MatrixXd& get_cov() { return cov; };

    void get_state(double& x, double& y, double& theta) { get_state_from_mu(this->mu, x, y, theta); };
    void get_pred_state(double& x, double& y, double& theta) { get_state_from_mu(this->pred_mu, x, y, theta); };

    std::vector<driverless_msgs::msg::Cone> get_cones();
};
