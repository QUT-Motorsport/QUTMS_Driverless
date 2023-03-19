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
    // Odometry uncertanty
    // R = ( σ_forward_vel^2 0                )
    //     ( 0               rotational_vel^2 )
    const Eigen::Matrix2d R = (
        Eigen::Matrix2d() << pow(0.1, 2), 0,
                             0,           pow(0.01, 2)
    ).finished();

    // Observation model uncertanty
    // Q = ( σ_range^2  0         )
    //     ( 0      σ_bearing^2 )
    const Eigen::Matrix2d Q = (
        Eigen::Matrix2d() << pow(0.01, 2), 0,
                             0,            pow(0.01, 2)
    ).finished();
    // clang-format on

   public:
    EKFslam();

    void predict(double forward_vel, double rotational_vel, double dt);
    void update(const std::vector<driverless_msgs::msg::ConeWithCovariance>& detected_cones);

    const Eigen::MatrixXd& get_pred_mu() { return pred_mu; };
    const Eigen::MatrixXd& get_pred_cov() { return pred_cov; };
    const Eigen::MatrixXd& get_mu() { return mu; };
    const Eigen::MatrixXd& get_cov() { return cov; };

    void get_state(double& x, double& y, double& theta) { get_state_from_mu(this->mu, x, y, theta); };
    void get_pred_state(double& x, double& y, double& theta) { get_state_from_mu(this->pred_mu, x, y, theta); };

    std::vector<driverless_msgs::msg::Cone> get_cones();
};
