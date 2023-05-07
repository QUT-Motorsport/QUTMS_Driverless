#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>

#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/debug_msg.hpp"
#include "driverless_msgs/msg/double_matrix.hpp"
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
    std::vector<ConeColourCount_t> cone_colours;
    void initalise_new_cone_colour();
    void update_cone_colour(driverless_msgs::msg::Cone cone, int associated_landmark_idx);

    std::map<int, int> cone_sim_indexes;
    void initalise_new_cone_sim_idx(int sim_index, int lm_index);
    std::optional<int> find_associated_cone_idx_from_sim_idx(int sim_index);

    Eigen::MatrixXd pred_mu;   // predicted state (mean, μ bar)
    Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

    Eigen::MatrixXd mu;   // final state (mean, μ)
    Eigen::MatrixXd cov;  // final state (covariance, ∑)

    Eigen::Matrix3d motion_uncertanty(double dt, double theta, double rotational_vel, double forward_vel,
                                      double time_weight, double rotation_weight, double forward_weight,
                                      double heading_time_weight) {
        Eigen::Matrix3d uncertanty = Eigen::Matrix3d::Zero();
        // uncertanty(0, 0) = time_weight * dt + rotation_weight * abs(rotational_vel) * dt +
        //                    forward_weight * abs(cos(theta) * forward_vel);
        // uncertanty(1, 1) = time_weight * dt + rotation_weight * abs(rotational_vel) * dt +
        //                    forward_weight * abs(sin(theta) * forward_vel);
        // uncertanty(2, 2) = heading_time_weight * dt;

        uncertanty(0, 0) = time_weight * dt * abs(cos(theta));
        uncertanty(1, 1) = time_weight * dt * abs(sin(theta));
        uncertanty(2, 2) = heading_time_weight * dt;
        return uncertanty;
    }

   public:
    EKFslam();

    void predict(double forward_vel, double rotational_vel, double dt, double uncertanty_time_weight,
                 double uncertanty_rotation_weight, double uncertanty_forward_weight,
                 double uncertanty_heading_time_weight);
    void update(const std::vector<driverless_msgs::msg::Cone>& detected_cones, double range_variance,
                double bearing_variance, double association_dist_threshold,
                std::optional<const rclcpp::Logger> logger = {});

    const Eigen::MatrixXd& get_pred_mu() { return pred_mu; };
    const Eigen::MatrixXd& get_pred_cov() { return pred_cov; };
    const Eigen::MatrixXd& get_mu() { return mu; };
    const Eigen::MatrixXd& get_cov() { return cov; };

    void get_state(double& x, double& y, double& theta) { get_state_from_mu(this->mu, x, y, theta); };
    void get_pred_state(double& x, double& y, double& theta) { get_state_from_mu(this->pred_mu, x, y, theta); };

    std::vector<driverless_msgs::msg::ConeWithCovariance> get_cones();
};
