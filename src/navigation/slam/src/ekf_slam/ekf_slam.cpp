#include "ekf_slam.hpp"

void get_state_from_mu(const Eigen::MatrixXd& mu, double& x, double& y, double& theta) {
    x = mu(0, 0);
    y = mu(1, 0);
    theta = mu(2, 0);
}

double wrap_pi(double x) {
    // [-pi, pi)
    double m = fmod(x + M_PI, M_PI * 2);
    if (m < 0) {
        m += M_PI * 2;
    }
    return m - M_PI;
}

int landmark_idx_to_cone_idx(int landmark_idx) { return (landmark_idx - CAR_STATE_SIZE) / LANDMARK_STATE_SIZE; }

int cone_idx_to_landmark_idx(int cone_idx) { return CAR_STATE_SIZE + cone_idx * LANDMARK_STATE_SIZE; }

int get_cone_colour(ConeColourCount_t cone_colour_count) {
    std::vector<int> counts = {cone_colour_count.blue, cone_colour_count.yellow, cone_colour_count.orange_big,
                               cone_colour_count.orange_small};

    auto max_iter = std::max_element(counts.begin(), counts.end());
    if (*max_iter == 0) {
        // max count is 0 -> we don't know what colour the cone is
        return driverless_msgs::msg::Cone::UNKNOWN;
    }

    int max_idx = std::distance(counts.begin(), max_iter);
    switch (max_idx) {
        case 0:
            return driverless_msgs::msg::Cone::BLUE;
        case 1:
            return driverless_msgs::msg::Cone::YELLOW;
        case 2:
            return driverless_msgs::msg::Cone::ORANGE_BIG;
        case 3:
            return driverless_msgs::msg::Cone::ORANGE_SMALL;

        default:
            return driverless_msgs::msg::Cone::UNKNOWN;
    }
}

std::optional<int> find_associated_landmark_idx(const Eigen::MatrixXd& mu, double search_x, double search_y) {
    // data association, uses lowest euclidian distance, within a threshold

    double min_distance = 3 * 3;  // m, threshold^2
    std::optional<int> idx = {};

    for (int i = CAR_STATE_SIZE; i < mu.rows(); i += LANDMARK_STATE_SIZE) {
        // iterate through the state, in intervals of two, since x and y
        // for each landmark are stored in the state vector
        double i_x = mu(i, 0);
        double i_y = mu(i + 1, 0);

        double distance = (search_x - i_x) * (search_x - i_x) + (search_y - i_y) * (search_y - i_y);
        if (distance < min_distance) {
            min_distance = distance;
            idx = i;
        }
    }

    return idx;
}

int initalise_new_landmark(Eigen::MatrixXd& mu, Eigen::MatrixXd& cov, double lm_x, double lm_y,
                           const Eigen::Matrix2d& lm_jacobian, const Eigen::Matrix2d& Q) {
    mu.conservativeResize(mu.rows() + LANDMARK_STATE_SIZE, Eigen::NoChange);
    cov.conservativeResize(cov.rows() + LANDMARK_STATE_SIZE, cov.cols() + LANDMARK_STATE_SIZE);

    int new_lm_idx = mu.rows() - LANDMARK_STATE_SIZE;

    mu(new_lm_idx, 0) = lm_x;
    mu(new_lm_idx + 1, 0) = lm_y;

    // initalise the new covariance rows/cols to zero
    cov(Eigen::lastN(LANDMARK_STATE_SIZE), Eigen::all).setZero();
    cov(Eigen::all, Eigen::lastN(LANDMARK_STATE_SIZE)).setZero();

    // initalise the landmark covariances using the jacobian
    cov.bottomRightCorner(LANDMARK_STATE_SIZE, LANDMARK_STATE_SIZE) = lm_jacobian * Q * lm_jacobian.transpose();

    return new_lm_idx;
}

Eigen::Matrix2d compute_landmark_jacobian(double theta, double lm_detection_x, double lm_detection_y) {
    Eigen::Matrix2d landmark_jacobian;

    // clang-format off
    landmark_jacobian << cos(theta + lm_detection_y), lm_detection_x * -sin(theta + lm_detection_y),
                         sin(theta + lm_detection_y), lm_detection_x * cos(theta + lm_detection_y);
    // clang-format on

    return landmark_jacobian;
}

void compute_expected_z(const Eigen::MatrixXd& mu, int landmark_idx, Eigen::MatrixXd& expected_z_out,
                        Eigen::MatrixXd& observation_jacobian) {
    // this function is h()

    double x, y, theta;
    get_state_from_mu(mu, x, y, theta);

    double lm_x = mu(landmark_idx);
    double lm_y = mu(landmark_idx + 1);

    double dx = lm_x - x;
    double dy = lm_y - y;

    double q = pow(dx, 2) + pow(dy, 2);

    // z = (  range  )
    //     ( bearing )
    expected_z_out(0, 0) = sqrt(q);
    expected_z_out(1, 0) = wrap_pi(atan2(dy, dx) - theta);

    Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(CAR_STATE_SIZE + LANDMARK_STATE_SIZE, mu.rows());
    Fx.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);
    Fx(CAR_STATE_SIZE, landmark_idx) = 1;
    Fx(CAR_STATE_SIZE + 1, landmark_idx + 1) = 1;

    Eigen::MatrixXd low_dim_jacobian(2, CAR_STATE_SIZE + LANDMARK_STATE_SIZE);

    low_dim_jacobian(0, 0) = 1 / q * -sqrt(q) * dx;
    low_dim_jacobian(0, 1) = 1 / q * -sqrt(q) * dy;
    low_dim_jacobian(0, 2) = 0;  // 1/q * 0
    low_dim_jacobian(0, 3) = 1 / q * sqrt(q) * dx;
    low_dim_jacobian(0, 4) = 1 / q * sqrt(q) * dy;

    low_dim_jacobian(1, 0) = 1 / q * dy;
    low_dim_jacobian(1, 1) = 1 / q * -dx;
    low_dim_jacobian(1, 2) = -1;  // 1/q * -q
    low_dim_jacobian(1, 3) = 1 / q * -dy;
    low_dim_jacobian(1, 4) = 1 / q * dx;

    observation_jacobian = low_dim_jacobian * Fx;
}

EKFslam::EKFslam() {
    // initalise state and convariance with just the car state
    // (landmarks will be added when they are detected)
    pred_mu = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, 1);
    pred_cov = 0.1 * Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);

    mu = pred_mu;
    cov = pred_cov;
}

void EKFslam::position_predict(const Eigen::Matrix<double, CAR_STATE_SIZE, 1>& pred_car_mu,
                               const Eigen::Matrix<double, CAR_STATE_SIZE, CAR_STATE_SIZE>& pred_car_cov) {
    // since we are taking position directly, no need to run motion models
    // or compute jacobians here
    this->pred_mu.topLeftCorner(CAR_STATE_SIZE, 1) = pred_car_mu;
    this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = pred_car_cov + this->R;
}

void EKFslam::position_delta_predict(const double delta_robot_x, const double delta_robot_theta) {
    double x, y, theta;
    get_state_from_mu(this->pred_mu, x, y, theta);

    this->pred_mu(0, 0) = x + delta_robot_x * cos(theta);
    this->pred_mu(1, 0) = y + delta_robot_x * sin(theta);
    this->pred_mu(2, 0) = wrap_pi(theta + delta_robot_theta);

    this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) += this->R;
}

void EKFslam::correct(const std::vector<driverless_msgs::msg::Cone>& detected_cones) {
    double x, y, theta;
    get_state_from_mu(this->pred_mu, x, y, theta);

    for (driverless_msgs::msg::Cone cone : detected_cones) {
        // landmark (cone) position in global frame
        double glob_lm_x = x + cone.location.x * cos(theta) - cone.location.y * sin(theta);
        double glob_lm_y = y + cone.location.x * sin(theta) + cone.location.y * cos(theta);

        std::optional<int> associated_idx = find_associated_landmark_idx(this->pred_mu, glob_lm_x, glob_lm_y);

        if (!associated_idx.has_value()) {
            // new landmark
            Eigen::Matrix2d landmark_jacobian = compute_landmark_jacobian(theta, cone.location.x, cone.location.y);
            associated_idx =
                initalise_new_landmark(this->pred_mu, this->pred_cov, glob_lm_x, glob_lm_y, landmark_jacobian, this->Q);
            this->initalise_new_cone_colour();
        }

        this->update_cone_colour(cone, associated_idx.value());

        // z = (  range  )
        //     ( bearing )
        Eigen::MatrixXd z(2, 1);
        z(0, 0) = sqrt(pow(cone.location.x, 2) + pow(cone.location.y, 2));
        z(1, 0) = wrap_pi(atan2(cone.location.y, cone.location.x));

        Eigen::MatrixXd expected_z(2, 1);
        Eigen::MatrixXd observation_jacobian = Eigen::MatrixXd::Zero(cov.rows(), cov.cols());  // H
        compute_expected_z(this->pred_mu, associated_idx.value(), expected_z, observation_jacobian);

        Eigen::MatrixXd K = this->pred_cov * observation_jacobian.transpose() *
                            ((observation_jacobian * this->pred_cov * observation_jacobian.transpose() + Q).inverse());

        this->pred_mu = this->pred_mu + K * (z - expected_z);
        this->pred_cov = (Eigen::MatrixXd::Identity(K.rows(), observation_jacobian.cols()) - K * observation_jacobian) *
                         this->pred_cov;
    }

    this->mu = this->pred_mu;
    this->cov = this->pred_cov;
}

void EKFslam::initalise_new_cone_colour() { this->cone_colours.push_back({0, 0, 0, 0}); }

void EKFslam::update_cone_colour(driverless_msgs::msg::Cone cone, int associated_landmark_idx) {
    int cone_colour_idx = landmark_idx_to_cone_idx(associated_landmark_idx);
    switch (cone.color) {
        case cone.YELLOW:
            this->cone_colours.at(cone_colour_idx).yellow += 1;
            break;
        case cone.BLUE:
            this->cone_colours.at(cone_colour_idx).blue += 1;
            break;
        case cone.ORANGE_BIG:
            this->cone_colours.at(cone_colour_idx).orange_big += 1;
            break;
        case cone.ORANGE_SMALL:
            this->cone_colours.at(cone_colour_idx).orange_small += 1;
            break;

        default:
            break;
    }
}

std::vector<driverless_msgs::msg::Cone> EKFslam::get_cones() {
    std::vector<driverless_msgs::msg::Cone> cones;
    for (uint i = 0; i < this->cone_colours.size(); i++) {
        int lm_idx = cone_idx_to_landmark_idx(i);
        driverless_msgs::msg::Cone cone;
        cone.location.x = this->mu(lm_idx, 0);
        cone.location.y = this->mu(lm_idx + 1, 0);
        cone.location.z = 0;
        cone.color = get_cone_colour(this->cone_colours[i]);
        cones.push_back(cone);
    }
    return cones;
}
