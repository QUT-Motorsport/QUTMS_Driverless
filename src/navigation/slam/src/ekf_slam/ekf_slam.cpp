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

void compute_motion_model(double dt, double forward_vel, double theta_dot, const Eigen::MatrixXd& mu,
                          Eigen::MatrixXd& pred_mu_out,
                          Eigen::MatrixXd& jacobian_out  // G_x
) {
    // this function is g()

    double x, y, theta;
    get_state_from_mu(mu, x, y, theta);

    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // predict new xdot, ydot, x, y, theta position
    pred_mu_out(0, 0) = x + forward_vel * dt * cos_theta;  // x'
    pred_mu_out(1, 0) = y + forward_vel * dt * sin_theta;  // y'
    pred_mu_out(2, 0) = theta + theta_dot * dt;            // theta'

    // compute jacobian for the robot state (G_x)
    jacobian_out << 1, 0, -forward_vel * dt * sin_theta, 0, 1, forward_vel * dt * cos_theta, 0, 0, 2;
}

void update_pred_motion_cov(const Eigen::MatrixXd& motion_jacobian,  // G_x
                            const Eigen::MatrixXd& cov, Eigen::MatrixXd& pred_cov_out) {
    // G = ( G_x  0 )
    //     ( 0    I )
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(cov.rows(), cov.cols());
    G.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = motion_jacobian;

    // R is just identity for car state atm
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(cov.rows(), cov.cols());
    R.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE) * 0.1;

    pred_cov_out = G * cov * G.transpose() + R;
    // pred_cov_out = G * cov * G.transpose();
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

    this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) = pred_car_cov;
    this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) += this->R;
}

void EKFslam::correct(const std::vector<driverless_msgs::msg::Cone>& detected_cones) {
    double x, y, theta;
    get_state_from_mu(this->pred_mu, x, y, theta);

    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    for (driverless_msgs::msg::Cone cone : detected_cones) {
        // landmark (cone) position in global frame
        double glob_lm_x = x + cone.location.x * cos_theta - cone.location.y * sin_theta;
        double glob_lm_y = y + cone.location.x * sin_theta + cone.location.y * cos_theta;

        std::optional<int> associated_idx = find_associated_landmark_idx(this->mu, glob_lm_x, glob_lm_y);

        if (!associated_idx.has_value()) {
            // new landmark
            this->pred_mu.conservativeResize(this->pred_mu.rows() + 2, Eigen::NoChange);
            this->pred_cov.conservativeResize(this->pred_cov.rows() + 2, pred_cov.cols() + 2);

            int new_lm_idx = this->pred_mu.rows() - 2;

            this->pred_mu(new_lm_idx, 0) = glob_lm_x;
            this->pred_mu(new_lm_idx + 1, 0) = glob_lm_y;

            for (int i = 0; i <= new_lm_idx + 1; i++) {
                this->pred_cov(i, new_lm_idx) = 0.5;
                this->pred_cov(i, new_lm_idx + 1) = 0.5;
                this->pred_cov(new_lm_idx, i) = 0.5;
                this->pred_cov(new_lm_idx + 1, i) = 0.5;
            }

            associated_idx = new_lm_idx;
        }

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
