#include "ekf_slam.hpp"

void get_state_from_mu(const Eigen::MatrixXd& mu, double& x, double& y, double& theta) {
    x = mu(0, 0);
    y = mu(1, 0);
    theta = mu(2, 0);
}

void set_state_on_mu(Eigen::MatrixXd& mu, double x, double y, double theta) {
    mu(0, 0) = x;
    mu(1, 0) = y;
    mu(2, 0) = theta;
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

std::optional<int> find_associated_landmark_idx(const Eigen::MatrixXd& mu, double global_x, double global_y) {
    // data association, uses lowest euclidian distance, within a threshold

    double min_distance = 1.5 * 1.5;  // m, threshold^2
    std::optional<int> idx = {};

    for (int i = CAR_STATE_SIZE; i < mu.rows(); i += LANDMARK_STATE_SIZE) {
        // iterate through the state, in intervals of two, since x and y
        // for each landmark are stored in the state vector
        double i_x = mu(i, 0);
        double i_y = mu(i + 1, 0);

        double distance = (global_x - i_x) * (global_x - i_x) + (global_y - i_y) * (global_y - i_y);
        if (distance < min_distance) {
            min_distance = distance;
            idx = i;
        }
    }

    return idx;
}

int initalise_new_landmark(Eigen::MatrixXd& mu, Eigen::MatrixXd& cov, double lm_map_x, double lm_map_y, double lm_range,
                           double lm_bearing, const Eigen::Matrix2d& Q) {
    mu.conservativeResize(mu.rows() + LANDMARK_STATE_SIZE, Eigen::NoChange);
    cov.conservativeResize(cov.rows() + LANDMARK_STATE_SIZE, cov.cols() + LANDMARK_STATE_SIZE);

    int new_lm_idx = mu.rows() - LANDMARK_STATE_SIZE;

    mu(new_lm_idx, 0) = lm_map_x;
    mu(new_lm_idx + 1, 0) = lm_map_y;

    // initalise the new covariance rows/cols to zero
    cov(Eigen::lastN(LANDMARK_STATE_SIZE), Eigen::all).setZero();
    cov(Eigen::all, Eigen::lastN(LANDMARK_STATE_SIZE)).setZero();

    double x, y, theta;
    get_state_from_mu(mu, x, y, theta);

    // clang-format off
    Eigen::Matrix<double, 2, 3> jGx;  // jacobian wrt state (pose)
    jGx << 1, 0, -lm_range*sin(theta+lm_bearing),
           0, 1, lm_range*cos(theta+lm_bearing);

    Eigen::Matrix2d jGz;  // jacobian wrt observations (range, bearing)
    jGz << cos(theta+lm_bearing), -lm_range*sin(theta+lm_bearing),
           sin(theta+lm_bearing), lm_range*cos(theta+lm_bearing);
    // clang-format on

    Eigen::Matrix<double, 2, 3> covLmState = jGx * cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE);
    Eigen::Matrix<double, 2, 2> covLmLm = covLmState * jGx.transpose() + jGz * Q * jGz.transpose();

    cov.bottomLeftCorner(LANDMARK_STATE_SIZE, CAR_STATE_SIZE) = covLmState;
    cov.topRightCorner(CAR_STATE_SIZE, LANDMARK_STATE_SIZE) = covLmState.transpose();
    cov.bottomRightCorner(LANDMARK_STATE_SIZE, LANDMARK_STATE_SIZE) = covLmLm;

    return new_lm_idx;
}

void compute_expected_z(const Eigen::MatrixXd& mu, int landmark_idx, Eigen::Matrix<double, 2, 1>& expected_z_out,
                        Eigen::Matrix<double, 2, -1>& jH_out) {
    // this function is h()

    double x, y, theta;
    get_state_from_mu(mu, x, y, theta);

    double lm_x = mu(landmark_idx);
    double lm_y = mu(landmark_idx + 1);

    double dx = lm_x - x;
    double dy = lm_y - y;

    double q = pow(dx, 2) + pow(dy, 2);
    double r = sqrt(q);

    // z = (  range  )
    //     ( bearing )
    expected_z_out(0, 0) = r;
    expected_z_out(1, 0) = wrap_pi(atan2(dy, dx) - theta);

    double sqrt_q = sqrt(q);
    jH_out = Eigen::MatrixXd::Zero(2, mu.rows());
    // clang-format off
    jH_out.topLeftCorner(2, CAR_STATE_SIZE) << -sqrt_q*dx, -sqrt_q*dy, 0,
                                               dy,         -dx,        -q;
    jH_out(Eigen::all, Eigen::seqN(landmark_idx, LANDMARK_STATE_SIZE)) << sqrt_q*dx, sqrt_q*dy,
                                                                          -dy,       dx;
    // clang-format on

    jH_out = (1 / q) * jH_out;
}

EKFslam::EKFslam() {
    // initalise state and convariance with just the car state
    // (landmarks will be added when they are detected)
    pred_mu = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, 1);
    pred_cov = Eigen::MatrixXd::Zero(CAR_STATE_SIZE, CAR_STATE_SIZE);
    // pred_cov = 0.5*Eigen::MatrixXd::Identity(CAR_STATE_SIZE, CAR_STATE_SIZE);
    // pred_mu(2, 0) = -M_PI_2;
    pred_cov(0, 0) = 0.5;
    pred_cov(1, 1) = 0.5;
    pred_cov(2, 2) = 0.01;
    mu = pred_mu;
    cov = pred_cov;
}

void EKFslam::predict(double forward_vel, double rotational_vel, double dt,
                      std::optional<rclcpp::Publisher<driverless_msgs::msg::DoubleMatrix>::SharedPtr> matrix_pub) {
    double x, y, theta;
    get_state_from_mu(this->pred_mu, x, y, theta);

    // this is the function f()
    double pred_x = x + dt * forward_vel * cos(theta);
    double pred_y = y + dt * forward_vel * sin(theta);
    double pred_theta = wrap_pi(theta + dt * rotational_vel);
    set_state_on_mu(this->pred_mu, pred_x, pred_y, pred_theta);

    // clang-format off
    Eigen::Matrix3d jFx;  // jacobian wrt state (pose)
    jFx << 1, 0, -dt*forward_vel*sin(theta),
           0, 1, dt*forward_vel*cos(theta),
           0, 0, 1;

    Eigen::Matrix<double, 3, 2> jFu;  // jacobian wrt input (odometry)
    jFu << dt*cos(theta), 0,
           dt*sin(theta), 0,
           0,             dt;
    // clang-format on

    this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) =
        jFx * this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) * jFx.transpose() +
        jFu * R * jFu.transpose();

    std::cout << pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) << "\n" << std::endl;

    // this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) =
    //     jFx * this->pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE) * jFx.transpose();

    mu = pred_mu;
    cov = pred_cov;

    // if (matrix_pub.has_value()) {
    //         driverless_msgs::msg::DoubleMatrix matrix;
    //         matrix.rows = pred_cov.rows();
    //         matrix.columns = pred_cov.cols();
    //         for (auto x : pred_cov.reshaped<Eigen::RowMajor>()) {
    //             matrix.values.push_back(x);
    //         }
    //         matrix_pub.value()->publish(matrix);
    //     }
}

void EKFslam::update(const std::vector<driverless_msgs::msg::ConeWithCovariance>& detected_cones,
                     std::optional<rclcpp::Publisher<driverless_msgs::msg::DebugMsg>::SharedPtr> debug_1_pub,
                     std::optional<rclcpp::Publisher<driverless_msgs::msg::DebugMsg>::SharedPtr> debug_2_pub,
                     std::optional<rclcpp::Publisher<driverless_msgs::msg::DoubleMatrix>::SharedPtr> matrix_pub) {
    for (auto const& cov_cone : detected_cones) {
        auto cone = cov_cone.cone;

        double x, y, theta;
        get_state_from_mu(pred_mu, x, y, theta);

        // landmark (cone) position in map frame
        double lm_map_x = x + cone.location.x * cos(theta) - cone.location.y * sin(theta);
        double lm_map_y = y + cone.location.x * sin(theta) + cone.location.y * cos(theta);

        // "observations" of the landmark - range, bearing
        double lm_range = sqrt(pow(cone.location.x, 2) + pow(cone.location.y, 2));
        double lm_bearing = wrap_pi(atan2(cone.location.y, cone.location.x));

        // std::optional<int> associated_idx = find_associated_landmark_idx(pred_mu, lm_map_x, lm_map_y);
        std::optional<int> associated_idx = find_associated_cone_idx_from_sim_idx(cone.sim_cone_index);

        if (!associated_idx.has_value()) {
            // new landmark
            associated_idx = initalise_new_landmark(pred_mu, pred_cov, lm_map_x, lm_map_y, lm_range, lm_bearing, Q);
            initalise_new_cone_colour();
            initalise_new_cone_sim_idx(cone.sim_cone_index, associated_idx.value());
        }

        update_cone_colour(cone, associated_idx.value());

        // z = (  range  )
        //     ( bearing )
        Eigen::Matrix<double, 2, 1> z;
        z << lm_range, lm_bearing;

        Eigen::Matrix<double, 2, 1> expected_z;
        Eigen::Matrix<double, 2, -1> jH;  // jacobian of obs model (H) wrt robot state and lm state
        compute_expected_z(pred_mu, associated_idx.value(), expected_z, jH);

        Eigen::MatrixXd K = pred_cov * jH.transpose() * ((jH * pred_cov * jH.transpose() + Q).inverse());

        if (debug_1_pub.has_value()) {
            driverless_msgs::msg::DebugMsg debug_1;
            auto col_0 = K.col(0);
            for (size_t i = 0; i < col_0.size(); i++) {
                debug_1.values.push_back(col_0(i));
            }
            debug_1_pub.value()->publish(debug_1);
        }

        if (debug_2_pub.has_value()) {
            driverless_msgs::msg::DebugMsg debug_2;
            auto col_1 = K.col(1);
            for (size_t i = 0; i < col_1.size(); i++) {
                debug_2.values.push_back(col_1(i));
            }
            debug_2_pub.value()->publish(debug_2);
        }

        if (matrix_pub.has_value()) {
            driverless_msgs::msg::DoubleMatrix matrix;
            matrix.rows = CAR_STATE_SIZE;
            matrix.columns = CAR_STATE_SIZE;
            for (auto x : pred_cov.topLeftCorner(CAR_STATE_SIZE, CAR_STATE_SIZE).reshaped<Eigen::RowMajor>()) {
                matrix.values.push_back(x);
            }
            // matrix.rows = pred_cov.rows();
            // matrix.columns = pred_cov.cols();
            // for (auto x : pred_cov.reshaped<Eigen::RowMajor>()) {
            //     matrix.values.push_back(x);
            // }
            matrix_pub.value()->publish(matrix);
        }

        Eigen::MatrixXd z_diff = z - expected_z;
        z_diff(1, 0) = wrap_pi(z_diff(1, 0));

        pred_mu = pred_mu + K * z_diff;
        pred_mu(2, 0) = wrap_pi(pred_mu(2, 0));

        pred_cov = (Eigen::MatrixXd::Identity(K.rows(), jH.cols()) - K * jH) * pred_cov;
        std::cout << pred_cov(1, 1) << " (update)" << std::endl;
    }

    mu = pred_mu;
    cov = pred_cov;
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

void EKFslam::initalise_new_cone_sim_idx(int sim_index, int lm_index) { cone_sim_indexes[sim_index] = lm_index; }

std::optional<int> EKFslam::find_associated_cone_idx_from_sim_idx(int sim_index) {
    if (!cone_sim_indexes.count(sim_index)) {
        return {};
    }
    return cone_sim_indexes.at(sim_index);
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
