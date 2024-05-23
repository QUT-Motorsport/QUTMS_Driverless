#include "node_sbg_translator.hpp"

SBGTranslate::SBGTranslate() : Node("sbg_translator_node") {
    // subscribe to sbg msgs
    // this->euler_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>(
    //     "/sbg/ekf_euler", 1, std::bind(&SBGTranslate::euler_callback, this, _1));
    // this->nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
    //     "/sbg/ekf_nav", 1, std::bind(&SBGTranslate::nav_callback, this, _1));
    // this->imu_sub_ = this->create_subscription<sbg_driver::msg::SbgImuData>(
    //     "/sbg/imu_data", 1, std::bind(&SBGTranslate::imu_callback, this, _1));

    // subscribe to ekf odom
    this->ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/imu/odometry", 1, std::bind(&SBGTranslate::ekf_odom_callback, this, _1));

    // this->imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu/data", 1, std::bind(&SBGTranslate::imu_data_callback, this, _1));

    // Odometry
    this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sbg_translated/odometry", 1);

    // Pose (for visuals)
    this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sbg_translated/pose", 1);
    this->raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu/pose", 1);

    // Pose path (for visuals)
    this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sbg_translated/path_odom", 1);

    // IMU for GPS init heading
    this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sbg_translated/imu", 1);

    m_utm0_.easting = 0.0;
    m_utm0_.northing = 0.0;
    m_utm0_.altitude = 0.0;
    m_utm0_.zone = 0;

    state_ = {0.0, 0.0, 0.0};

    last_pub_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "---SBG Odom Converter Node Initialised---");
}

// void SBGTranslate::imu_data_callback(sensor_msgs::msg::Imu imu_data_msg) {
//     sensor_msgs::msg::Imu imu_msg = imu_data_msg;

//     // orient the position delta by the last yaw
//     double yaw = quat_to_euler(imu_msg.orientation.z);

//     imu_msg.orientation = euler_to_quat(0.0, 0.0, -yaw);
//     imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z;
    
//     imu_pub_->publish(imu_msg);
// }

void SBGTranslate::update_odom() {
    // only update if all messages have been received
    if (!received_imu_ || !received_nav_ || !received_euler_) {
        return;
    }

    nav_msgs::msg::Odometry odom_msg;
    double utm_northing, utm_easting;

    // The pose message provides the position and orientation of the robot relative to the frame specified in
    // header.frame_id
    odom_msg.header.stamp = last_imu_msg_.header.stamp;
    odom_msg.header.frame_id = "odom";

    // Convert latitude and longitude to UTM coordinates.
    // If the zone is not set, set it.
    if (m_utm0_.zone == 0) {
        initUTM(last_nav_msg_.latitude, last_nav_msg_.longitude, last_nav_msg_.altitude);
    }

    LLtoUTM(last_nav_msg_.latitude, last_nav_msg_.longitude, m_utm0_.zone, utm_easting, utm_northing);
    odom_msg.pose.pose.position.x = utm_northing - m_utm0_.easting;
    odom_msg.pose.pose.position.y = utm_easting - m_utm0_.northing;
    odom_msg.pose.pose.position.z = last_nav_msg_.altitude - m_utm0_.altitude;

    // Compute convergence angle.
    double longitudeRad = sbgDegToRadD(last_nav_msg_.longitude);
    double latitudeRad = sbgDegToRadD(last_nav_msg_.latitude);
    double central_meridian = sbgDegToRadD(computeMeridian(m_utm0_.zone));
    double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));

    // Convert position standard deviations to UTM frame.
    double std_east = last_nav_msg_.position_accuracy.x;
    double std_north = last_nav_msg_.position_accuracy.y;
    double std_x = std_north * cos(convergence_angle) - std_east * sin(convergence_angle);
    double std_y = std_north * sin(convergence_angle) + std_east * cos(convergence_angle);
    double std_z = last_nav_msg_.position_accuracy.z;
    odom_msg.pose.covariance[0 * 6 + 0] = std_x * std_x;
    odom_msg.pose.covariance[1 * 6 + 1] = std_y * std_y;
    odom_msg.pose.covariance[2 * 6 + 2] = std_z * std_z;
    odom_msg.pose.covariance[3 * 6 + 3] = last_euler_msg_.accuracy.x * last_euler_msg_.accuracy.x;
    odom_msg.pose.covariance[4 * 6 + 4] = last_euler_msg_.accuracy.y * last_euler_msg_.accuracy.y;
    odom_msg.pose.covariance[5 * 6 + 5] = last_euler_msg_.accuracy.z * last_euler_msg_.accuracy.z;

    // Convert euler angles to quaternion.
    odom_msg.pose.pose.orientation =
        euler_to_quat(last_euler_msg_.angle.x, last_euler_msg_.angle.y, last_euler_msg_.angle.z + init_yaw_);

    // The twist message gives the linear and angular velocity relative to the frame defined in child_frame_id
    odom_msg.twist.twist.linear.x = last_nav_msg_.velocity.x;
    odom_msg.twist.twist.linear.y = last_nav_msg_.velocity.y;
    odom_msg.twist.twist.linear.z = last_nav_msg_.velocity.z;
    odom_msg.twist.twist.angular.x = last_imu_msg_.gyro.x;
    odom_msg.twist.twist.angular.y = last_imu_msg_.gyro.y;
    odom_msg.twist.twist.angular.z = last_imu_msg_.gyro.z;
    odom_msg.twist.covariance[0 * 6 + 0] = last_nav_msg_.velocity_accuracy.x * last_nav_msg_.velocity_accuracy.x;
    odom_msg.twist.covariance[1 * 6 + 1] = last_nav_msg_.velocity_accuracy.y * last_nav_msg_.velocity_accuracy.y;
    odom_msg.twist.covariance[2 * 6 + 2] = last_nav_msg_.velocity_accuracy.z * last_nav_msg_.velocity_accuracy.z;
    odom_msg.twist.covariance[3 * 6 + 3] = 0;
    odom_msg.twist.covariance[4 * 6 + 4] = 0;
    odom_msg.twist.covariance[5 * 6 + 5] = 0;

    odom_msg.child_frame_id = "base_footprint";

    // publish
    // odom_pub_->publish(odom_msg);

    // publish imu message
    imu_pub_->publish(make_imu_msg(odom_msg));
}

void SBGTranslate::initUTM(double Lat, double Long, double altitude) {
    int zoneNumber;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

    zoneNumber = int((LongTemp + 180) / 6) + 1;

    if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0) {
        zoneNumber = 32;
    }

    m_utm0_.zone = zoneNumber;
    m_utm0_.altitude = altitude;
    LLtoUTM(Lat, Long, m_utm0_.zone, m_utm0_.northing, m_utm0_.easting);

    RCLCPP_INFO(this->get_logger(), "initialized from lat:%f long:%f easting:%fm (%dkm) northing:%fm (%dkm)", Lat, Long,
                m_utm0_.zone, m_utm0_.easting, (int)(m_utm0_.easting) / 1000, m_utm0_.northing,
                (int)(m_utm0_.northing) / 1000);
}

/*
 * Modification of gps_common::LLtoUTM() to use a constant UTM zone.
 *
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Originally written by Chuck Gantz- chuck.gantz@globalstar.com.
 */
void SBGTranslate::LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting) {
    const double RADIANS_PER_DEGREE = M_PI / 180.0;

    // WGS84 Parameters
    const double WGS84_A = 6378137.0;     // major axis
    const double WGS84_E = 0.0818191908;  // first eccentricity

    // UTM Parameters
    const double UTM_K0 = 0.9996;               // scale factor
    const double UTM_E2 = (WGS84_E * WGS84_E);  // e^2

    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

    double LatRad = Lat * RADIANS_PER_DEGREE;
    double LongRad = LongTemp * RADIANS_PER_DEGREE;
    double LongOriginRad;

    // +3 puts origin in middle of zone
    LongOrigin = (zoneNumber - 1) * 6 - 180 + 3;
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

    eccPrimeSquared = (eccSquared) / (1 - eccSquared);

    N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);

    M = a *
        ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) *
             LatRad -
         (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) *
             sin(2 * LatRad) +
         (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad) -
         (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

    UTMEasting = (double)(k0 * N *
                              (A + (1 - T + C) * A * A * A / 6 +
                               (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) +
                          500000.0);

    UTMNorthing =
        (double)(k0 *
                 (M + N * tan(LatRad) *
                          (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                           (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));

    if (Lat < 0) {
        UTMNorthing += 10000000.0;  // 10000000 meter offset for southern hemisphere
    }
}

void SBGTranslate::euler_callback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg) {
    // update last angles
    last_euler_msg_ = *msg;

    if (!received_euler_) {
        // initialize yaw
        init_yaw_ = last_euler_msg_.angle.z;
    }
    received_euler_ = true;
    update_odom();
}

void SBGTranslate::nav_callback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
    // update last position
    last_nav_msg_ = *msg;
    received_nav_ = true;
    update_odom();
}

void SBGTranslate::imu_callback(const sbg_driver::msg::SbgImuData::SharedPtr msg) {
    // update last imu
    last_imu_msg_ = *msg;
    received_imu_ = true;
    update_odom();
}

void SBGTranslate::ekf_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!received_odom_) {
        // initialize yaw
        // convert quat to euler
        double yaw = quat_to_euler(msg->pose.pose.orientation);
        init_yaw_ = -yaw;

        // initialize position
        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;

        received_odom_ = true;
        return;
    }

    // create a new pose msg with the same header
    raw_pose_pub_->publish(make_pose_msg(*msg));

    // create a new odom msg with the same header
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.child_frame_id = "chassis";

    // orient the position delta by the last yaw
    double yaw = quat_to_euler(msg->pose.pose.orientation);

    double update_x = msg->pose.pose.position.x - last_x_;
    double update_y = msg->pose.pose.position.y - last_y_;

    double magnitude = sqrt(update_x * update_x + update_y * update_y);

    double delta_yaw = -yaw - init_yaw_ - state_[2];
    double delta_x = magnitude * cos(state_[2]);
    double delta_y = magnitude * sin(state_[2]);

    // update the state
    state_[0] += delta_x;
    state_[1] += delta_y;
    state_[2] += delta_yaw;

    // update the last position
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;

    // update the odom msg
    odom_msg.pose.pose.position.x = state_[0];
    odom_msg.pose.pose.position.y = state_[1];
    odom_msg.pose.pose.position.z = 0.0;

    // convert yaw to quaternion
    odom_msg.pose.pose.orientation = euler_to_quat(0.0, 0.0, state_[2]);

    // use existing covariance
    odom_msg.pose.covariance = msg->pose.covariance;

    // rotate twist by yaw
    double vel_magnitude = sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                                msg->twist.twist.linear.y * msg->twist.twist.linear.y);

    odom_msg.twist.twist.linear.x = vel_magnitude * cos(state_[2]);
    odom_msg.twist.twist.linear.y = vel_magnitude * sin(state_[2]);
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = msg->twist.twist.angular.z;

    // publish the odom msg
    odom_pub_->publish(odom_msg);

    // publish imu message
    // imu_pub_->publish(make_imu_msg(odom_msg));

    // publish pose message
    pose_pub_->publish(make_pose_msg(odom_msg));

    // throttle path pub
    // rclcpp time
    if (this->now() - last_pub_time_ < rclcpp::Duration(std::chrono::seconds(1) / 10)) {
        return;
    }

    // keep 50 points in the path
    if (path_msg_.poses.size() > 50) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }
    // update the path
    path_msg_.header = odom_msg.header;
    path_msg_.header.frame_id = "track";
    path_msg_.poses.push_back(make_pose_msg(odom_msg));
    path_pub_->publish(path_msg_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGTranslate>());
    rclcpp::shutdown();
    return 0;
}
