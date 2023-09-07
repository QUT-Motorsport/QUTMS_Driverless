
// Much of this odometry conversion is adapted from:
// https://github.com/SBG-Systems/sbg_ros2_driver/blob/master/src/message_wrapper.cpp

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "driverless_common/common.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

typedef struct _UTM0 {
    double easting;
    double northing;
    double altitude;
    int zone;
} UTM0;

class SBGConvert : public rclcpp::Node {
   private:
    // subscribers
    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr euler_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr nav_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // initial values
    float init_yaw_;
    float last_x_;
    float last_y_;
    UTM0 m_utm0_;

    // updated values
    sbg_driver::msg::SbgImuData last_imu_msg_;
    sbg_driver::msg::SbgEkfNav last_nav_msg_;
    sbg_driver::msg::SbgEkfEuler last_euler_msg_;
    std::vector<float> state_;

    bool received_imu_ = false;
    bool received_nav_ = false;
    bool received_euler_ = false;
    bool received_odom_ = false;

    void update_odom() {
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

    sensor_msgs::msg::Imu make_imu_msg(nav_msgs::msg::Odometry odom_msg) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = odom_msg.header.stamp;
        imu_msg.header.frame_id = "base_footprint";

        imu_msg.orientation = odom_msg.pose.pose.orientation;

        return imu_msg;
    }

    geometry_msgs::msg::Quaternion euler_to_quat(double x, double y, double z) {
        tf2::Quaternion q;
        q.setRPY(x, y, z);
        q.normalize();
        return tf2::toMsg(q);
    }

    void initUTM(double Lat, double Long, double altitude) {
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

        RCLCPP_INFO(this->get_logger(), "initialized from lat:%f long:%f easting:%fm (%dkm) northing:%fm (%dkm)", Lat,
                    Long, m_utm0_.zone, m_utm0_.easting, (int)(m_utm0_.easting) / 1000, m_utm0_.northing,
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
    void LLtoUTM(double Lat, double Long, int zoneNumber, double &UTMNorthing, double &UTMEasting) {
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
             (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
              45 * eccSquared * eccSquared * eccSquared / 1024) *
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

    double computeMeridian(int zone_number) { return (zone_number == 0) ? 0.0 : (zone_number - 1) * 6.0 - 177.0; }
    double sbgDegToRadD(double angle) { return angle * M_PI / 180.0; }

    void euler_callback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg) {
        // update last angles
        last_euler_msg_ = *msg;

        if (!received_euler_) {
            // initialize yaw
            init_yaw_ = last_euler_msg_.angle.z;
        }
        received_euler_ = true;
        update_odom();
    }

    void nav_callback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
        // update last position
        last_nav_msg_ = *msg;
        received_nav_ = true;
        update_odom();
    }

    void imu_callback(const sbg_driver::msg::SbgImuData::SharedPtr msg) {
        // update last imu
        last_imu_msg_ = *msg;
        received_imu_ = true;
        update_odom();
    }

    void ekf_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!received_odom_) {
            // initialize yaw
            // convert quat to euler
            double yaw = quat_to_euler(msg->pose.pose.orientation);
            init_yaw_ = yaw;

            // initialize position
            last_x_ = msg->pose.pose.position.x;
            last_y_ = msg->pose.pose.position.y;

            received_odom_ = true;
            return;
        }

        // create a new odom msg with the same header
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = msg->header;

        // orient the position delta by the last yaw
        double yaw = quat_to_euler(msg->pose.pose.orientation);

        double update_x = msg->pose.pose.position.x - last_x_;
        double update_y = msg->pose.pose.position.y - last_y_;

        double magnitude = sqrt(update_x * update_x + update_y * update_y);

        double delta_yaw = yaw - init_yaw_ - state_[2];
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

        // publish the odom msg
        odom_pub_->publish(odom_msg);

        // publish imu message
        imu_pub_->publish(make_imu_msg(odom_msg));
    }

    double quat_to_euler(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

   public:
    SBGConvert() : Node("sbg_converter_node") {
        // subscribe to sbg msgs
        // this->euler_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>(
        //     "/sbg/ekf_euler", 1, std::bind(&SBGConvert::euler_callback, this, _1));
        // this->nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
        //     "/sbg/ekf_nav", 1, std::bind(&SBGConvert::nav_callback, this, _1));
        // this->imu_sub_ = this->create_subscription<sbg_driver::msg::SbgImuData>(
        //     "/sbg/imu_data", 1, std::bind(&SBGConvert::imu_callback, this, _1));

        // subscribe to ekf odom
        this->ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/imu/odometry", 1, std::bind(&SBGConvert::ekf_odom_callback, this, _1));

        // Odometry
        this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/sbg_ekf", 1);

        // IMU for GPS init heading
        this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/sbg_ekf_heading", 1);

        m_utm0_.easting = 0.0;
        m_utm0_.northing = 0.0;
        m_utm0_.altitude = 0.0;
        m_utm0_.zone = 0;

        state_ = {0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "---SBG Odom Converter Node Initialised---");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGConvert>());
    rclcpp::shutdown();
    return 0;
}
