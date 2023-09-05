
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"
#include "geometry_msgs/msg/twist.hpp
#include "driverless_common/common.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class SBGConvert : public rclcpp::Node {
   private:
    // subscriber
    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr euler_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr nav_sub_;
    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;

    // publishers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr odom_pub_;

    // init orientation
    geometry_msgs::msg::Quaternion init_orientation;
    // last orientation
    geometry_msgs::msg::Quaternion last_orientation;
    // init twist
    geometry_msgs::msg::Twist init_twist;
    // last twist
    geometry_msgs::msg::Twist last_twist;

    const nav_msgs::msg::Odometry update_odom(const sbg_driver::msg::SbgImuData &ref_sbg_imu_msg,
                                              const sbg_driver::msg::SbgEkfNav &ref_ekf_nav_msg,
                                              const tf2::Quaternion &ref_orientation,
                                              const sbg_driver::msg::SbgEkfEuler &ref_ekf_euler_msg) {
        nav_msgs::msg::Odometry odo_ros_msg;
        double utm_northing, utm_easting;
        std::string utm_zone;
        geometry_msgs::msg::TransformStamped transform;

        // The pose message provides the position and orientation of the robot relative to the frame specified in
        // header.frame_id
        odo_ros_msg.header = createRosHeader(ref_sbg_imu_msg.time_stamp);
        odo_ros_msg.header.frame_id = m_odom_frame_id_;
        tf2::convert(ref_orientation, odo_ros_msg.pose.pose.orientation);

        // Convert latitude and longitude to UTM coordinates.
        if (m_utm0_.zone == 0) {
            initUTM(ref_ekf_nav_msg.latitude, ref_ekf_nav_msg.longitude, ref_ekf_nav_msg.altitude);
        }

        LLtoUTM(ref_ekf_nav_msg.latitude, ref_ekf_nav_msg.longitude, m_utm0_.zone, utm_easting, utm_northing);
        odo_ros_msg.pose.pose.position.x = utm_northing - m_utm0_.easting;
        odo_ros_msg.pose.pose.position.y = utm_easting - m_utm0_.northing;
        odo_ros_msg.pose.pose.position.z = ref_ekf_nav_msg.altitude - m_utm0_.altitude;

        // Compute convergence angle.
        double longitudeRad = sbgDegToRadD(ref_ekf_nav_msg.longitude);
        double latitudeRad = sbgDegToRadD(ref_ekf_nav_msg.latitude);
        double central_meridian = sbgDegToRadD(computeMeridian(m_utm0_.zone));
        double convergence_angle = atan(tan(longitudeRad - central_meridian) * sin(latitudeRad));

        // Convert position standard deviations to UTM frame.
        double std_east = ref_ekf_nav_msg.position_accuracy.x;
        double std_north = ref_ekf_nav_msg.position_accuracy.y;
        double std_x = std_north * cos(convergence_angle) - std_east * sin(convergence_angle);
        double std_y = std_north * sin(convergence_angle) + std_east * cos(convergence_angle);
        double std_z = ref_ekf_nav_msg.position_accuracy.z;
        odo_ros_msg.pose.covariance[0 * 6 + 0] = std_x * std_x;
        odo_ros_msg.pose.covariance[1 * 6 + 1] = std_y * std_y;
        odo_ros_msg.pose.covariance[2 * 6 + 2] = std_z * std_z;
        odo_ros_msg.pose.covariance[3 * 6 + 3] = ref_ekf_euler_msg.accuracy.x * ref_ekf_euler_msg.accuracy.x;
        odo_ros_msg.pose.covariance[4 * 6 + 4] = ref_ekf_euler_msg.accuracy.y * ref_ekf_euler_msg.accuracy.y;
        odo_ros_msg.pose.covariance[5 * 6 + 5] = ref_ekf_euler_msg.accuracy.z * ref_ekf_euler_msg.accuracy.z;

        // The twist message gives the linear and angular velocity relative to the frame defined in child_frame_id
        odo_ros_msg.child_frame_id = m_frame_id_;
        odo_ros_msg.twist.twist.linear.x = ref_ekf_nav_msg.velocity.x;
        odo_ros_msg.twist.twist.linear.y = ref_ekf_nav_msg.velocity.y;
        odo_ros_msg.twist.twist.linear.z = ref_ekf_nav_msg.velocity.z;
        odo_ros_msg.twist.twist.angular.x = ref_sbg_imu_msg.gyro.x;
        odo_ros_msg.twist.twist.angular.y = ref_sbg_imu_msg.gyro.y;
        odo_ros_msg.twist.twist.angular.z = ref_sbg_imu_msg.gyro.z;
        odo_ros_msg.twist.covariance[0 * 6 + 0] =
            ref_ekf_nav_msg.velocity_accuracy.x * ref_ekf_nav_msg.velocity_accuracy.x;
        odo_ros_msg.twist.covariance[1 * 6 + 1] =
            ref_ekf_nav_msg.velocity_accuracy.y * ref_ekf_nav_msg.velocity_accuracy.y;
        odo_ros_msg.twist.covariance[2 * 6 + 2] =
            ref_ekf_nav_msg.velocity_accuracy.z * ref_ekf_nav_msg.velocity_accuracy.z;
        odo_ros_msg.twist.covariance[3 * 6 + 3] = 0;
        odo_ros_msg.twist.covariance[4 * 6 + 4] = 0;
        odo_ros_msg.twist.covariance[5 * 6 + 5] = 0;

        return odo_ros_msg;
    }

    void update_odom() {
        // use last velocity and steering angle to update odom
        odom_msg.header.stamp = this->now();
        odom_msg.twist.twist.linear.x = last_velocity;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = last_velocity * tan(last_steering_angle) / AXLE_WIDTH;
        odom_pub_->publish(odom_msg);
    }

    void euler_callback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg) {
        // update last angles
    }

   public:
    SBGConvert() : Node("sbg_converter_node") {
        // subscribe to sbg msgs
        this->euler_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>(
            "/sbg/ekf_euler", QOS_LATEST, std::bind(&SBGConvert::euler_callback, this, _1));
        this->nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
            "/sbg/ekf_nav", QOS_LATEST, std::bind(&SBGConvert::nav_callback, this, _1));
        this->imu_sub_ = this->create_subscription<sbg_driver::msg::SbgImuData>(
            "/sbg/imu_data", QOS_LATEST, std::bind(&SBGConvert::imu_callback, this, _1));

        // Odometry
        this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/sbg_ekf", QOS_LATEST);

        RCLCPP_INFO(this->get_logger(), "---CANBus Translator Node Initialised---");
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGConvert>());
    rclcpp::shutdown();
    return 0;
}
