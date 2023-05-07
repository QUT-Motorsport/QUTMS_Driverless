#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "../markers.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/debug_msg.hpp"
#include "driverless_msgs/msg/double_matrix.hpp"
#include "driverless_msgs/msg/wss_velocity.hpp"
#include "ekf_slam.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

const std::string PARAM_RANGE_VARIANCE = "range_variance";
const std::string PARAM_BEARING_VARIANCE = "bearing_variance";
const std::string PARAM_UNCERTANTY_TIME_WEIGHT = "uncertanty_time_weight";
const std::string PARAM_UNCERTANTY_ROTATION_WEIGHT = "uncertanty_rotation_weight";
const std::string PARAM_UNCERTANTY_FORWARD_WEIGHT = "uncertanty_forward_weight";
const std::string PARAM_UNCERTANTY_HEADING_TIME_WEIGHT = "uncertanty_heading_time_weight";
const std::string PARAM_ASSOCIATION_DIST_THRESHOLD = "association_dist_threshold";
const std::string PARAM_USE_TOTAL_ABS_VEL = "use_total_abs_vel";

double compute_dt(rclcpp::Time start_, rclcpp::Time end_) { return (end_ - start_).nanoseconds() * 1e-9; }

class EKFSLAMNode : public rclcpp::Node {
   private:
    EKFslam ekf_slam;

    double forward_vel;
    double rotational_vel;
    std::optional<rclcpp::Time> last_update;

    double range_variance;
    double bearing_variance;

    double uncertanty_time_weight;
    double uncertanty_rotation_weight;
    double uncertanty_forward_weight;
    double uncertanty_heading_time_weight;

    bool association_dist_threshold;
    bool use_total_abs_vel;

    std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> twist_queue;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr track_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
            "velocity", 10, std::bind(&EKFSLAMNode::twist_callback, this, _1));

        detection_sub = create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "cone_detection", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        pose_pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/pose", 10);
        track_pub = create_publisher<driverless_msgs::msg::ConeDetectionStamped>("slam/track", 10);

        declare_parameter<double>(PARAM_RANGE_VARIANCE, 0.1);
        declare_parameter<double>(PARAM_BEARING_VARIANCE, 0.1);

        declare_parameter<double>(PARAM_UNCERTANTY_TIME_WEIGHT, 0.005);
        declare_parameter<double>(PARAM_UNCERTANTY_ROTATION_WEIGHT, 0.005);
        declare_parameter<double>(PARAM_UNCERTANTY_FORWARD_WEIGHT, 0.005);
        declare_parameter<double>(PARAM_UNCERTANTY_HEADING_TIME_WEIGHT, 0.005);

        declare_parameter<double>(PARAM_ASSOCIATION_DIST_THRESHOLD, 1.5);
        declare_parameter<bool>(PARAM_USE_TOTAL_ABS_VEL, false);

        range_variance = get_parameter(PARAM_RANGE_VARIANCE).as_double();
        bearing_variance = get_parameter(PARAM_BEARING_VARIANCE).as_double();

        uncertanty_time_weight = get_parameter(PARAM_UNCERTANTY_TIME_WEIGHT).as_double();
        uncertanty_rotation_weight = get_parameter(PARAM_UNCERTANTY_ROTATION_WEIGHT).as_double();
        uncertanty_forward_weight = get_parameter(PARAM_UNCERTANTY_FORWARD_WEIGHT).as_double();
        uncertanty_heading_time_weight = get_parameter(PARAM_UNCERTANTY_HEADING_TIME_WEIGHT).as_double();

        association_dist_threshold = get_parameter(PARAM_ASSOCIATION_DIST_THRESHOLD).as_double();
        use_total_abs_vel = get_parameter(PARAM_USE_TOTAL_ABS_VEL).as_bool();
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { twist_queue.push(msg); }

    void predict(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;

        if (use_total_abs_vel) {
            forward_vel = abs(msg->twist.linear.x) + abs(msg->twist.linear.y) + abs(msg->twist.linear.z);
        } else {
            forward_vel = msg->twist.linear.x;
        }

        rotational_vel = -msg->twist.angular.z;

        if (!last_update.has_value()) {
            last_update = stamp;
            return;
        }

        double dt = compute_dt(last_update.value(), stamp);

        if (!(dt > 0)) {
            return;
        }

        ekf_slam.predict(forward_vel, rotational_vel, dt, uncertanty_time_weight, uncertanty_rotation_weight,
                         uncertanty_forward_weight, uncertanty_heading_time_weight);

        last_update = stamp;
    }

    void cone_detection_callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr detection_msg) {
        rclcpp::Time stamp = detection_msg->header.stamp;

        if (!last_update.has_value()) {
            last_update = stamp;
            return;
        }

        while (!twist_queue.empty() && compute_dt(twist_queue.front()->header.stamp, stamp) >= 0) {
            predict(twist_queue.front());
            twist_queue.pop();
        }

        double dt = compute_dt(last_update.value(), stamp);

        if (!(dt > 0)) {
            return;
        }

        std::transform(detection_msg->cones_with_cov.cbegin(), detection_msg->cones_with_cov.cend(),
                       std::back_inserter(detection_msg->cones),
                       [](const driverless_msgs::msg::ConeWithCovariance& c) { return c.cone; });

        ekf_slam.predict(forward_vel, rotational_vel, dt, uncertanty_time_weight, uncertanty_rotation_weight,
                         uncertanty_forward_weight, uncertanty_heading_time_weight);
        ekf_slam.update(detection_msg->cones, range_variance, bearing_variance, association_dist_threshold,
                        this->get_logger());

        last_update = stamp;
        publish_state(detection_msg->header.stamp);
    }

    void publish_state(builtin_interfaces::msg::Time stamp) {
        double x, y, theta;
        ekf_slam.get_state(x, y, theta);
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "track";
        pose_msg.header.stamp = stamp;
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = 0;
        Eigen::Quaterniond q(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
        auto coeffs = q.coeffs();
        pose_msg.pose.pose.orientation.x = coeffs(0);
        pose_msg.pose.pose.orientation.y = coeffs(1);
        pose_msg.pose.pose.orientation.z = coeffs(2);
        pose_msg.pose.pose.orientation.w = coeffs(3);
        pose_msg.pose.covariance[0 + 0 * 6] = ekf_slam.get_cov()(0, 0);
        pose_msg.pose.covariance[1 + 1 * 6] = ekf_slam.get_cov()(1, 1);
        pose_msg.pose.covariance[5 + 5 * 6] = ekf_slam.get_cov()(2, 2);
        pose_pub->publish(pose_msg);

        driverless_msgs::msg::ConeDetectionStamped cones_msg;
        cones_msg.header.frame_id = "track";
        cones_msg.header.stamp = stamp;
        cones_msg.cones_with_cov = ekf_slam.get_cones();
        track_pub->publish(cones_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto ekf_node = std::make_shared<EKFSLAMNode>();
    rclcpp::spin(ekf_node);
    rclcpp::shutdown();

    return 0;
}
