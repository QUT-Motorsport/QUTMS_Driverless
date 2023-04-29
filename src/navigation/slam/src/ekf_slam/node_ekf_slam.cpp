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

double compute_dt(rclcpp::Time start_, rclcpp::Time end_) { return (end_ - start_).nanoseconds() * 1e-9; }

class EKFSLAMNode : public rclcpp::Node {
   private:
    EKFslam ekf_slam;

    double forward_vel;
    double rotational_vel;
    std::optional<rclcpp::Time> last_update;

    std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> twist_queue;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr track_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "imu/velocity", 10, std::bind(&EKFSLAMNode::twist_callback, this, _1));

        detection_sub = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "vision/cone_detection2", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("slam/markers", 10);
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/pose", 10);
        track_pub = this->create_publisher<driverless_msgs::msg::ConeDetectionStamped>("slam/track", 10);
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { twist_queue.push(msg); }

    void predict(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;

        // forward_vel = abs(msg->twist.linear.x) + abs(msg->twist.linear.y);
        forward_vel = msg->twist.linear.x;
        rotational_vel = msg->twist.angular.z;

        if (!last_update.has_value()) {
            last_update = stamp;
            return;
        }

        double dt = compute_dt(last_update.value(), stamp);

        if (!(dt > 0)) {
            return;
        }

        ekf_slam.predict(forward_vel, rotational_vel, dt);

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

        ekf_slam.predict(forward_vel, rotational_vel, dt);
        ekf_slam.update(detection_msg->cones, this->get_logger());

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
        std::cout << ekf_slam.get_cov()(2, 2) << std::endl;
        pose_pub->publish(pose_msg);

        driverless_msgs::msg::ConeDetectionStamped cones_msg;
        cones_msg.header.frame_id = "track";
        cones_msg.header.stamp = stamp;
        cones_msg.cones_with_cov = ekf_slam.get_cones();
        track_pub->publish(cones_msg);
    }

    void publish_visualisations(builtin_interfaces::msg::Time stamp) {
        double x, y, theta;
        this->ekf_slam.get_state(x, y, theta);

        auto marker_array = visualization_msgs::msg::MarkerArray();

        marker_array.markers.push_back(car_marker(stamp, "car", 1, x, y, theta));
        marker_array.markers.push_back(
            cov_marker(stamp, "car", 2, x, y, ekf_slam.get_cov()(0, 0), ekf_slam.get_cov()(1, 1)));

        auto cones = ekf_slam.get_cones();

        for (uint i = 0; i < cones.size(); i++) {
            auto cone = cones[i];
            int lm_idx = cone_idx_to_landmark_idx(i);
            double cov_x = ekf_slam.get_cov()(lm_idx, lm_idx);
            double cov_y = ekf_slam.get_cov()(lm_idx + 1, lm_idx + 1);
            marker_array.markers.push_back(
                cone_marker(stamp, "cone_pos", i, cone.cone.location.x, cone.cone.location.y, cone.cone.color));
            marker_array.markers.push_back(
                cov_marker(stamp, "cone_cov", i, cone.cone.location.x, cone.cone.location.y, cov_x, cov_y));
        }

        this->viz_pub->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto ekf_node = std::make_shared<EKFSLAMNode>();
    rclcpp::spin(ekf_node);
    rclcpp::shutdown();

    return 0;
}
