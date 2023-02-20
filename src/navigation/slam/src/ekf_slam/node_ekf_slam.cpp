#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <vector>

#include "../markers.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
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
    struct Pose {
        double x;
        double y;
        double theta;
    };

    EKFslam ekf_slam;

    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_msgs;
    std::optional<Pose> prev_pose;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "zed2i/zed_node/pose_with_covariance", 10, std::bind(&EKFSLAMNode::pose_callback, this, _1));

        detection_sub = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "sim/cone_detection", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_visualisation", 10);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
        this->pose_msgs.push_back(pose_msg);
    }

    void cone_detection_callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr detection_msg) {
        auto earlier_than_detection =
            [detection_msg](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
                return compute_dt(pose_msg->header.stamp, detection_msg->header.stamp) >= 0;
            };

        auto result = std::find_if(std::rbegin(pose_msgs), std::rend(pose_msgs), earlier_than_detection);
        if (result == std::rend(pose_msgs)) {
            return;
        }

        this->pose_delta_update(*result);
        this->cone_update(detection_msg);

        std::cout << "Cones" << std::endl;
        publish_visualisations(detection_msg->header.stamp);
    }

    void pose_delta_update(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
        tf2::Quaternion q_orientation;
        tf2::convert(pose_msg->pose.pose.orientation, q_orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orientation).getRPY(roll, pitch, yaw);

        Pose pose = {pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, yaw};

        if (!prev_pose.has_value()) {
            prev_pose = pose;
        }

        double dx = pose.x - prev_pose.value().x;
        double dy = pose.y - prev_pose.value().y;
        double delta_robot_x = sqrt(pow(dx, 2) + pow(dy, 2));
        double delta_robot_theta = pose.theta - prev_pose.value().theta;

        this->ekf_slam.predict(delta_robot_x, delta_robot_theta);
        prev_pose = pose;
    }

    void cone_update(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr detection_msg) {
        this->ekf_slam.update(detection_msg->cones);
    }

    void print_state() {
        double pred_x, pred_y, pred_theta;
        this->ekf_slam.get_pred_state(pred_x, pred_y, pred_theta);

        std::cout << "PRED" << std::endl;
        std::cout << "pos: " << pred_x << " " << pred_y << std::endl;
        std::cout << "theta: " << pred_theta << std::endl;
        std::cout << "x car cov: " << this->ekf_slam.get_pred_cov()(0, 0) << std::endl;
        std::cout << "y car cov: " << this->ekf_slam.get_pred_cov()(1, 1) << std::endl;
        std::cout << std::endl;

        double x, y, theta;
        this->ekf_slam.get_state(x, y, theta);

        std::cout << "UPDATED" << std::endl;
        std::cout << "pos: " << x << " " << y << std::endl;
        std::cout << "theta: " << theta << std::endl;
        std::cout << "x car cov: " << this->ekf_slam.get_pred_cov()(0, 0) << std::endl;
        std::cout << "y car cov: " << this->ekf_slam.get_pred_cov()(1, 1) << std::endl;
        std::cout << std::endl;
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
                cone_marker(stamp, "cone_pos", i, cone.location.x, cone.location.y, cone.color));
            marker_array.markers.push_back(
                cov_marker(stamp, "cone_cov", i, cone.location.x, cone.location.y, cov_x, cov_y));
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
