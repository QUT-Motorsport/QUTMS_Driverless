#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <optional>

#include "../colours.h"
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
    EKFslam ekf_slam;

    std::optional<builtin_interfaces::msg::Time> last_sensed_control_update;
    std::optional<builtin_interfaces::msg::Time> last_cone_update;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "zed2i/zed_node/pose_with_covariance", 10, std::bind(&EKFSLAMNode::pose_callback, this, _1));

        detection_sub = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "detection/cone_detection", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_visualisation", 10);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
        // catch first call where we have no "last update time"pose_msg->pose.pose.orientation
        if (!this->last_sensed_control_update.has_value()) {
            this->last_sensed_control_update = pose_msg->header.stamp;
            return;
        }

        double dt = compute_dt(last_sensed_control_update.value(), pose_msg->header.stamp);
        this->last_sensed_control_update = pose_msg->header.stamp;

        if (dt == 0) {
            return;
        }

        Eigen::Matrix<double, CAR_STATE_SIZE, 1> pred_mu;
        tf2::Quaternion q_orientation;
        tf2::convert(pose_msg->pose.pose.orientation, q_orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orientation).getRPY(roll, pitch, yaw);
        pred_mu << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, wrap_pi(yaw);

        // Covariance from odom
        // xx, xy, xz, xi, xj, xk
        // yx, yy, yz, yi, yj, yk
        // zx, zy, zz, zi, zj, zk
        // ix, iy, iz, ii, ij, ik
        // jx, jy, jz, ji, jj, jk
        // kx, ky, kz, ki, kj, kk
        Eigen::Matrix<double, CAR_STATE_SIZE, CAR_STATE_SIZE> pred_car_cov;
        pred_car_cov << pose_msg->pose.covariance[0], 0, 0, 0, pose_msg->pose.covariance[8], 0, 0, 0,
            pose_msg->pose.covariance[35];

        this->ekf_slam.position_predict(pred_mu, pred_car_cov);

        std::cout << "Pose" << std::endl;
        print_state();
        publish_visualisations(pose_msg->header.stamp);
    }

    void cone_detection_callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr detection_msg) {
        // catch first call where we have no "last update time"
        if (!this->last_cone_update.has_value()) {
            this->last_cone_update = detection_msg->header.stamp;
            return;
        }

        double dt = compute_dt(last_cone_update.value(), detection_msg->header.stamp);
        this->last_cone_update = detection_msg->header.stamp;

        if (dt == 0) {
            return;
        }

        if (!this->last_sensed_control_update.has_value() ||
            compute_dt(last_sensed_control_update.value(), last_cone_update.value()) < 0) {
            return;
        }

        this->ekf_slam.correct(detection_msg->cones);

        std::cout << "Cones" << std::endl;
        print_state();
        publish_visualisations(detection_msg->header.stamp);
    }

    void print_state() {
        double pred_x, pred_y, pred_theta;
        this->ekf_slam.get_pred_state(pred_x, pred_y, pred_theta);

        std::cout << "PRED" << std::endl;
        std::cout << "pos: " << pred_x << " " << pred_y << std::endl;
        std::cout << "theta: " << pred_theta << std::endl;
        std::cout << std::endl;

        double x, y, theta;
        this->ekf_slam.get_state(x, y, theta);

        std::cout << "UPDATED" << std::endl;
        std::cout << "pos: " << x << " " << y << std::endl;
        std::cout << "theta: " << theta << std::endl;
        std::cout << std::endl;
    }

    void publish_visualisations(builtin_interfaces::msg::Time stamp) {
        double x, y, theta, pred_x, pred_y, pred_theta;
        this->ekf_slam.get_state(x, y, theta);
        this->ekf_slam.get_pred_state(pred_x, pred_y, pred_theta);

        tf2::Quaternion pred_heading;
        tf2::Quaternion heading;
        pred_heading.setRPY(0, 0, pred_theta);
        heading.setRPY(0, 0, theta);

        auto marker_array = visualization_msgs::msg::MarkerArray();

        // car
        auto pred_car_marker = visualization_msgs::msg::Marker();
        pred_car_marker.header.frame_id = "map";
        pred_car_marker.header.stamp = stamp;
        pred_car_marker.ns = "car";
        pred_car_marker.id = 1;  // 1 represents predicted car
        pred_car_marker.type = visualization_msgs::msg::Marker::ARROW;
        pred_car_marker.action = visualization_msgs::msg::Marker::ADD;
        pred_car_marker.pose.position.x = pred_x;
        pred_car_marker.pose.position.y = pred_y;
        pred_car_marker.pose.position.z = 0;
        tf2::convert(pred_heading, pred_car_marker.pose.orientation);
        pred_car_marker.scale.x = 1.0;
        pred_car_marker.scale.y = 0.2;
        pred_car_marker.scale.z = 0.2;
        pred_car_marker.color.r = 0.0f;
        pred_car_marker.color.g = 1.0f;
        pred_car_marker.color.b = 0.0f;
        pred_car_marker.color.a = 1.0;
        marker_array.markers.push_back(pred_car_marker);

        auto car_marker = visualization_msgs::msg::Marker();
        car_marker.header.frame_id = "map";
        car_marker.header.stamp = stamp;
        car_marker.ns = "car";
        car_marker.id = 2;  // 2 represents estimated car
        car_marker.type = visualization_msgs::msg::Marker::ARROW;
        car_marker.action = visualization_msgs::msg::Marker::ADD;
        car_marker.pose.position.x = x;
        car_marker.pose.position.y = y;
        car_marker.pose.position.z = 0;
        tf2::convert(heading, car_marker.pose.orientation);
        car_marker.scale.x = 1.0;
        car_marker.scale.y = 0.2;
        car_marker.scale.z = 0.2;
        car_marker.color.r = 1.0f;
        car_marker.color.g = 0.0f;
        car_marker.color.b = 0.0f;
        car_marker.color.a = 1.0;
        marker_array.markers.push_back(car_marker);

        std::cout << "x car cov: " << ekf_slam.get_cov()(0, 0) << std::endl;
        std::cout << "y car cov: " << ekf_slam.get_cov()(1, 1) << std::endl;
        auto car_cov_marker = visualization_msgs::msg::Marker();
        car_cov_marker.header.frame_id = "map";
        car_cov_marker.header.stamp = stamp;
        car_cov_marker.ns = "car";
        car_cov_marker.id = 3;  // 3 represents car covariance
        car_cov_marker.type = visualization_msgs::msg::Marker::SPHERE;
        car_cov_marker.action = visualization_msgs::msg::Marker::ADD;
        car_cov_marker.pose.position.x = x;
        car_cov_marker.pose.position.y = y;
        car_cov_marker.pose.position.z = 0;
        car_cov_marker.scale.x = 3 * sqrt(abs(ekf_slam.get_cov()(0, 0)));
        car_cov_marker.scale.y = 3 * sqrt(abs(ekf_slam.get_cov()(1, 1)));
        car_cov_marker.scale.z = 0.05;
        car_cov_marker.color.r = 0.1;
        car_cov_marker.color.g = 0.1;
        car_cov_marker.color.b = 0.1;
        car_cov_marker.color.a = 0.3;
        marker_array.markers.push_back(car_cov_marker);

        for (int i = CAR_STATE_SIZE; i < ekf_slam.get_mu().rows(); i += LANDMARK_STATE_SIZE) {
            auto cone_pos_marker = visualization_msgs::msg::Marker();
            cone_pos_marker.header.frame_id = "map";
            cone_pos_marker.header.stamp = stamp;
            cone_pos_marker.ns = "cone_pos";
            cone_pos_marker.id = i;
            cone_pos_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            cone_pos_marker.action = visualization_msgs::msg::Marker::ADD;
            cone_pos_marker.pose.position.x = ekf_slam.get_mu()(i, 0);
            cone_pos_marker.pose.position.y = ekf_slam.get_mu()(i + 1, 0);
            cone_pos_marker.pose.position.z = 0;
            cone_pos_marker.scale.x = 0.2;
            cone_pos_marker.scale.y = 0.2;
            cone_pos_marker.scale.z = 0.5;
            cone_pos_marker.color.r = colours[i].r;
            cone_pos_marker.color.g = colours[i].g;
            cone_pos_marker.color.b = colours[i].b;
            cone_pos_marker.color.a = 1.0;
            marker_array.markers.push_back(cone_pos_marker);

            auto cone_cov_marker = visualization_msgs::msg::Marker();
            cone_cov_marker.header.frame_id = "map";
            cone_cov_marker.header.stamp = stamp;
            cone_cov_marker.ns = "cone_cov";
            cone_cov_marker.id = i;
            cone_cov_marker.type = visualization_msgs::msg::Marker::SPHERE;
            cone_cov_marker.action = visualization_msgs::msg::Marker::ADD;
            cone_cov_marker.pose.position.x = ekf_slam.get_mu()(i, 0);
            cone_cov_marker.pose.position.y = ekf_slam.get_mu()(i + 1, 0);
            cone_cov_marker.pose.position.z = 0;
            cone_cov_marker.scale.x = 3 * sqrt(abs(ekf_slam.get_cov()(i, i)));
            cone_cov_marker.scale.y = 3 * sqrt(abs(ekf_slam.get_cov()(i + 1, i + 1)));
            cone_cov_marker.scale.z = 0.05;
            cone_cov_marker.color.r = 0.1;
            cone_cov_marker.color.g = 0.1;
            cone_cov_marker.color.b = 0.1;
            cone_cov_marker.color.a = 0.3;
            marker_array.markers.push_back(cone_cov_marker);
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
