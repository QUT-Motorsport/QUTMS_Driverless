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

    Eigen::MatrixXd pred_mu;   // predicted state (mean, μ bar)
    Eigen::MatrixXd pred_cov;  // predicted state (covariance, ∑ bar)

    Eigen::MatrixXd mu;   // final state (mean, μ)
    Eigen::MatrixXd cov;  // final state (covariance, ∑)

    std::optional<builtin_interfaces::msg::Time> last_sensed_control_update;
    std::optional<builtin_interfaces::msg::Time> last_cone_update;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "gss", 10, std::bind(&EKFSLAMNode::sensed_control_callback, this, _1));

        detection_sub = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "sim_translator/cone_detection", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_visualisation", 10);
    }

    void sensed_control_callback(const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg) {
        // catch first call where we have no "last update time"
        if (!this->last_sensed_control_update.has_value()) {
            this->last_sensed_control_update = vel_msg->header.stamp;
            return;
        }

        double dt = compute_dt(last_sensed_control_update.value(), vel_msg->header.stamp);
        this->last_sensed_control_update = vel_msg->header.stamp;

        if (dt == 0) {
            return;
        }

        Eigen::Matrix<double, CAR_STATE_SIZE, 1> pred_mu;
        Eigen::Matrix<double, CAR_STATE_SIZE, CAR_STATE_SIZE> pred_car_cov;

        // this->ekf_slam.position_predict(pred_mu, pred_car_cov);

        print_state();
        publish_visualisations(vel_msg->header.stamp);
    }

    void cone_detection_callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr msg) {
        // catch first call where we have no "last update time"
        if (!this->last_cone_update.has_value()) {
            this->last_cone_update = msg->header.stamp;
            return;
        }

        double dt = compute_dt(last_cone_update.value(), msg->header.stamp);
        this->last_cone_update = msg->header.stamp;

        if (dt == 0) {
            return;
        }

        if (!this->last_sensed_control_update.has_value() ||
            compute_dt(last_sensed_control_update.value(), last_cone_update.value()) < 0) {
            return;
        }

        std::cout << "Cone Callback" << std::endl;
        print_state();
        publish_visualisations(msg->header.stamp);
    }

    void print_matricies() {
        std::cout << "pred_mu:\n" << this->pred_mu << "\n" << std::endl;
        std::cout << "pred_cov:\n" << this->pred_cov << "\n" << std::endl;
        std::cout << "mu:\n" << this->mu << "\n" << std::endl;
        std::cout << "cov:\n" << this->cov << "\n" << std::endl;
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
        pred_car_marker.ns = "ekf";
        pred_car_marker.id = -1;  // -1 represents predicted car
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
        car_marker.ns = "ekf";
        car_marker.id = -1;  // -1 represents predicted car
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

        for (int i = CAR_STATE_SIZE; i < mu.rows(); i += LANDMARK_STATE_SIZE) {
            auto cone_marker = visualization_msgs::msg::Marker();
            cone_marker.header.frame_id = "map";
            cone_marker.header.stamp = stamp;
            cone_marker.ns = "ekf";
            cone_marker.id = i;
            cone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            cone_marker.action = visualization_msgs::msg::Marker::ADD;
            cone_marker.pose.position.x = mu(i, 0);
            cone_marker.pose.position.y = mu(i + 1, 0);
            cone_marker.pose.position.z = 0;
            cone_marker.scale.x = 0.2;
            cone_marker.scale.y = 0.2;
            cone_marker.scale.z = 0.5;
            cone_marker.color.r = colours[i].r;
            cone_marker.color.g = colours[i].g;
            cone_marker.color.b = colours[i].b;
            cone_marker.color.a = 1.0;
            marker_array.markers.push_back(cone_marker);
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
