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

    rclcpp::Subscription<driverless_msgs::msg::WSSVelocity>::SharedPtr wss_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr detection_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    rclcpp::Publisher<driverless_msgs::msg::DebugMsg>::SharedPtr debug_1_pub;
    rclcpp::Publisher<driverless_msgs::msg::DebugMsg>::SharedPtr debug_2_pub;
    rclcpp::Publisher<driverless_msgs::msg::DoubleMatrix>::SharedPtr matrix_pub;

   public:
    EKFSLAMNode() : Node("ekf_node") {
        twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "imu/velocity", 10, std::bind(&EKFSLAMNode::twist_callback, this, _1));

        detection_sub = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "vision/cone_detection", 10, std::bind(&EKFSLAMNode::cone_detection_callback, this, _1));

        viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ekf_visualisation", 10);
        debug_1_pub = this->create_publisher<driverless_msgs::msg::DebugMsg>("debug_1", 10);
        debug_2_pub = this->create_publisher<driverless_msgs::msg::DebugMsg>("debug_2", 10);
        matrix_pub = this->create_publisher<driverless_msgs::msg::DoubleMatrix>("matrix", 10);
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        forward_vel = msg->twist.linear.x;
        rotational_vel = msg->twist.angular.z;
        predict(msg->header.stamp);
    }

    void predict(rclcpp::Time stamp) {
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

        publish_visualisations(stamp);
    }

    void cone_detection_callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr detection_msg) {
        rclcpp::Time stamp = detection_msg->header.stamp;

        if (!last_update.has_value()) {
            last_update = stamp;
            return;
        }

        double dt = compute_dt(last_update.value(), stamp);

        if (!(dt > 0)) {
            return;
        }

        ekf_slam.predict(forward_vel, rotational_vel, dt);
        // ekf_slam.update(
        //     detection_msg->cones_with_cov,
        //     debug_1_pub,
        //     debug_2_pub,
        //     matrix_pub
        // );

        last_update = stamp;

        publish_visualisations(detection_msg->header.stamp);
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
