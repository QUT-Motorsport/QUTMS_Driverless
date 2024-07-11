#include "node_sbg_translator.hpp"

SBGTranslate::SBGTranslate() : Node("sbg_translator_node") {
    // subscribe to ekf odom
    this->ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/imu/odometry", 1, std::bind(&SBGTranslate::ekf_odom_callback, this, _1));

    // Odometry
    this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sbg_translated/odometry", 1);

    // // Pose (for visuals)
    // this->pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sbg_translated/pose", 1);
    // this->raw_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu/pose", 1);

    // // Pose path (for visuals)
    // this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sbg_translated/path_odom", 1);

    state_ = {0.0, 0.0, 0.0};

    last_pub_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "---SBG Odom Converter Node Initialised---");
}


double SBGTranslate::filer_yaw(double x, double y) {
    // remove outliers from yaw with a kalman filter

    double Q = 0.4;
    double R = 0.1;

    // measurement update
    // double x_delta = x - state_[0];
    // double y_delta = y - state_[1];

    // add to circular buffer
    update_pos_buffer(state_[0], state_[1]);

    // get the angle between the two vectors
    // use average of the last 10 deltas
    // double yaw_vec_angle = atan2(
    //     std::accumulate(y_deltas_.begin(), y_deltas_.end(), 0.0) / y_deltas_.size(),
    //     std::accumulate(x_deltas_.begin(), x_deltas_.end(), 0.0) / x_deltas_.size()
    // );
    // use the difference between the first and last element
    double yaw_vec_angle = atan2(y_deltas_.back() - y_deltas_.front(), x_deltas_.back() - x_deltas_.front());
    yaw_vec_angle = wrap_to_pi(yaw_vec_angle);

    double K = yaw_cov_ / (yaw_cov_ + R);

    double yaw_update = state_[2] + K * (yaw_vec_angle - state_[2]);
    yaw_update = wrap_to_pi(yaw_update);

    // RCLCPP_INFO(this->get_logger(), "Yaw measurement: %f, Yaw update: %f, cov: %f, K: %f", yaw_vec_angle,
    // yaw_vec_angle - state_[2], yaw_cov_, K);

    // update covariance
    yaw_cov_ = (1 - K) * yaw_cov_;

    // prediction
    // x = x + v * dt
    double yaw_pred = yaw_update + last_yaw_rate_ * (this->now() - last_time_).seconds();
    yaw_pred = wrap_to_pi(yaw_pred);  // -pi to pi

    yaw_cov_ += Q;  // process noise, continues to increase the covariance if measurement is ignored

    // RCLCPP_INFO(this->get_logger(), "Yaw pred: %f, cov: %f", yaw_pred, yaw_cov_);

    // update the states
    state_[0] = x;
    state_[1] = y;
    last_yaw_vec_angle_ = yaw_vec_angle;

    return yaw_pred;
}

void SBGTranslate::ekf_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!received_odom_) {
        // initialize position and yaw
        state_[0] = msg->pose.pose.position.x;
        state_[1] = msg->pose.pose.position.y;
        state_[2] = 0.0;

        state_[0] = -msg->pose.pose.position.y;
        state_[1] = msg->pose.pose.position.x;

        init_x = state_[0];
        init_y = state_[1];

        last_yaw_rate_ = 0.0;
        last_time_ = this->now();

        received_odom_ = true;
        return;
    }

    // flip x and y and invert the yaw
    nav_msgs::msg::Odometry odom_msg = *msg;

    odom_msg.pose.pose.position.x = -msg->pose.pose.position.y;
    odom_msg.pose.pose.position.y = msg->pose.pose.position.x;

    odom_msg.twist.twist.linear.x = -msg->twist.twist.linear.y;
    odom_msg.twist.twist.linear.y = msg->twist.twist.linear.x;

    double yaw = filer_yaw(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    if (state_[0] < init_x + 5.0) {
        yaw = 0.0;
    }
    odom_msg.pose.pose.orientation = euler_to_quat(0.0, 0.0, yaw);

    // get angular rate between current and last
    odom_msg.twist.twist.angular.z = wrap_to_pi(yaw - state_[2]) / (this->now() - last_time_).seconds();

    // update the last time
    last_time_ = this->now();
    last_yaw_rate_ = -msg->twist.twist.angular.z;
    last_yaw_change_ = odom_msg.twist.twist.angular.z;
    state_[2] = yaw;

    odom_pub_->publish(odom_msg);

    // // for foxglove visuals
    // // publish the odom msg
    // odom_pub_->publish(odom_msg);

    // // publish pose message
    // pose_pub_->publish(make_pose_msg(odom_msg));

    // // throttle path pub
    // // rclcpp time
    // if (this->now() - last_pub_time_ < rclcpp::Duration(std::chrono::seconds(1) / 10)) {
    //     return;
    // }

    // // keep 50 points in the path
    // if (path_msg_.poses.size() > 50) {
    //     path_msg_.poses.erase(path_msg_.poses.begin());
    // }
    // // update the path
    // path_msg_.header = odom_msg.header;
    // path_msg_.header.frame_id = "track";
    // path_msg_.poses.push_back(make_pose_msg(odom_msg));
    // path_pub_->publish(path_msg_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGTranslate>());
    rclcpp::shutdown();
    return 0;
}
