#include <pure_pursuit.hpp>

Point PurePursuit::get_rvwp(Pose const &car_axle_pos) const {
    // Find the path point closest the to position of the car's axle.
    auto closest_dist = DBL_MAX;
    int closest_index = -1;
    for (size_t i = 0; i < path.size(); i++) {
        auto const &point = path.at(i);
        double diff_x = point.x - car_axle_pos.x;
        double diff_y = point.y - car_axle_pos.y;
        double dist = diff_x * diff_x + diff_y * diff_y;
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }

    // Note that it is logically impossible for closest_index to be -1 (no closest point found) as this method
    // is called after a path.size() > 0 check.
    Point closest_path_point = closest_index > -1 ? path.at(closest_index) : Point(car_axle_pos.x, car_axle_pos.y);
    if (closest_index == -1) RCLCPP_WARN(get_logger_(), "Could not find closest point, have used car's axle pos");

    auto last_dist = DBL_MAX;
    int rvwp_index = -1;
    for (size_t i = 0; i < path.size(); i++) {
        auto const &point = path.at(i);
        double diff_x = point.x - closest_path_point.x;
        double diff_y = point.y - closest_path_point.y;
        double dist = diff_x * diff_x + diff_y * diff_y;
        if (dist <= this->squared_lookahead || dist >= last_dist) continue;

        double a = angle(closest_path_point.x, closest_path_point.y, point.x, point.y);
        double error = wrap_to_pi(car_axle_pos.yaw - a);
        if (error > -M_PI_2 && error < M_PI_2) {
            last_dist = dist;
            rvwp_index = i;
        }
    }

    if (rvwp_index == -1) {
        // Handle path points size overflow.
        int path_points_count = path.size() - 1;
        if (closest_index + fallback_path_points_offset > path_points_count) {
            rvwp_index = abs(path_points_count - (closest_index + fallback_path_points_offset));
        } else {
            rvwp_index = closest_index + fallback_path_points_offset;
        }
    }
    return path.at(rvwp_index);
}

void PurePursuit::timer_callback() {
    // Listens to odom->base_footprint transform and updates the state.
    // Solution is to get the delta and add it to the previous state and subtract the delta from the previous
    // map->odom transform.
    if (!this->can_drive()) {
        return;
    }
    RCLCPP_INFO_ONCE(get_logger_(), "Can drive, starting pursuit");

    auto start_time = std::chrono::high_resolution_clock::now();

    geometry_msgs::msg::TransformStamped odom_to_base;
    try {
        odom_to_base = buffer->lookupTransform("track", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException const &e) {
        RCLCPP_WARN(get_logger_(), "Transform Exception: %s", e.what());
        return;
    }

    tf2::Quaternion q(odom_to_base.transform.rotation.x, odom_to_base.transform.rotation.y,
                      odom_to_base.transform.rotation.z, odom_to_base.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Pose position = get_wheel_position(odom_to_base.transform.translation.x, odom_to_base.transform.translation.y, yaw);

    double steering_angle = 0.0;
    double velocity = 0.0;

    // try {
    //     // Check if the car is behind the RVWP
    //     Point rvwp = get_rvwp(position);
    //     if (rvwp.x < position.x) {
    //         throw std::logic_error("RVWP is behind the car");
    //     }
    //     steering_angle = calc_steering(position, rvwp);
    //     velocity = calc_velocity(steering_angle);

    //     // publish rvwp
    //     geometry_msgs::msg::PointStamped rvwp_msg;
    //     rvwp_msg.header.stamp = now();
    //     rvwp_msg.header.frame_id = "track";
    //     rvwp_msg.point.x = rvwp.x;
    //     rvwp_msg.point.y = rvwp.y;
    //     this->rvwp_pub->publish(rvwp_msg);

    // } catch (std::logic_error const& e) {
    //     RCLCPP_ERROR(get_logger(), "Logic Error: %s", e.what());
    // }
    // Check if the car is behind the RVWP
    Point rvwp = get_rvwp(position);
    steering_angle = calc_steering(position, rvwp);
    velocity = calc_velocity(steering_angle);

    // publish rvwp
    geometry_msgs::msg::PointStamped rvwp_msg;
    rvwp_msg.header.stamp = get_now_();
    rvwp_msg.header.frame_id = "track";
    rvwp_msg.point.x = rvwp.x;
    rvwp_msg.point.y = rvwp.y;
    this->rvwp_pub->publish(rvwp_msg);

    // publish drive message
    ackermann_msgs::msg::AckermannDriveStamped driving_command;
    driving_command.header.stamp = get_now_();
    driving_command.drive.speed = velocity;
    driving_command.drive.steering_angle = steering_angle;
    this->driving_command_pub->publish(driving_command);

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = now - start_time;
    RCLCPP_DEBUG(get_logger_(), "Elapsed time: %f", duration.count());
}

void PurePursuit::path_callback(nav_msgs::msg::Path const &spline_path_msg) {
    double distance = 0.0;
    path.clear();
    for (const auto &pose : spline_path_msg.poses) {
        if (path.size() >= 2) {
            auto const &previous_point = path.at(path.size() - 1);
            auto diff_x = previous_point.x - pose.pose.position.x;
            auto diff_y = previous_point.y - pose.pose.position.y;
            distance += sqrt(diff_x * diff_x + diff_y * diff_y);
        }
        path.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    // Calculate the number of path points to skip when finding a fallback RVWP.
    // Formula is the number of path points divided by the calculated distance in metres, giving the number of
    // points per metre, which is then multiplied by the lookahead value (also in metres) giving the number of
    // path points that shuld be skipped.
    fallback_path_points_offset = round(path.size() / distance * lookahead);
}

void PurePursuit::initialise_params(rclcpp::Node::SharedPtr node) {
    kp_ang = node->declare_parameter<double>("kp_ang", -3.0);
    vel_max = node->declare_parameter<double>("vel_max", 5.0);
    vel_min = node->declare_parameter<double>("vel_min", 2.0);
    lookahead = node->declare_parameter<double>("lookahead", 3.0);
    discovery_lookahead = node->declare_parameter<double>("discovery_lookahead", 3.5);
    discovery_vel_max = node->declare_parameter<double>("discovery_vel_max", 3.0);
    discovery_vel_min = node->declare_parameter<double>("discovery_vel_min", 2.0);

    kp_ang = node->get_parameter("kp_ang").as_double();
    lookahead = node->get_parameter("discovery_lookahead").as_double();
    vel_max = node->get_parameter("discovery_vel_max").as_double();
    vel_min = node->get_parameter("discovery_vel_min").as_double();

    squared_lookahead = lookahead * lookahead;
}

void PurePursuit::initialise_params(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    kp_ang = node->declare_parameter<double>("kp_ang", -3.0);
    vel_max = node->declare_parameter<double>("vel_max", 5.0);
    vel_min = node->declare_parameter<double>("vel_min", 2.0);
    lookahead = node->declare_parameter<double>("lookahead", 3.0);
    discovery_lookahead = node->declare_parameter<double>("discovery_lookahead", 3.5);
    discovery_vel_max = node->declare_parameter<double>("discovery_vel_max", 3.0);
    discovery_vel_min = node->declare_parameter<double>("discovery_vel_min", 2.0);

    kp_ang = node->get_parameter("kp_ang").as_double();
    lookahead = node->get_parameter("discovery_lookahead").as_double();
    vel_max = node->get_parameter("discovery_vel_max").as_double();
    vel_min = node->get_parameter("discovery_vel_min").as_double();

    squared_lookahead = lookahead * lookahead;
}

PurePursuit::PurePursuit(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer,
                         std::shared_ptr<tf2_ros::TransformListener> listener) {
    this->node = node;
    this->buffer = buffer;
    this->listener = listener;
    initialise_params(this->node);
}

PurePursuit::PurePursuit(rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node, tf2_ros::Buffer::SharedPtr buffer,
                         std::shared_ptr<tf2_ros::TransformListener> listener) {
    this->lifecycle_node = lifecycle_node;
    this->buffer = buffer;
    this->listener = listener;
    initialise_params(this->lifecycle_node);
}
