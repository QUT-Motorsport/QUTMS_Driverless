#include "map_creation/node_cone_placement.hpp"

namespace map_creation {

ConeAssociation::ConeAssociation() : Node("cone_placement_node") {
    // Create subscribers
    lidar_subscriber = create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
        "/lidar/cone_detection", 1, std::bind(&ConeAssociation::callback, this, std::placeholders::_1));
    vision_subscriber = create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
        "/vision/cone_detection", 1, std::bind(&ConeAssociation::callback, this, std::placeholders::_1));
    state_subscriber = create_subscription<driverless_msgs::msg::State>(
        "/system/as_status", 1, std::bind(&ConeAssociation::state_callback, this, std::placeholders::_1));

    // Create publishers
    global_publisher = create_publisher<driverless_msgs::msg::ConeDetectionStamped>("/slam/global_map", 1);
    local_publisher = create_publisher<driverless_msgs::msg::ConeDetectionStamped>("/slam/local_map", 1);

    // Initialize TF2 buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, this);

    // params
    declare_parameter<double>("view_x", 10.0);
    declare_parameter<double>("view_y", 10.0);
    declare_parameter<double>("radius", 5.0);
    declare_parameter<int>("min_detections", 10);

    view_x = get_parameter("view_x").as_double();
    view_y = get_parameter("view_y").as_double();
    radius = get_parameter("radius").as_double();
    min_detections = get_parameter("min_detections").as_int();

    RCLCPP_INFO(get_logger(), "---SLAM node initialised---");
    RCLCPP_INFO(get_logger(), "PARAMS: view_x: %f, view_y: %f, radius: %f, min_detections: %d", view_x, view_y, radius,
                min_detections);
}

void ConeAssociation::state_callback(const driverless_msgs::msg::State::SharedPtr msg) {
    // we haven't started driving yet
    if (msg->state == driverless_msgs::msg::State::DRIVING && msg->lap_count == 0) {
        mapping = true;
    }

    // we have finished mapping
    if (msg->lap_count > 0 && mapping) {
        RCLCPP_INFO_ONCE(get_logger(), "Lap completed, mapping completed");
        // mapping = false;
    }
}

void ConeAssociation::callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr msg) {
    if (!mapping) {
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    // skip if no transform received
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer->lookupTransform("track", "base_footprint", rclcpp::Time(0));
    } catch (tf2::TransformException &e) {
        RCLCPP_INFO(get_logger(), "Transform exception: %s", e.what());
        return;
    }

    // get the transform values
    tf2::Quaternion q;
    tf2::fromMsg(map_to_base.transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // update the last pose
    pose = {map_to_base.transform.translation.x, map_to_base.transform.translation.y, yaw};

    // process detected cones
    // search through the list of TrackedCones and update the location of the cone if it is within a radius
    // if not, add it to the list
    for (const auto &cone : msg->cones) {
        findClosestCone(cone, msg->header.frame_id);
    }

    // if no cones were detected, return
    if (track.empty()) {
        return;
    }

    // create and publish map cone detection msgs
    createConeDetections(*msg);

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Process time (ns): %ld, cones: %ld", elapsed_time,
                         track.size());
}

void ConeAssociation::findClosestCone(const driverless_msgs::msg::Cone &cone, std::string frame_id) {
    // find the closest cone on the map to the detected cone
    // if it is within a radius, update the location of the cone
    // otherwise, add it to the list

    // create a tracked cone
    auto current_cone = TrackedCone(cone, frame_id, pose);
    auto closest_dist = DBL_MAX;
    int closest_index = -1;
    for (size_t i = 0; i < track.size(); i++) {
        auto const &tracked_cone = track.at(i);
        double distance = dist(current_cone.map_x, current_cone.map_y, tracked_cone.map_x, tracked_cone.map_y);
        if (distance < closest_dist && distance < radius) {
            closest_dist = distance;
            closest_index = i;
        }
    }

    // check if we found a cone within the radius
    if (closest_index != -1) {
        // update the location of the cone using the average of the previous location and the new location
        auto &tracked_cone = track.at(closest_index);
        tracked_cone.update(current_cone, pose);
    } else {
        if (current_cone.sensor == "lidar") {
            // add the cone to the list
            track.push_back(current_cone);
        }
    }
}

void ConeAssociation::createConeDetections(const driverless_msgs::msg::ConeDetectionStamped &msg) {
    auto global_msg = std::make_shared<driverless_msgs::msg::ConeDetectionStamped>();
    global_msg->header.stamp = msg.header.stamp;
    global_msg->header.frame_id = "track";

    auto local_msg = std::make_shared<driverless_msgs::msg::ConeDetectionStamped>();
    local_msg->header.stamp = msg.header.stamp;
    local_msg->header.frame_id = "base_footprint";

    for (const auto &cone : track) {
        // ensure that the cone has been detected enough
        // if (!cone.confirmed) {
        //     continue;
        // }
        if (cone.frame_count < min_detections) {
            continue;
        }

        auto cone_msg = cone.cone_as_msg();
        global_msg->cones.push_back(cone_msg);

        auto cone_with_cov = std::make_shared<driverless_msgs::msg::ConeWithCovariance>();
        cone_with_cov->cone = cone_msg;
        cone_with_cov->covariance = cone.cov_as_msg();

        global_msg->cones_with_cov.push_back(*cone_with_cov);

        if (cone.local_x < 0 || cone.local_x > view_x || cone.local_y < -view_y || cone.local_y > view_y) {
            continue;
        }

        auto local_cone_msg = cone.local_cone_as_msg();
        local_msg->cones.push_back(local_cone_msg);

        auto local_cone_with_cov = std::make_shared<driverless_msgs::msg::ConeWithCovariance>();
        local_cone_with_cov->cone = local_cone_msg;
        local_cone_with_cov->covariance = cone.cov_as_msg();

        local_msg->cones_with_cov.push_back(*local_cone_with_cov);
    }

    global_publisher->publish(*global_msg);
    local_publisher->publish(*local_msg);
}
};  // namespace map_creation

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_creation::ConeAssociation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
