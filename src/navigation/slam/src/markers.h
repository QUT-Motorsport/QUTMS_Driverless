#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "colours.h"
#include "visualization_msgs/msg/marker.hpp"

visualization_msgs::msg::Marker car_marker(builtin_interfaces::msg::Time stamp, std::string ns, int id, double x,
                                           double y, double theta) {
    tf2::Quaternion heading;
    heading.setRPY(0, 0, theta);

    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "track";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    tf2::convert(heading, marker.pose.orientation);
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::msg::Marker cone_marker(builtin_interfaces::msg::Time stamp, std::string ns, int id, double x,
                                            double y, int colour) {
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "track";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    // marker.color.r = colours[id].r;
    // marker.color.g = colours[id].g;
    // marker.color.b = colours[id].b;
    marker.color.r = cone_colours[colour].r;
    marker.color.g = cone_colours[colour].g;
    marker.color.b = cone_colours[colour].b;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::msg::Marker cov_marker(builtin_interfaces::msg::Time stamp, std::string ns, int id, double x,
                                           double y, double cov_x, double cov_y) {
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "track";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.scale.x = 3 * sqrt(abs(cov_x));
    marker.scale.y = 3 * sqrt(abs(cov_y));
    marker.scale.z = 0.05;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 0.3;

    return marker;
}
