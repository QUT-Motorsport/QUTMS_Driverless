/// \brief Acknowledgement of originality
// MIT License

// Copyright (c) 2020 Edinburgh University Formula Student

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

#include "cone_detection_stamped_plugin/cone_detection_stamped_plugin.hpp"

#include <Eigen/Eigen>

namespace rviz_marker_plugins {
namespace displays {

PLUGINLIB_EXPORT_CLASS(rviz_marker_plugins::displays::ConeDetectionStampedPlugin, rviz_common::Display)

ConeDetectionStampedPlugin::ConeDetectionStampedPlugin()
    : rviz_common::RosTopicDisplay<driverless_msgs::msg::ConeDetectionStamped>(),
      id_(0),
      marker_common_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this)) {
    color_option_property_ = new rviz_common::properties::EnumProperty("Color Display", "Cone", "Cone colour to use",
                                                                       this, SLOT(updateColorOption()));
    color_option_property_->addOption("Cone", ConeColorOption::CONE);
    color_option_property_->addOption("Flat", ConeColorOption::FLAT);

    color_property_ =
        new rviz_common::properties::ColorProperty("Color", QColor(200, 200, 200), "Color of cones to display", this);
    color_property_->hide();
}

void ConeDetectionStampedPlugin::updateColorOption() {
    ConeColorOption color_option = static_cast<ConeColorOption>(color_option_property_->getOptionInt());
    cone_color_option_ = color_option;
    switch (color_option) {
        case CONE:
            color_property_->hide();
            break;
        case FLAT:
            color_property_->show();
            break;
    }
}

void ConeDetectionStampedPlugin::onInitialize() {
    RTDClass::onInitialize();
    marker_common_->initialize(context_, scene_node_);

    topic_property_->setValue("/cones");
    topic_property_->setDescription("driverless_msgs::msg::ConeDetectionStamped topic to subscribe to.");

    initMarkers();
}

void ConeDetectionStampedPlugin::load(const rviz_common::Config &config) {
    Display::load(config);
    marker_common_->load(config);
}

void ConeDetectionStampedPlugin::processMessage(driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr msg) {
    delete_all_marker_.header = msg->header;
    delete_all_marker_.id = id_;
    marker_array_.markers.push_back(delete_all_marker_);

    marker_common_->addMessage(std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

    marker_array_.markers.clear();

    setMarkerArray(msg);
    marker_common_->addMessage(std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

    marker_array_.markers.clear();
}

void ConeDetectionStampedPlugin::update(float wall_dt, float ros_dt) { marker_common_->update(wall_dt, ros_dt); }

void ConeDetectionStampedPlugin::reset() {
    RosTopicDisplay::reset();
    marker_common_->clearMarkers();
}

void ConeDetectionStampedPlugin::initMarkers() {
    delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

    blue_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
    blue_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    blue_cone_marker_.pose.orientation.x = 0.0;
    blue_cone_marker_.pose.orientation.y = 0.0;
    blue_cone_marker_.pose.orientation.z = 0.0;
    blue_cone_marker_.pose.orientation.w = 1.0;
    blue_cone_marker_.scale.x = 1.0;
    blue_cone_marker_.scale.y = 1.0;
    blue_cone_marker_.scale.z = 1.0;
    blue_cone_marker_.mesh_resource = "package://rviz_marker_plugins/meshes/cone.dae";
    blue_cone_marker_.color.r = 0.0;
    blue_cone_marker_.color.g = 0.0;
    blue_cone_marker_.color.b = 1.0;
    blue_cone_marker_.color.a = 1.0;
    blue_cone_marker_.ns = "cone";

    yellow_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
    yellow_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    yellow_cone_marker_.pose.orientation.x = 0.0;
    yellow_cone_marker_.pose.orientation.y = 0.0;
    yellow_cone_marker_.pose.orientation.z = 0.0;
    yellow_cone_marker_.pose.orientation.w = 1.0;
    yellow_cone_marker_.scale.x = 1.0;
    yellow_cone_marker_.scale.y = 1.0;
    yellow_cone_marker_.scale.z = 1.0;
    yellow_cone_marker_.mesh_resource = "package://rviz_marker_plugins/meshes/cone.dae";
    yellow_cone_marker_.color.r = 1.0;
    yellow_cone_marker_.color.g = 1.0;
    yellow_cone_marker_.color.b = 0.0;
    yellow_cone_marker_.color.a = 1.0;
    yellow_cone_marker_.ns = "cone";

    orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
    orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    orange_cone_marker_.pose.orientation.x = 0.0;
    orange_cone_marker_.pose.orientation.y = 0.0;
    orange_cone_marker_.pose.orientation.z = 0.0;
    orange_cone_marker_.pose.orientation.w = 1.0;
    orange_cone_marker_.scale.x = 1.0;
    orange_cone_marker_.scale.y = 1.0;
    orange_cone_marker_.scale.z = 1.0;
    orange_cone_marker_.mesh_resource = "package://rviz_marker_plugins/meshes/cone.dae";
    orange_cone_marker_.color.r = 1.0;
    orange_cone_marker_.color.g = 0.549;
    orange_cone_marker_.color.b = 0.0;
    orange_cone_marker_.color.a = 1.0;
    orange_cone_marker_.ns = "cone";

    big_orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
    big_orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    big_orange_cone_marker_.pose.orientation.x = 0.0;
    big_orange_cone_marker_.pose.orientation.y = 0.0;
    big_orange_cone_marker_.pose.orientation.z = 0.0;
    big_orange_cone_marker_.pose.orientation.w = 1.0;
    big_orange_cone_marker_.scale.x = 1.0;
    big_orange_cone_marker_.scale.y = 1.0;
    big_orange_cone_marker_.scale.z = 1.0;
    big_orange_cone_marker_.mesh_resource = "package://rviz_marker_plugins/meshes/big_cone.dae";
    big_orange_cone_marker_.color.r = 1.0;
    big_orange_cone_marker_.color.g = 0.271;
    big_orange_cone_marker_.color.b = 0.0;
    big_orange_cone_marker_.color.a = 1.0;
    big_orange_cone_marker_.ns = "cone";

    unknown_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    unknown_cone_marker_.pose.orientation.x = 0.0;
    unknown_cone_marker_.pose.orientation.y = 0.0;
    unknown_cone_marker_.pose.orientation.z = 0.0;
    unknown_cone_marker_.pose.orientation.w = 1.0;
    unknown_cone_marker_.scale.x = 1.0;
    unknown_cone_marker_.scale.y = 1.0;
    unknown_cone_marker_.scale.z = 1.0;
    unknown_cone_marker_.mesh_resource = "package://rviz_marker_plugins/meshes/cone.dae";
    unknown_cone_marker_.color.r = 0.0;
    unknown_cone_marker_.color.g = 1.0;
    unknown_cone_marker_.color.b = 0.0;
    unknown_cone_marker_.color.a = 1.0;
    unknown_cone_marker_.ns = "cone";

    covariance_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    covariance_marker_.pose.orientation.x = 0.0;
    covariance_marker_.pose.orientation.y = 0.0;
    covariance_marker_.pose.orientation.z = 0.0;
    covariance_marker_.pose.orientation.w = 1.0;
    covariance_marker_.scale.x = 1.0;
    covariance_marker_.scale.y = 1.0;
    covariance_marker_.scale.z = 1.0;
    covariance_marker_.color.r = 1.0;
    covariance_marker_.color.g = 0.271;
    covariance_marker_.color.b = 0.271;
    covariance_marker_.color.a = 0.7;
    covariance_marker_.ns = "covariance";
}

void ConeDetectionStampedPlugin::setConeMarker(const driverless_msgs::msg::Cone &cone,
                                               const std_msgs::msg::Header &header, const int &id,
                                               visualization_msgs::msg::Marker *marker) {
    marker->id = id;
    marker->header = header;
    marker->pose.position.x = cone.location.x;
    marker->pose.position.y = cone.location.y;
    marker->pose.position.z = cone.location.z;
}

void ConeDetectionStampedPlugin::setCovarianceMarker(const driverless_msgs::msg::ConeWithCovariance &cone,
                                                     const std_msgs::msg::Header &header, const int &id) {
    // https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
    covariance_marker_.id = id;
    covariance_marker_.header = header;
    covariance_marker_.pose.position.x = cone.cone.location.x;
    covariance_marker_.pose.position.y = cone.cone.location.y;
    covariance_marker_.pose.position.z = cone.cone.location.z;

    // Convert the covariance message to a matrix
    Eigen::Matrix2d cov_matrix;
    cov_matrix << static_cast<float>(cone.covariance[0]), static_cast<float>(cone.covariance[1]),
        static_cast<float>(cone.covariance[2]), static_cast<float>(cone.covariance[3]);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov_matrix);
    const Eigen::Vector2d &eigValues(eig.eigenvalues());
    const Eigen::Matrix2d &eigVectors(eig.eigenvectors());
    double x_scale = 2 * std::sqrt(5.991 * eigValues[0]);
    double y_scale = 2 * std::sqrt(5.991 * eigValues[1]);
    covariance_marker_.scale.x = x_scale;
    covariance_marker_.scale.y = y_scale;
    covariance_marker_.scale.z = 0.01;
    // Angle between x-axis and first eigenvector
    double angle = std::atan2(eigVectors(1, 0), eigVectors(0, 0));
    // Rotate ellipse such that x-axis is aligned with first eignevector
    // See: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
    covariance_marker_.pose.orientation.x = 0;
    covariance_marker_.pose.orientation.y = 0;
    covariance_marker_.pose.orientation.z = std::sin(angle * 0.5);
    covariance_marker_.pose.orientation.w = std::cos(angle * 0.5);
}

visualization_msgs::msg::Marker ConeDetectionStampedPlugin::getColoredMarker(
    visualization_msgs::msg::Marker cone_marker) {
    visualization_msgs::msg::Marker marker = cone_marker;
    switch (cone_color_option_) {
        case FLAT: {
            QColor color = color_property_->getColor();
            marker.color.r = static_cast<float>(color.red()) / 255.0f;
            marker.color.g = static_cast<float>(color.green()) / 255.0f;
            marker.color.b = static_cast<float>(color.blue()) / 255.0f;
        }
        default:
            break;
    }
    return marker;
}

void ConeDetectionStampedPlugin::setMarkerArray(const driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr &msg) {
    for (const auto &cone : msg->cones_with_cov) {
        visualization_msgs::msg::Marker cone_marker;
        switch (cone.cone.color) {
            case driverless_msgs::msg::Cone::BLUE:
                cone_marker = blue_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::YELLOW:
                cone_marker = yellow_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::ORANGE_BIG:
                cone_marker = big_orange_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::ORANGE_SMALL:
                cone_marker = orange_cone_marker_;
                break;
            default:
                cone_marker = unknown_cone_marker_;
                break;
        }
        setConeMarker(cone.cone, msg->header, id_, &cone_marker);
        setCovarianceMarker(cone, msg->header, id_);
        auto marker = getColoredMarker(cone_marker);
        marker_array_.markers.push_back(marker);
        marker_array_.markers.push_back(covariance_marker_);
        id_++;
    }
    for (const auto &cone : msg->cones) {
        visualization_msgs::msg::Marker cone_marker;
        switch (cone.color) {
            case driverless_msgs::msg::Cone::BLUE:
                cone_marker = blue_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::YELLOW:
                cone_marker = yellow_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::ORANGE_BIG:
                cone_marker = big_orange_cone_marker_;
                break;
            case driverless_msgs::msg::Cone::ORANGE_SMALL:
                cone_marker = orange_cone_marker_;
                break;
            default:
                cone_marker = unknown_cone_marker_;
                break;
        }
        setConeMarker(cone, msg->header, id_, &cone_marker);
        auto marker = getColoredMarker(cone_marker);
        marker_array_.markers.push_back(marker);
        marker_array_.markers.push_back(covariance_marker_);
        id_++;
    }
}

}  // namespace displays
}  // namespace rviz_marker_plugins
