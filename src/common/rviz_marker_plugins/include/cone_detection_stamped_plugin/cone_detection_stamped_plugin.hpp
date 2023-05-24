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

#ifndef RVIZ_MARKER_PLUGINS_INCLUDE_CONE_DETECTION_STAMPED_PLUGIN_CONE_DETECTION_STAMPED_PLUGIN_HPP_
#define RVIZ_MARKER_PLUGINS_INCLUDE_CONE_DETECTION_STAMPED_PLUGIN_CONE_DETECTION_STAMPED_PLUGIN_HPP_

#include <driverless_msgs/msg/cone_detection_stamped.hpp>
#include <driverless_msgs/msg/cone_with_covariance.hpp>
#include <memory>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_marker_plugins {
namespace displays {

enum ConeColorOption { CONE = 0, FLAT = 1 };

class ConeDetectionStampedPlugin : public rviz_common::RosTopicDisplay<driverless_msgs::msg::ConeDetectionStamped> {
    Q_OBJECT

   public:
    ConeDetectionStampedPlugin();

    void onInitialize() override;

    void load(const rviz_common::Config &config) override;

    void update(float wall_dt, float ros_dt) override;

    void reset() override;

   private Q_SLOTS:
    void updateColorOption();

   private:
    void initMarkers();

    void setConeMarker(const driverless_msgs::msg::Cone &cone, const std_msgs::msg::Header &header,
                       const int &id, visualization_msgs::msg::Marker *marker);

    visualization_msgs::msg::Marker getColoredMarker(visualization_msgs::msg::Marker cone_marker);

    void setCovarianceMarker(const driverless_msgs::msg::ConeWithCovariance &cone, const std_msgs::msg::Header &header,
                             const int &id);

    void setMarkerArray(const driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr &msg);

    void processMessage(driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr msg) override;

    int id_;

    ConeColorOption cone_color_option_;

    std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;

    rviz_common::properties::EnumProperty *color_option_property_;
    rviz_common::properties::ColorProperty *color_property_;

    visualization_msgs::msg::Marker blue_cone_marker_;

    visualization_msgs::msg::Marker yellow_cone_marker_;

    visualization_msgs::msg::Marker orange_cone_marker_;

    visualization_msgs::msg::Marker big_orange_cone_marker_;

    visualization_msgs::msg::Marker unknown_cone_marker_;

    visualization_msgs::msg::Marker covariance_marker_;

    visualization_msgs::msg::Marker delete_all_marker_;

    visualization_msgs::msg::MarkerArray marker_array_;
};

}
}

#endif
