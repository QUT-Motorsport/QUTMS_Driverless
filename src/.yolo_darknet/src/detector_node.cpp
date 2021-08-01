// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <darknet_vendor/darknet_vendor.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "openrobotics_darknet_ros/detector_node.hpp"
#include "openrobotics_darknet_ros/detector_network.hpp"
#include "openrobotics_darknet_ros/parse.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/parameter_value.hpp"

namespace openrobotics
{
namespace darknet_ros
{
class DetectorNodePrivate
{
public:
  void on_image_rx(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
  {
    vision_msgs::msg::Detection2DArray::UniquePtr detections(
      new vision_msgs::msg::Detection2DArray);
    // std::cerr << "using threshold " << threshold_ << " nms " << nms_threshold_ << "\n";
    if (network_->detect(*image_msg, threshold_, nms_threshold_, &*detections)) {
      detections_pub_->publish(std::move(detections));
    }
  }

  rcl_interfaces::msg::SetParametersResult
  on_parameters_change(const std::vector<rclcpp::Parameter> & new_values)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    double new_threshold = threshold_;
    double new_nms_threshold = nms_threshold_;

    for (const auto & parameter : new_values) {
      if (threshold_desc_.name == parameter.get_name()) {
        new_threshold = parameter.as_double();
        // TODO(sloretz) range constraints in parameter description
        if (new_threshold < 0.0 || new_threshold > 1.0) {
          result.successful = false;
          result.reason = "threshold out of range [0.0, 1.0]";
        }
      } else if (nms_threshold_desc_.name == parameter.get_name()) {
        new_nms_threshold = parameter.as_double();
        if (new_nms_threshold < 0.0 || new_nms_threshold > 1.0) {
          result.successful = false;
          result.reason = "nms_threshold out of range [0.0, 1.0]";
        }
      }
    }
    if (result.successful) {
      threshold_ = new_threshold;
      nms_threshold_ = new_nms_threshold;
      // std::cerr << "New threshold " << threshold_ << " nms " << nms_threshold_ << "\n";
    }
    return result;
  }

  std::unique_ptr<DetectorNetwork> network_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  double threshold_ = 0.25;
  double nms_threshold_ = 0.45;

  rcl_interfaces::msg::ParameterDescriptor threshold_desc_;
  rcl_interfaces::msg::ParameterDescriptor nms_threshold_desc_;
};

DetectorNode::DetectorNode(rclcpp::NodeOptions options)
: rclcpp::Node("detector_node", options), impl_(new DetectorNodePrivate)
{
  // Read-only input parameters: cfg, weights, classes
  rcl_interfaces::msg::ParameterDescriptor network_cfg_desc;
  network_cfg_desc.description = "Path to config file describing network";
  network_cfg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  network_cfg_desc.read_only = true;
  network_cfg_desc.name = "network.config";
  const std::string network_config_path = declare_parameter(
    network_cfg_desc.name,
    rclcpp::ParameterValue(),
    network_cfg_desc).get<std::string>();

  rcl_interfaces::msg::ParameterDescriptor network_weights_desc;
  network_weights_desc.description = "Path to file describing network weights";
  network_weights_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  network_weights_desc.read_only = true;
  network_weights_desc.name = "network.weights";
  const std::string network_weights_path = declare_parameter(
    network_weights_desc.name,
    rclcpp::ParameterValue(),
    network_weights_desc).get<std::string>();

  rcl_interfaces::msg::ParameterDescriptor network_class_names_desc;
  network_class_names_desc.description = "Path to file with class names (one per line)";
  network_class_names_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  network_class_names_desc.read_only = true;
  network_class_names_desc.name = "network.class_names";
  const std::string network_class_names_path = declare_parameter(
    network_class_names_desc.name,
    rclcpp::ParameterValue(),
    network_class_names_desc).get<std::string>();

  impl_->threshold_desc_.description = "Minimum detection confidence [0.0, 1.0]";
  impl_->threshold_desc_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  impl_->threshold_desc_.name = "detection.threshold";
  impl_->threshold_ = declare_parameter(
    impl_->threshold_desc_.name,
    rclcpp::ParameterValue(impl_->threshold_),
    impl_->threshold_desc_).get<double>();

  impl_->nms_threshold_desc_.description =
    "Non Maximal Suppression threshold for filtering overlapping boxes [0.0, 1.0]";
  impl_->nms_threshold_desc_.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  impl_->nms_threshold_desc_.name = "detection.nms_threshold";
  impl_->nms_threshold_ = declare_parameter(
    impl_->nms_threshold_desc_.name,
    rclcpp::ParameterValue(impl_->nms_threshold_),
    impl_->nms_threshold_desc_).get<double>();

  set_on_parameters_set_callback(
    std::bind(&DetectorNodePrivate::on_parameters_change, &*impl_, std::placeholders::_1));

  // TODO(sloretz) raise if user tried to initialize node with undeclared parameters

  std::vector<std::string> class_names = parse_class_names(network_class_names_path);
  impl_->network_.reset(
    new DetectorNetwork(network_config_path, network_weights_path, class_names));

  // Ouput topic ~/detections [vision_msgs/msg/Detection2DArray]
  impl_->detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "detections", 1);

  // Input topic ~/images [sensor_msgs/msg/Image]
  impl_->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/fsds/camera/cam1", 12, std::bind(&DetectorNodePrivate::on_image_rx, &*impl_, std::placeholders::_1));
}

DetectorNode::~DetectorNode()
{
}
}  // namespace darknet_ros
}  // namespace openrobotics

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(openrobotics::darknet_ros::DetectorNode)
