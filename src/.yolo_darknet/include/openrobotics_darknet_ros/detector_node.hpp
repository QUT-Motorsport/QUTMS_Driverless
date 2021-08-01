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

#ifndef OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_
#define OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "openrobotics_darknet_ros/visibility_node.hpp"


namespace openrobotics
{
namespace darknet_ros
{
// Forward declaration
class DetectorNodePrivate;

class DetectorNode : public rclcpp::Node
{
public:
  /// \brief Create a node that uses ROS parameters to get the network
  DARKNET_ROS_NODE_PUBLIC
  explicit DetectorNode(rclcpp::NodeOptions options);

  DARKNET_ROS_NODE_PUBLIC
  virtual ~DetectorNode();

private:
  std::unique_ptr<DetectorNodePrivate> impl_;
};
}  // namespace darknet_ros
}  // namespace openrobotics

#endif  // OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_
