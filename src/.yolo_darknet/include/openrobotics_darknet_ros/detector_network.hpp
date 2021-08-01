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

#ifndef OPENROBOTICS_DARKNET_ROS__DETECTOR_NETWORK_HPP_
#define OPENROBOTICS_DARKNET_ROS__DETECTOR_NETWORK_HPP_

#include <memory>
#include <string>
#include <vector>

#include "openrobotics_darknet_ros/visibility.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace openrobotics
{
namespace darknet_ros
{
// Forward declaration
class DetectorNetworkPrivate;

class DetectorNetwork
{
public:
  /// \brief load a network from disk
  /// \param[in] config_file Path to a file describing the network
  /// \param[in] weights_file Path to a file containing the network's weights
  /// \param[in] classes Ordered list of class names the network can predict
  DARKNET_ROS_PUBLIC
  DetectorNetwork(
    const std::string & config_file,
    const std::string & weights_file,
    const std::vector<std::string> & classes);

  DARKNET_ROS_PUBLIC
  ~DetectorNetwork();

  /// \brief Detect objects in image
  /// \param[in] image An image to analyze
  /// \param[in] threshold How confident the network must be to detect something [0.0, 1.0]
  /// \param[in] nms_threshold Non-Maximal Suppression threhsold [0.0, 1.0].
  ///   When the intersection over union (iou) of two bounding boxes is greater than nms_threshold,
  ///   the box with the lower objectness score is discarded.
  /// \param[out] output_detections Things detected in the image (does not set source_img)
  /// \return number of objects detected
  DARKNET_ROS_PUBLIC
  size_t
  detect(
    const sensor_msgs::msg::Image & image,
    double threshold,
    double nms_threshold,
    vision_msgs::msg::Detection2DArray * output_detections);

private:
  std::unique_ptr<DetectorNetworkPrivate> impl_;
};
}  // namespace darknet_ros
}  // namespace openrobotics

#endif  // OPENROBOTICS_DARKNET_ROS__DETECTOR_NETWORK_HPP_
