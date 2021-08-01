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

#ifndef OPENROBOTICS_DARKNET_ROS__PARSE_HPP_
#define OPENROBOTICS_DARKNET_ROS__PARSE_HPP_

#include <string>
#include <vector>

#include "openrobotics_darknet_ros/visibility.hpp"

namespace openrobotics
{
namespace darknet_ros
{
/// \brief Read file containing class names, one per line
/// \param[in] filename a path to a file containing classes a network can detect
/// \return a container with all of the class names detected
DARKNET_ROS_PUBLIC
std::vector<std::string>
parse_class_names(const std::string & filename);
}  // namespace darknet_ros
}  // namespace openrobotics

#endif  // OPENROBOTICS_DARKNET_ROS__PARSE_HPP_
