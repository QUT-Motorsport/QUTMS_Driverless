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
//
#ifndef DARKNET_DETECTIONS_HPP_
#define DARKNET_DETECTIONS_HPP_

#include <darknet_vendor/darknet_vendor.h>

namespace openrobotics
{
namespace darknet_ros
{
/// \brief RAII wrapper around darknet type `detections`
class DarknetDetections
{
public:
  /// \brief Steal ownership of detections
  DarknetDetections(detection * darknet_detections, size_t num_detections)
  : detections_(darknet_detections), num_detections_(num_detections)
  {
  }

  ~DarknetDetections()
  {
    free_detections(detections_, num_detections_);
  }

  detection * detections_;
  const size_t num_detections_ = 0;
};
}  // namespace darknet_ros
}  // namespace openrobotics
#endif  // DARKNET_DETECTIONS_HPP_
