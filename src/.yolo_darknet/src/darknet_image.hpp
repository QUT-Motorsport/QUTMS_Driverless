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

#ifndef DARKNET_IMAGE_HPP_
#define DARKNET_IMAGE_HPP_

#include <darknet_vendor/darknet_vendor.h>
#include <cv_bridge/cv_bridge.h>

#include <memory>

namespace openrobotics
{
namespace darknet_ros
{
/// \brief RAII wrapper around darknet type `image`
class DarknetImage
{
public:
  /// \brief Steal ownership of image
  explicit DarknetImage(image darknet_image)
  : image_(darknet_image)
  {
  }

  explicit DarknetImage(const sensor_msgs::msg::Image & image_msg)
  {
    // Convert to open cv type with a known format
    std::shared_ptr<void const> dummy_object;
    cv_bridge::CvImageConstPtr opencv_image = cv_bridge::toCvShare(image_msg, dummy_object, "rgb8");
    const cv::Mat & image_matrix = opencv_image->image;

    // Make a darknet image with this data
    const int width = image_matrix.cols;
    const int height = image_matrix.rows;
    const int channels = 3;  // rgb
    image_ = make_image(width, height, channels);

    for (int channel = 0; channel < channels; ++channel) {
      for (int row = 0; row < image_matrix.rows; ++row) {
        for (int column = 0; column < image_matrix.cols; ++column) {
          // Darknet stores each channel separately in R G B order
          // Within a channel pixels are in row-major order
          size_t darknet_idx = channel * height * width + row * width + column;
          image_.data[darknet_idx] = image_matrix.ptr(row, column)[channel] / 255.0f;
        }
      }
    }
  }

  ~DarknetImage()
  {
    free_image(image_);
  }

  image image_;
};
}  // namespace darknet_ros
}  // namespace openrobotics
#endif  // DARKNET_IMAGE_HPP_
