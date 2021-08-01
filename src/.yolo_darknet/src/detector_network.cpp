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

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "darknet_detections.hpp"
#include "darknet_image.hpp"
#include "openrobotics_darknet_ros/detector_network.hpp"

namespace openrobotics
{
namespace darknet_ros
{
class DetectorNetworkPrivate
{
public:
  DetectorNetworkPrivate() {}

  ~DetectorNetworkPrivate()
  {
    if (network_) {
      free_network(network_);
      network_ = nullptr;
    }
  }

  // darknet network
  network * network_ = nullptr;
  // Classes the network can detect
  std::vector<std::string> class_names_;
};

DetectorNetwork::DetectorNetwork(
  const std::string & config_file,
  const std::string & weights_file,
  const std::vector<std::string> & classes)
: impl_(new DetectorNetworkPrivate())
{
  if (!std::ifstream(config_file)) {
    std::stringstream str;
    str << "Could not open " << config_file;
    throw std::invalid_argument(str.str());
  } else if (!std::ifstream(weights_file)) {
    std::stringstream str;
    str << "Could not open " << weights_file;
    throw std::invalid_argument(str.str());
  }

  impl_->class_names_ = classes;

  // Make copies because of darknet's lack of const
  std::unique_ptr<char> config_mutable(new char[config_file.size() + 1]);
  std::unique_ptr<char> weights_mutable(new char[weights_file.size() + 1]);
  snprintf(&*config_mutable, config_file.size() + 1, "%s", config_file.c_str());
  snprintf(&*weights_mutable, weights_file.size() + 1, "%s", weights_file.c_str());

  const int clear = 0;
  impl_->network_ = load_network(&*config_mutable, &*weights_mutable, clear);
  if (nullptr == impl_->network_) {
    std::stringstream str;
    str << "Failed to load network from " << config_file << " and " << weights_file;
    throw std::invalid_argument(str.str());
  }

  // TODO(sloretz) what is this and why do examples set it?
  const int batch = 1;
  set_batch_network(impl_->network_, batch);

  const int num_classes_int = impl_->network_->layers[impl_->network_->n - 1].classes;
  if (num_classes_int <= 0) {
    throw std::invalid_argument("Invalid network, it expects no classes");
  }
  size_t num_classes = static_cast<size_t>(num_classes_int);
  if (num_classes != classes.size()) {
    std::stringstream str;
    str << "DetectorNetwork expects " << num_classes << " class names but got " << classes.size();
    throw std::invalid_argument(str.str());
  }
}

DetectorNetwork::~DetectorNetwork()
{
}

size_t
DetectorNetwork::detect(
  const sensor_msgs::msg::Image & image_msg,
  double threshold,
  double nms_threshold,
  vision_msgs::msg::Detection2DArray * output_detections)
{
  DarknetImage orig_image(image_msg);

  // resize image to network size, filling rest with gray
  DarknetImage resized_image(
    letterbox_image(orig_image.image_, impl_->network_->w, impl_->network_->h));

  // Ask network to make predictions
  network_predict(impl_->network_, resized_image.image_.data);

  // Get predictions from network
  int num_detections = 0;
  // TODO(sloretz) what do hier, map, and relative do?
  const float hier = 0;
  int * map = nullptr;
  const int relative = 0;
  detection * darknet_detections = get_network_boxes(
    impl_->network_, image_msg.width, image_msg.height, threshold,
    hier, map, relative,
    &num_detections);

  if (num_detections <= 0) {
    return 0;
  }

  DarknetDetections raii_detections(darknet_detections, static_cast<size_t>(num_detections));

  // Non-maximal suppression: filters overlapping bounding boxes
  if (nms_threshold > 0.0f) {
    const int num_classes = impl_->network_->layers[impl_->network_->n - 1].classes;
    do_nms_sort(darknet_detections, num_detections, num_classes, nms_threshold);
  }

  // Populate output message
  output_detections->header = image_msg.header;
  output_detections->detections.reserve(num_detections);
  for (int i = 0; i < num_detections; ++i) {
    auto & detection = darknet_detections[i];
    output_detections->detections.emplace_back();
    auto & detection_ros = output_detections->detections.back();

    // Copy probabilities of each class
    for (int cls = 0; cls < detection.classes; ++cls) {
      if (detection.prob[cls] > 0.0f) {
        detection_ros.results.emplace_back();
        auto & hypothesis = detection_ros.results.back();
        hypothesis.id = impl_->class_names_.at(cls);
        hypothesis.score = detection.prob[cls];
      }
    }

    if (detection_ros.results.empty()) {
      // nms suppressed this detection
      output_detections->detections.pop_back();
      continue;
    }

    // Copy bounding box, darknet uses center of bounding box too
    detection_ros.bbox.center.x = detection.bbox.x;
    detection_ros.bbox.center.y = detection.bbox.y;
    detection_ros.bbox.size_x = detection.bbox.w;
    detection_ros.bbox.size_y = detection.bbox.h;
  }

  // Not using num_detections because nms may have suppressed some
  return output_detections->detections.size();
}
}  // namespace darknet_ros
}  // namespace openrobotics
