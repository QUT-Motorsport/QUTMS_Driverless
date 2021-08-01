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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "openrobotics_darknet_ros/detector_network.hpp"

TEST(network, config_does_not_exist)
{
  const std::string config = "does_not_exist.cfg";
  const std::string weights = "does_not_exist.weights";
  std::vector<std::string> classes{"foo", "bar"};

  try {
    openrobotics::darknet_ros::DetectorNetwork network(config, weights, classes);
    ASSERT_TRUE(false);
  } catch (const std::invalid_argument &) {
  }
}
