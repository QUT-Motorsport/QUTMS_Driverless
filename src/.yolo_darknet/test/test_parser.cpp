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

#include "openrobotics_darknet_ros/parse.hpp"

using openrobotics::darknet_ros::parse_class_names;

TEST(parse, names_file_does_not_exist)
{
  std::vector<std::string> class_names;

  class_names = parse_class_names("openrobotics_darknet_ros_file_does_not_exist");

  ASSERT_EQ(0u, class_names.size());
}

TEST(parse, one_name)
{
  std::vector<std::string> class_names;

  class_names = parse_class_names("data/1_class_name.txt");

  ASSERT_EQ(1u, class_names.size());
  EXPECT_EQ("tomato", class_names[0]);
}

TEST(parse, 3_class_names_with_whitespace)
{
  std::vector<std::string> class_names;

  class_names = parse_class_names("data/3_class_names_with_whitespace.txt");

  ASSERT_EQ(3u, class_names.size());
  EXPECT_EQ("foo", class_names[0]);
  EXPECT_EQ("bar", class_names[1]);
  EXPECT_EQ("baz", class_names[2]);
}

TEST(parse, five_names)
{
  std::vector<std::string> class_names;

  class_names = parse_class_names("data/5_class_names.txt");

  ASSERT_EQ(5u, class_names.size());
  EXPECT_EQ("car", class_names[0]);
  EXPECT_EQ("boat", class_names[1]);
  EXPECT_EQ("bus", class_names[2]);
  EXPECT_EQ("airplane", class_names[3]);
  EXPECT_EQ("space ship", class_names[4]);
}
