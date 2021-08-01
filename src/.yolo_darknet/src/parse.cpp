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

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "openrobotics_darknet_ros/parse.hpp"

namespace openrobotics
{
namespace darknet_ros
{
std::vector<std::string>
parse_class_names(const std::string & filename)
{
  std::ifstream fin(filename);
  std::vector<std::string> class_names;
  std::string line;
  while (fin) {
    std::getline(fin, line);
    if (!fin) {
      if (!fin.eof()) {
        std::stringstream str;
        str << "Failed to read [" << filename << "] line " << class_names.size();
        throw std::runtime_error(str.str());
      }
      break;
    }
    if (line.empty()) {
      // Ignore blank lines
      continue;
    }
    class_names.emplace_back(line);
  }
  return class_names;
}
}  // namespace darknet_ros
}  // namespace openrobotics
