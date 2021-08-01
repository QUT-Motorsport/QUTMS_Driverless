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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef OPENROBOTICS_DARKNET_ROS__VISIBILITY_HPP_
#define OPENROBOTICS_DARKNET_ROS__VISIBILITY_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DARKNET_ROS_EXPORT __attribute__ ((dllexport))
    #define DARKNET_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define DARKNET_ROS_EXPORT __declspec(dllexport)
    #define DARKNET_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DARKNET_ROS_BUILDING_LIBRARY
    #define DARKNET_ROS_PUBLIC DARKNET_ROS_EXPORT
  #else
    #define DARKNET_ROS_PUBLIC DARKNET_ROS_IMPORT
  #endif
  #define DARKNET_ROS_PUBLIC_TYPE DARKNET_ROS_PUBLIC
  #define DARKNET_ROS_LOCAL
#else
  #define DARKNET_ROS_EXPORT __attribute__ ((visibility("default")))
  #define DARKNET_ROS_IMPORT
  #if __GNUC__ >= 4
    #define DARKNET_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define DARKNET_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DARKNET_ROS_PUBLIC
    #define DARKNET_ROS_LOCAL
  #endif
  #define DARKNET_ROS_PUBLIC_TYPE
#endif

#endif  // OPENROBOTICS_DARKNET_ROS__VISIBILITY_HPP_
