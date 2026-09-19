// Copyright 2025 TIER IV, Inc.
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

#ifndef TEST_CASE_HPP_
#define TEST_CASE_HPP_

#include "ros1_test_paths.hpp"

#include <geometry_msgs/Pose.h>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>

namespace autoware::test_utils
{

inline geometry_msgs::Pose parse_pose(const YAML::Node & node)
{
  geometry_msgs::Pose pose;
  pose.position.x = node["position"]["x"].as<double>();
  pose.position.y = node["position"]["y"].as<double>();
  pose.position.z = node["position"]["z"].as<double>();
  pose.orientation.x = node["orientation"]["x"].as<double>();
  pose.orientation.y = node["orientation"]["y"].as<double>();
  pose.orientation.z = node["orientation"]["z"].as<double>();
  pose.orientation.w = node["orientation"]["w"].as<double>();
  return pose;
}

struct TestCaseFormat1
{
  std::string map_abs_path;
  std::unordered_map<std::string, geometry_msgs::Pose> manual_poses;
};

inline TestCaseFormat1 load_test_case_format1(const YAML::Node & node)
{
  const auto map_rel_path = node["map_rel_path"].as<std::string>();
  const auto map_abs_path =
    autoware::test_utils::lanelet2_utils_source_root() / "sample_map" / map_rel_path;
  int cnt = 0;
  std::unordered_map<std::string, geometry_msgs::Pose> manual_poses;
  for (const auto & pose : node["manual_poses"]) {
    const auto name = "P" + std::to_string(cnt);
    manual_poses[name] = parse_pose(pose);
    cnt++;
  }
  return {map_abs_path.string(), manual_poses};
}

inline TestCaseFormat1 load_test_case(const std::string & test_case_file_path)
{
  try {
    YAML::Node node = YAML::LoadFile(test_case_file_path);
    const auto version = node["version"].as<int>();
    if (version == 1) {
      return load_test_case_format1(node);
    }
    throw std::invalid_argument("unsupported version: " + std::to_string(version));
  } catch (std::exception & e) {
    std::cerr << "failed to load:  " << e.what() << std::endl;
    throw e;
  }
}

}  // namespace autoware::test_utils
#endif  // TEST_CASE_HPP_
