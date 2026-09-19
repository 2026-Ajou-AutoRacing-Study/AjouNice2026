#include <boost/make_shared.hpp>
#include <ros/package.h>
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

#include "autoware/path_generator/node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace autoware::path_generator
{
namespace
{
PathGenerator::RouteManagerData create_route_manager_data()
{
  PathGenerator::RouteManagerData route_manager_data;

  const auto lanelet_map_path =
    ros::package::getPath("autoware_lanelet2_utils") +
    "/sample_map/vm_01_10-12/dense_centerline/lanelet2_map.osm";
  const auto lanelet_map_bin = autoware::test_utils::make_map_bin_msg(lanelet_map_path);
  if (lanelet_map_bin.header.frame_id == "") {
    throw std::runtime_error(
      "Frame ID of the map is empty. The file might not exist or be corrupted:" + lanelet_map_path);
  }
  route_manager_data.lanelet_map_bin_ptr = boost::make_shared<LaneletMapBin>(lanelet_map_bin);

  const auto route_path = autoware::test_utils::get_absolute_path_to_route(
    "autoware_path_generator", "dense_centerline_route.yaml");
  const auto route =
    autoware::test_utils::parse<std::optional<autoware_planning_msgs::LaneletRoute>>(
      route_path);
  if (!route) {
    throw std::runtime_error(
      "Failed to parse YAML file: " + route_path + ". The file might be corrupted.");
  }
  route_manager_data.route_ptr = boost::make_shared<LaneletRoute>(*route);

  return route_manager_data;
}
}  // namespace

TEST(CurrentDenseCenterlineFixture, GeneratePathWithCompleteOriginalParams)
{
  ros::NodeHandle node("/full_path_generator_test");
  PathGenerator path_generator(node);
  const auto route_manager_data = create_route_manager_data();
  path_generator.initialize_route_manager(
    route_manager_data, route_manager_data.route_ptr->start_pose);

  // Original fixture omitted nearest thresholds and used obsolete
  // smooth_goal_connection key. Load the actual complete original schema.
  ::path_generator::ParamListener listener(node);
  const auto params=listener.get_params();
  const auto path = path_generator.generate_path(route_manager_data.route_ptr->start_pose, params);

  ASSERT_TRUE(path.has_value());
  ASSERT_FALSE(path->points.empty());
}
}  // namespace autoware::path_generator
