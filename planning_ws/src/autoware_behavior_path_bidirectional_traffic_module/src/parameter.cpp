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

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <string>
#include <string_view>

namespace autoware::behavior_path_planner
{

bool BidirectionalTrafficModuleParameters::init_from_node(ros::NodeHandle * node, std::string_view ns)
{
  if (!node) return false;
  const std::string prefix{ns};
  const auto load = [&](const char * leaf, double * value) {
    const auto key = prefix + "/" + leaf;
    if (node->getParam(key, *value)) return true;
    ROS_WARN_STREAM("[bidirectional_traffic] missing or invalid parameter: " << key);
    return false;
  };
  return load("time_to_prepare_pull_over", &time_to_prepare_pull_over) &&
         load("min_distance_from_roadside", &min_distance_from_roadside) &&
         load("keep_left_distance_from_center_line", &keep_left_distance_from_center_line) &&
         load("shift_distance_to_pull_over_from_center_line", &shift_distance_to_pull_over_from_center_line) &&
         load("wait_time_for_oncoming_car", &wait_time_for_oncoming_car) &&
         load("max_lateral_jerk", &max_lateral_jerk) &&
         load("min_lateral_jerk", &min_lateral_jerk);
}

EgoParameters::EgoParameters(
  const double & base_link2front, const double & base_link2rear, const double & vehicle_width)
: base_link2front(base_link2front), base_link2rear(base_link2rear), vehicle_width(vehicle_width)
{
}

[[nodiscard]] autoware_utils_geometry::Polygon2d EgoParameters::ego_polygon(
  const geometry_msgs::Pose & ego_pose) const
{
  return autoware_utils_geometry::to_footprint(
    ego_pose, base_link2front, base_link2rear, vehicle_width);
}

EgoParameters EgoParameters::from_planner_data(const PlannerData & planner_data)
{
  return {
    planner_data.parameters.base_link2front, planner_data.parameters.base_link2rear,
    planner_data.parameters.vehicle_width};
}

};  // namespace autoware::behavior_path_planner
