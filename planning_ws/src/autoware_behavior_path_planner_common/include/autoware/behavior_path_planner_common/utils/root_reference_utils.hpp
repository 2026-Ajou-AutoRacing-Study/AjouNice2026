// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/route_handler/route_handler.hpp>
namespace autoware::behavior_path_planner::utils {
using Pose = geometry_msgs::Pose;
using Point = geometry_msgs::Point;
using PoseWithUuidStamped = autoware_planning_msgs::PoseWithUuidStamped;
using PathWithLaneId = autoware_internal_planning_msgs::PathWithLaneId;
using RouteHandler = autoware::route_handler::RouteHandler;
bool isInLaneletWithYawThreshold(
  const Pose & current_pose, const lanelet::ConstLanelet & lanelet, const double yaw_threshold,
  const double radius = 0.0);
bool isEgoOutOfRoute(
  const Pose & self_pose, const lanelet::ConstLanelet & closest_road_lane,
  const std::optional<PoseWithUuidStamped> & modified_goal,
  const std::shared_ptr<RouteHandler> & route_handler);
std::vector<lanelet::Id> get_lanelet_id_from_path(const PathWithLaneId & path);
lanelet::ConstLanelets get_lanelet_sequence_from_path(
  const PathWithLaneId & path, const RouteHandler & route_handler);
lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler);
BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data);
BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data);
void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const bool enable_expanding_freespace_areas,
  const std::shared_ptr<const PlannerData> planner_data, const bool is_driving_forward);
void generateDrivableArea(
  PathWithLaneId & path, const double vehicle_length, const double offset,
  const bool is_driving_forward);
}
