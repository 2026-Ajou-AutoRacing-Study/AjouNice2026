// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__PLANNING_HPP_

#include <autoware/component_interface_specs/utils.hpp>

#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/RouteState.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/ClearRoute.h>
#include <autoware_planning_msgs/SetLaneletRoute.h>
#include <autoware_planning_msgs/SetWaypointRoute.h>

namespace autoware::component_interface_specs::planning
{

struct SetLaneletRoute
{
  using Service = autoware_planning_msgs::SetLaneletRoute;
  static constexpr char name[] = "/planning/set_lanelet_route";
};

struct SetWaypointRoute
{
  using Service = autoware_planning_msgs::SetWaypointRoute;
  static constexpr char name[] = "/planning/set_waypoint_route";
};

struct ClearRoute
{
  using Service = autoware_planning_msgs::ClearRoute;
  static constexpr char name[] = "/planning/clear_route";
};

struct RouteState
{
  using Message = autoware_planning_msgs::RouteState;
  static constexpr char name[] = "/planning/route_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

struct LaneletRoute
{
  using Message = autoware_planning_msgs::LaneletRoute;
  static constexpr char name[] = "/planning/route";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

struct Trajectory
{
  using Message = autoware_planning_msgs::Trajectory;
  static constexpr char name[] = "/planning/trajectory";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

}  // namespace autoware::component_interface_specs::planning

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
