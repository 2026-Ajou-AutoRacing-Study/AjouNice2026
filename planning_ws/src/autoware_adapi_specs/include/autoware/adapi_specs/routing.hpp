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

#ifndef AUTOWARE__ADAPI_SPECS__ROUTING_HPP_
#define AUTOWARE__ADAPI_SPECS__ROUTING_HPP_

#include <autoware/adapi_specs/qos.hpp>

#include <autoware_adapi_v1_msgs/Route.h>
#include <autoware_adapi_v1_msgs/RouteState.h>
#include <autoware_adapi_v1_msgs/ClearRoute.h>
#include <autoware_adapi_v1_msgs/SetRoute.h>
#include <autoware_adapi_v1_msgs/SetRoutePoints.h>

namespace autoware::adapi_specs::routing
{

struct SetRoutePoints
{
  using Service = autoware_adapi_v1_msgs::SetRoutePoints;
  static constexpr char name[] = "/api/routing/set_route_points";
};

struct SetRoute
{
  using Service = autoware_adapi_v1_msgs::SetRoute;
  static constexpr char name[] = "/api/routing/set_route";
};

struct ChangeRoutePoints
{
  using Service = autoware_adapi_v1_msgs::SetRoutePoints;
  static constexpr char name[] = "/api/routing/change_route_points";
};

struct ChangeRoute
{
  using Service = autoware_adapi_v1_msgs::SetRoute;
  static constexpr char name[] = "/api/routing/change_route";
};

struct ClearRoute
{
  using Service = autoware_adapi_v1_msgs::ClearRoute;
  static constexpr char name[] = "/api/routing/clear_route";
};

struct RouteState
{
  using Message = autoware_adapi_v1_msgs::RouteState;
  static constexpr char name[] = "/api/routing/state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct Route
{
  using Message = autoware_adapi_v1_msgs::Route;
  static constexpr char name[] = "/api/routing/route";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::adapi_specs::routing

#endif  // AUTOWARE__ADAPI_SPECS__ROUTING_HPP_
