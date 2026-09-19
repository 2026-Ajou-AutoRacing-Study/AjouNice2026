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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/PlanningFactorArray.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <unique_identifier_msgs/UUID.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace autoware::surround_obstacle_checker
{

using autoware::motion_utils::VehicleStopChecker;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::ControlPoint;
using autoware_internal_planning_msgs::PlanningFactor;
using autoware_internal_planning_msgs::PlanningFactorArray;
using autoware_internal_planning_msgs::VelocityLimit;
using autoware_internal_planning_msgs::VelocityLimitClearCommand;
using autoware_perception_msgs::PredictedObject;
using autoware_perception_msgs::PredictedObjects;
using autoware_perception_msgs::Shape;
using geometry_msgs::PolygonStamped;
using unique_identifier_msgs::UUID;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

using geometry_msgs::Point;
using geometry_msgs::Pose;

}  // namespace autoware::surround_obstacle_checker

#endif  // TYPE_ALIAS_HPP_
