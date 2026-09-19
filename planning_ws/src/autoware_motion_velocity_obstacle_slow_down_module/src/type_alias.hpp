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

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include "autoware_internal_debug_msgs/Float32MultiArrayStamped.h"
#include "autoware_internal_debug_msgs/Float32Stamped.h"
#include "autoware_internal_debug_msgs/Float64Stamped.h"
#include "autoware_internal_planning_msgs/VelocityLimit.h"
#include "autoware_internal_planning_msgs/VelocityLimitClearCommand.h"
#include "autoware_perception_msgs/PredictedObject.h"
#include "autoware_perception_msgs/PredictedObjects.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "unique_identifier_msgs/UUID.h"
#include "visualization_msgs/MarkerArray.h"
#include <autoware_internal_planning_msgs/PlanningFactor.h>
#include <autoware_internal_planning_msgs/SafetyFactorArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

namespace autoware::motion_velocity_planner
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_debug_msgs::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::Float32Stamped;
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_internal_planning_msgs::VelocityLimit;
using autoware_internal_planning_msgs::VelocityLimitClearCommand;
using autoware_perception_msgs::ObjectClassification;
using autoware_perception_msgs::PredictedObject;
using autoware_perception_msgs::PredictedObjects;
using autoware_perception_msgs::PredictedPath;
using autoware_perception_msgs::Shape;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;
using unique_identifier_msgs::UUID;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
namespace bg = boost::geometry;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware_internal_planning_msgs::PlanningFactor;
using autoware_internal_planning_msgs::SafetyFactorArray;
}  // namespace autoware::motion_velocity_planner

#endif  // TYPE_ALIAS_HPP_
