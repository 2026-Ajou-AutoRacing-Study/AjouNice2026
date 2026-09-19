// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_vehicle_msgs/TurnIndicatorsCommand.h>
#include <tier4_planning_msgs/AvoidanceDebugMsg.h>
#include <tier4_planning_msgs/AvoidanceDebugMsgArray.h>
#include <tier4_rtc_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>

namespace autoware::behavior_path_planner
{
// auto msgs
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_perception_msgs::ObjectClassification;
using autoware_perception_msgs::PredictedObject;
using autoware_perception_msgs::PredictedObjects;
using autoware_perception_msgs::PredictedPath;
using autoware_perception_msgs::Shape;
using autoware_vehicle_msgs::TurnIndicatorsCommand;
using autoware::route_handler::RouteHandler;

// ROS 2 general msgs
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseWithCovariance;
using geometry_msgs::TransformStamped;
using geometry_msgs::Vector3;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// tier4 msgs
using tier4_planning_msgs::AvoidanceDebugMsg;
using tier4_planning_msgs::AvoidanceDebugMsgArray;
using tier4_rtc_msgs::State;
using unique_identifier_msgs::UUID;
using autoware::rtc_interface::RTCInterface;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::planning_factor_interface::PlanningFactor;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware_utils::generate_uuid;

// tier4 utils functions
using autoware_utils::append_marker_array;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_lateral_deviation;
using autoware_utils::calc_offset_pose;
using autoware_utils::calc_yaw_deviation;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using autoware_utils::create_point;
using autoware_utils::create_quaternion_from_rpy;
using autoware_utils::get_point;
using autoware_utils::get_pose;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using autoware_utils::pose2transform;
using autoware_utils::to_hex_string;

#ifdef AW149_BPP_HOST_ABI
// The executable BPP Common profile does not re-export this source helper
// through behavior_path_planner::utils. Keep the source-compatible name
// profile-local: the standalone Common provider already owns it.
namespace utils
{
using autoware::object_recognition_utils::getHighestProbLabel;
}  // namespace utils
#endif

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
