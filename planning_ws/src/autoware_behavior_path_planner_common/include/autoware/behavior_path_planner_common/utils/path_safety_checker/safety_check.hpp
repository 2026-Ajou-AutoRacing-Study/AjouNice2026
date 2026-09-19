// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__SAFETY_CHECK_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__SAFETY_CHECK_HPP_

// ROS1 source-name adapter for the verified safety-core subset.  Debug-map and
// provider-publication overloads deliberately remain unrendered: accepting
// those arguments without their side effects would hide a safety regression.
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <autoware_internal_planning_msgs/SafetyFactorArray.h>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
using Polygon2d = autoware_utils_geometry::Polygon2d;
using VehicleInfo = autoware::vehicle_info_utils::VehicleInfo;
using PathWithLaneId = autoware_internal_planning_msgs::PathWithLaneId;

inline bool isTargetObjectOncoming(
  const geometry_msgs::Pose & vehicle_pose, const geometry_msgs::Pose & object_pose,
  const double angle_threshold = M_PI_2)
{
  return is_target_object_oncoming(vehicle_pose, object_pose, angle_threshold);
}

inline bool isTargetObjectFront(
  const geometry_msgs::Pose & ego_pose, const Polygon2d & object_polygon,
  const double base_to_front)
{
  return is_target_object_front(ego_pose, object_polygon, base_to_front);
}

inline Polygon2d createExtendedPolygon(
  const geometry_msgs::Pose & base_link_pose, const VehicleInfo & vehicle_info,
  const double lon_length, const double lat_margin, const bool is_stopped_obj)
{
  return create_extended_ego_polygon(
    base_link_pose, vehicle_info, lon_length, lat_margin, is_stopped_obj);
}

inline Polygon2d createExtendedPolygon(
  const PoseWithVelocityAndPolygonStamped & object_pose_with_polygon,
  const double lon_length, const double lat_margin, const bool is_stopped_obj)
{
  return create_extended_object_polygon(
    object_pose_with_polygon, lon_length, lat_margin, is_stopped_obj);
}

inline double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params)
{
  return calc_rss_distance(front_object_velocity, rear_object_velocity, rss_params);
}

inline std::vector<PoseWithVelocityStamped> filterPredictedPathByTimeHorizon(
  const std::vector<PoseWithVelocityStamped> & path, const double time_horizon)
{
  return filter_predicted_path_by_time_horizon(path, time_horizon);
}

inline ExtendedPredictedObject filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObject & object, const double time_horizon,
  const bool check_all_predicted_path)
{
  return filter_extended_predicted_object_by_time_horizon(
    object, time_horizon, check_all_predicted_path);
}

inline ExtendedPredictedObjects filterObjectPredictedPathByTimeHorizon(
  const ExtendedPredictedObjects & objects, const double time_horizon,
  const bool check_all_predicted_path)
{
  ExtendedPredictedObjects filtered;
  filtered.reserve(objects.size());
  for (const auto & object : objects) {
    filtered.push_back(filterObjectPredictedPathByTimeHorizon(
      object, time_horizon, check_all_predicted_path));
  }
  return filtered;
}

inline std::vector<PoseWithVelocityStamped> filterPredictedPathAfterTargetPose(
  const std::vector<PoseWithVelocityStamped> & path, const geometry_msgs::Pose & target_pose)
{
  return filter_predicted_path_after_target_pose(path, target_pose);
}

inline std::pair<bool, bool> checkObjectsCollisionRough(
  const PathWithLaneId & path, const autoware_perception_msgs::PredictedObjects & objects,
  const double min_margin_threshold, const double max_margin_threshold,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_point)
{
  return check_objects_collision_rough(
    path, objects, min_margin_threshold, max_margin_threshold, parameters,
    use_offset_ego_point);
}

inline bool checkPolygonsIntersects(
  const std::vector<Polygon2d> & first, const std::vector<Polygon2d> & second)
{
  return check_polygons_intersect(first, second);
}

// Source public surface for modules that perform a per-object RSS decision
// and then publish its debug state through the shared ROS1 UUID map.
bool checkCollision(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const RSSparams & rss_parameters,
  double hysteresis_factor, double yaw_difference_th, CollisionCheckDebug & debug);

CollisionCheckDebugPair createObjectDebug(const ExtendedPredictedObject & object);
void updateCollisionCheckDebugMap(
  CollisionCheckDebugMap & debug_map, CollisionCheckDebugPair & object_debug, bool is_safe);

autoware_perception_msgs::PredictedPath convertToPredictedPath(
  const std::vector<PoseWithVelocityStamped> & path, double time_resolution);
bool checkSafetyWithRSS(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const std::vector<ExtendedPredictedObject> & objects, CollisionCheckDebugMap & debug_map,
  const BehaviorPathPlannerParameters & parameters, const RSSparams & rss_params,
  bool check_all_predicted_path, double hysteresis_factor, double yaw_difference_th);

double shortest_distance_from_ego_footprint_to_objects_on_path(
  const PathWithLaneId & path, const autoware_perception_msgs::PredictedObjects & objects,
  const BehaviorPathPlannerParameters & parameters, bool use_offset_ego_pose);
bool checkSafetyWithIntegralPredictedPolygon(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, bool check_all_predicted_path,
  const IntegralPredictedPolygonParams & params, CollisionCheckDebugMap & debug_map);

inline autoware_internal_planning_msgs::SafetyFactorArray to_safety_factor_array(
  const CollisionCheckDebugMap & debug_map)
{
  autoware_internal_planning_msgs::SafetyFactorArray safety_factors;
  safety_factors.is_safe = std::all_of(
    debug_map.begin(), debug_map.end(), [](const auto & result) { return result.second.is_safe; });
  for (const auto & [uuid, data] : debug_map) {
    autoware_internal_planning_msgs::SafetyFactor factor;
    factor.type = autoware_internal_planning_msgs::SafetyFactor::OBJECT;
    factor.is_safe = data.is_safe;
    factor.object_id.uuid = uuid;
    factor.points.push_back(data.expected_object_pose.position);
    safety_factors.factors.push_back(factor);
  }
  return safety_factors;
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker

#endif
