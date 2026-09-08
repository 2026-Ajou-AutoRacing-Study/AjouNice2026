// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_

// ROS1 source-name adapter.  It intentionally delegates only to the tested
// dependency-light core; data-manager owned selection remains outside this
// surface until its provider-bound render is available.
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <boost/geometry.hpp>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
using PathPointWithLaneId = autoware_internal_planning_msgs::PathPointWithLaneId;
using PredictedObject = autoware_perception_msgs::PredictedObject;
using PredictedObjects = autoware_perception_msgs::PredictedObjects;

inline bool isCentroidWithinLanelet(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet,
  const double yaw_threshold)
{
  return is_centroid_within_lanelet(object, lanelet, yaw_threshold);
}

inline bool isPolygonOverlapLanelet(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet,
  const double yaw_threshold)
{
  if (!is_polygon_overlap_lanelet(object, lanelet)) return false;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const lanelet::BasicPoint3d search_point{
    object_pose.position.x, object_pose.position.y, object_pose.position.z};
  const auto closest_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    lanelet, search_point);
  return std::abs(autoware_utils_geometry::calc_yaw_deviation(closest_pose, object_pose)) <
         yaw_threshold;
}

inline bool isPolygonOverlapLanelet(
  const PredictedObject & object, const autoware_utils_geometry::Polygon2d & lanelet_polygon)
{
  return !boost::geometry::disjoint(
    lanelet_polygon, autoware_utils_geometry::to_polygon2d(object));
}

inline bool isPolygonOverlapLanelet(
  const PredictedObject & object, const lanelet::BasicPolygon2d & lanelet_polygon)
{
  // ROS1 Lanelet2's BasicPolygon2d is not registered as a Boost polygon.
  // Convert each 2D point to the staged Boost surface before preserving the
  // source overlap operation.
  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : lanelet_polygon) {
    boost::geometry::append(
      converted.outer(), autoware_utils_geometry::Point2d{point.x(), point.y()});
  }
  boost::geometry::correct(converted);
  return !boost::geometry::disjoint(converted, autoware_utils_geometry::to_polygon2d(object));
}

template <class ObjectsPointer>
inline PredictedObjects filterObjects(
  const ObjectsPointer & objects,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & current_lanes, const geometry_msgs::Point & current_pose,
  const std::shared_ptr<ObjectsFilteringParams> & params)
{
  if (!objects || objects->objects.empty() || !route_handler || !params) return {};
  const auto path = route_handler->getCenterLinePath(
    current_lanes, params->object_check_backward_distance, params->object_check_forward_distance);
  ObjectsFilteringCoreParams core_params;
  core_params.ignore_object_velocity_threshold = params->ignore_object_velocity_threshold;
  core_params.object_check_forward_distance = params->object_check_forward_distance;
  core_params.object_check_backward_distance = params->object_check_backward_distance;
  core_params.object_types_to_check = params->object_types_to_check;
  return filter_objects_for_path(*objects, path.points, current_pose, core_params);
}

inline PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, const double velocity_threshold,
  const bool remove_above_threshold = false)
{
  return filter_objects_by_velocity(objects, velocity_threshold, remove_above_threshold);
}

inline PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, const double velocity_threshold, const double max_velocity)
{
  PredictedObjects filtered = objects;
  filtered.objects.erase(
    std::remove_if(filtered.objects.begin(), filtered.objects.end(), [&](const auto & object) {
      return !filter::velocity_filter(
        object.kinematics.initial_twist_with_covariance.twist, velocity_threshold, max_velocity);
    }),
    filtered.objects.end());
  return filtered;
}

inline void filterObjectsByPosition(
  PredictedObjects & objects, const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::Point & current_pose, const double forward_distance,
  const double backward_distance)
{
  filter_objects_by_position(
    objects, path_points, current_pose, forward_distance, backward_distance);
}

inline void filterObjectsWithinRadius(
  PredictedObjects & objects, const geometry_msgs::Point & reference_point,
  const double search_radius)
{
  filter_objects_within_radius(objects, reference_point, search_radius);
}

inline bool isTargetObjectType(
  const PredictedObject & object, const ObjectTypesToCheck & target_object_types)
{
  return is_target_object_type(object, target_object_types);
}

inline void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types)
{
  filter_objects_by_class(objects, target_object_types);
}

using LaneletCondition = std::function<bool(
  const PredictedObject &, const lanelet::ConstLanelet &, double)>;

inline std::pair<std::vector<size_t>, std::vector<size_t>> separateObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets,
  const LaneletCondition & condition, const double yaw_threshold = M_PI)
{
  return separate_object_indices_by_lanelets(objects, target_lanelets, condition, yaw_threshold);
}

inline std::pair<PredictedObjects, PredictedObjects> separateObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets,
  const LaneletCondition & condition, const double yaw_threshold = M_PI)
{
  return separate_objects_by_lanelets(objects, target_lanelets, condition, yaw_threshold);
}

template <typename PredictedPathsType>
PredictedPathsType get_highest_confidence_paths(PredictedPathsType predicted_paths)
{
  if (predicted_paths.empty()) return predicted_paths;
  std::sort(predicted_paths.begin(), predicted_paths.end(), [](const auto & first, const auto & second) {
    return first.confidence > second.confidence;
  });
  const auto highest = predicted_paths.front().confidence;
  const auto last = std::find_if(
    std::next(predicted_paths.begin()), predicted_paths.end(),
    [highest](const auto & path) { return path.confidence != highest; });
  return {predicted_paths.begin(), last};
}

template <typename PredictedPathsType>
PredictedPathsType get_object_predicted_paths(
  const PredictedPathsType & predicted_paths, const bool use_all_predicted_path)
{
  return use_all_predicted_path ? predicted_paths : get_highest_confidence_paths(predicted_paths);
}

inline std::vector<PredictedPathWithPolygon> getPredictedPathFromObj(
  const ExtendedPredictedObject & object, const bool use_all_predicted_path)
{
  return get_object_predicted_paths(object, use_all_predicted_path);
}

// Source-equivalent ego prediction surface used by scene modules.  This is a
// real path/Frenet integration, not a fixed pose or an empty fallback.
std::vector<PoseWithVelocityStamped> createPredictedPath(
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::Pose & vehicle_pose, double current_velocity,
  size_t ego_seg_idx, bool is_object_front, bool limit_to_max_velocity);

inline bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  return is_centroid_within_lanelets(object, target_lanelets);
}

inline ExtendedPredictedObject transform(
  const PredictedObject & object, const double safety_check_time_horizon,
  const double safety_check_time_resolution)
{
  return transform_extended_predicted_object(
    object, safety_check_time_horizon, safety_check_time_resolution);
}

inline TargetObjectsOnLane createTargetObjectsOnLane(
  const lanelet::ConstLanelets & current_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const PredictedObjects & objects, const std::shared_ptr<ObjectsFilteringParams> & params)
{
  if (!params) return {};
  return create_target_objects_on_lane(
    current_lanes, route_handler, objects, params->object_lane_configuration,
    params->include_opposite_lane, params->invert_opposite_lane,
    params->safety_check_time_horizon, params->safety_check_time_resolution);
}

template <typename Func>
void filterObjects(PredictedObjects & selected, PredictedObjects & removed, Func filter)
{
  const auto partitioned = std::partition(
    selected.objects.begin(), selected.objects.end(), filter);
  removed.objects.insert(removed.objects.end(), partitioned, selected.objects.end());
  selected.objects.erase(partitioned, selected.objects.end());
}

template <typename Func>
void filterObjects(PredictedObjects & objects, Func filter)
{
  PredictedObjects removed;
  filterObjects(objects, removed, filter);
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker

#endif
