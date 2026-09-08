#pragma once

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/parameters_core.hpp>
#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <autoware/route_handler/route_handler.hpp>

#include <cmath>
#include <functional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::path_safety_checker::filter
{
// Dependency-light source slice of objects_filtering.cpp used by dynamic footprints.
bool velocity_filter(const geometry_msgs::Twist & object_twist, double velocity_threshold, double max_velocity);
bool position_filter(const autoware_perception_msgs::PredictedObject & object, const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points, const geometry_msgs::Point & current_pose, double forward_distance, double backward_distance);
bool is_within_circle(const geometry_msgs::Point & object_position, const geometry_msgs::Point & reference_point, double search_radius);
bool is_vehicle(const autoware_perception_msgs::ObjectClassification & classification);
}

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
autoware_perception_msgs::PredictedObjects filter_objects_for_path(
  const autoware_perception_msgs::PredictedObjects & objects,
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points,
  const geometry_msgs::Point & current_pose, const ObjectsFilteringCoreParams & params);
autoware_perception_msgs::PredictedObjects filter_objects_by_velocity(const autoware_perception_msgs::PredictedObjects & objects, double velocity_threshold, bool remove_above_threshold = false);
void filter_objects_by_position(autoware_perception_msgs::PredictedObjects & objects, const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points, const geometry_msgs::Point & current_pose, double forward_distance, double backward_distance);
void filter_objects_within_radius(autoware_perception_msgs::PredictedObjects & objects, const geometry_msgs::Point & reference_point, double search_radius);
bool is_target_object_type(const autoware_perception_msgs::PredictedObject & object, const ObjectTypesToCheck & target_object_types);
void filter_objects_by_class(autoware_perception_msgs::PredictedObjects & objects, const ObjectTypesToCheck & target_object_types);
bool is_centroid_within_lanelet(
  const autoware_perception_msgs::PredictedObject & object,
  const lanelet::ConstLanelet & lanelet, double yaw_threshold);
bool is_centroid_within_lanelets(const autoware_perception_msgs::PredictedObject & object, const lanelet::ConstLanelets & target_lanelets);
bool is_polygon_overlap_lanelet(
  const autoware_perception_msgs::PredictedObject & object,
  const lanelet::ConstLanelet & lanelet);
std::pair<std::vector<size_t>, std::vector<size_t>> separate_object_indices_by_lanelets(
  const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const autoware_perception_msgs::PredictedObject &,
                           const lanelet::ConstLanelet &, double)> & condition,
  double yaw_threshold = M_PI);
std::pair<autoware_perception_msgs::PredictedObjects, autoware_perception_msgs::PredictedObjects>
separate_objects_by_lanelets(
  const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const autoware_perception_msgs::PredictedObject &,
                           const lanelet::ConstLanelet &, double)> & condition,
  double yaw_threshold = M_PI);
TargetObjectsOnLane create_target_objects_on_lane(
  const lanelet::ConstLanelets & current_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware_perception_msgs::PredictedObjects & objects,
  const ObjectLaneConfiguration & configuration, bool include_opposite,
  bool invert_opposite, double time_horizon, double time_resolution);
}
