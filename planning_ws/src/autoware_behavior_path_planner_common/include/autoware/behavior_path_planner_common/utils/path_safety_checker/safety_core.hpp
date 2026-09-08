#pragma once

#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/parameters_core.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_perception_msgs/PredictedPath.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <geometry_msgs/Pose.h>

#include <cmath>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
double calc_rss_distance(double front_object_velocity, double rear_object_velocity, const RSSparams & params);
double calc_minimum_longitudinal_length(double front_object_velocity, double rear_object_velocity, const RSSparams & params);
double calc_obstacle_min_length(const autoware_perception_msgs::Shape & shape);
double calc_obstacle_max_length(const autoware_perception_msgs::Shape & shape);
bool is_target_object_oncoming(const geometry_msgs::Pose & vehicle_pose, const geometry_msgs::Pose & object_pose, double angle_threshold = M_PI_2);
bool is_target_object_front(const geometry_msgs::Pose & ego_pose, const autoware_utils_geometry::Polygon2d & object_polygon, double base_to_front);
autoware_utils_geometry::Polygon2d create_extended_ego_polygon(const geometry_msgs::Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double longitudinal_length, double lateral_margin, bool is_stopped_object);
autoware_utils_geometry::Polygon2d create_extended_ego_polygon_along_path(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const geometry_msgs::Pose & base_link_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double longitudinal_length,
  double lateral_margin, bool is_stopped_object);
autoware_utils_geometry::Polygon2d create_extended_object_polygon(
  const PoseWithVelocityAndPolygonStamped & object_pose, double longitudinal_length,
  double lateral_margin, bool is_stopped_object);
std::vector<autoware_utils_geometry::Polygon2d> create_extended_ego_polygons(
  const std::vector<PoseWithVelocityStamped> & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  double forward_margin, double backward_margin, double lateral_margin);
std::optional<PoseWithVelocityStamped> calc_interpolated_pose_with_velocity(const std::vector<PoseWithVelocityStamped> & path, double relative_time);
std::vector<PoseWithVelocityStamped> filter_predicted_path_by_time_horizon(const std::vector<PoseWithVelocityStamped> & path, double time_horizon);
std::vector<PoseWithVelocityStamped> filter_predicted_path_after_target_pose(const std::vector<PoseWithVelocityStamped> & path, const geometry_msgs::Pose & target_pose);
std::optional<PoseWithVelocityAndPolygonStamped> calc_interpolated_object_pose(
  const std::vector<PoseWithVelocityAndPolygonStamped> & path, double relative_time,
  const autoware_perception_msgs::Shape & shape);
std::pair<bool, bool> check_objects_collision_rough(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::PredictedObjects & objects,
  double min_margin_threshold, double max_margin_threshold,
  const autoware::behavior_path_planner::BehaviorPathPlannerParameters & parameters,
  bool use_offset_ego_point);
bool check_polygons_intersect(const std::vector<autoware_utils_geometry::Polygon2d> & first, const std::vector<autoware_utils_geometry::Polygon2d> & second);
std::optional<geometry_msgs::Pose> calc_object_predicted_pose(const autoware_perception_msgs::PredictedPath & path, double relative_time);
std::vector<PredictedPathWithPolygon> transform_object_predicted_paths(const autoware_perception_msgs::PredictedObject & object, double safety_check_time_horizon, double safety_check_time_resolution);
ExtendedPredictedObject transform_extended_predicted_object(
  const autoware_perception_msgs::PredictedObject & object,
  double safety_check_time_horizon, double safety_check_time_resolution);
std::vector<PredictedPathWithPolygon> get_object_predicted_paths(
  const ExtendedPredictedObject & object, bool use_all_predicted_paths);
ExtendedPredictedObject filter_extended_predicted_object_by_time_horizon(
  const ExtendedPredictedObject & object, double time_horizon, bool use_all_predicted_paths);
bool check_safety_with_integral_predicted_polygons(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, bool use_all_predicted_paths,
  const IntegralPredictedPolygonParams & params);
// Rectangle-policy RSS core.  Path-following extension remains owned by the
// planner-path layer; an unsupported policy fails closed.
bool check_safety_with_rss_rectangle(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const RSSparams & params,
  bool use_all_predicted_paths, double hysteresis_factor, double yaw_difference_threshold);
bool check_safety_with_rss(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const RSSparams & params,
  bool use_all_predicted_paths, double hysteresis_factor, double yaw_difference_threshold);
bool check_safety_with_rss(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, CollisionCheckDebugMap & debug_map,
  const RSSparams & params, bool use_all_predicted_paths, double hysteresis_factor,
  double yaw_difference_threshold);
}
