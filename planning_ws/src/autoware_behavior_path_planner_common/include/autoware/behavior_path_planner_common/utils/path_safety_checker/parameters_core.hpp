#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <boost/array.hpp>
#include <unique_identifier_msgs/UUID.h>

#include <utility>
#include <vector>
#include <map>
#include <algorithm>
#include <string>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
// Source-compatible object-class selection subset. The route/safety runtime
// parameter owners remain separate from this dependency-light ABI.
struct ObjectTypesToCheck
{
  bool check_car{true};
  bool check_truck{true};
  bool check_bus{true};
  bool check_trailer{true};
  bool check_unknown{true};
  bool check_bicycle{true};
  bool check_motorcycle{true};
  bool check_pedestrian{true};
};

// Explicit ROS1-core equivalent of the object-filtering parameters owned by
// the ROS2 RouteHandler runtime. This retains the filter contract without
// importing that runtime owner into the dependency-light core.
struct ObjectsFilteringCoreParams
{
  double ignore_object_velocity_threshold{0.0};
  double object_check_forward_distance{0.0};
  double object_check_backward_distance{0.0};
  ObjectTypesToCheck object_types_to_check{};
};

struct ObjectLaneConfiguration
{
  bool check_current_lane{false};
  bool check_right_lane{false};
  bool check_left_lane{false};
  bool check_shoulder_lane{false};
  bool check_other_lane{false};
};

struct RSSparams
{
  std::string extended_polygon_policy{"rectangle"};
  double rear_vehicle_reaction_time{0.0};
  double rear_vehicle_safety_time_margin{0.0};
  double lateral_distance_max_threshold{0.0};
  double longitudinal_distance_min_threshold{0.0};
  double longitudinal_velocity_delta_time{0.0};
  double front_vehicle_deceleration{0.0};
  double rear_vehicle_deceleration{0.0};
};

struct PoseWithVelocity
{
  geometry_msgs::Pose pose{};
  double velocity{0.0};
  PoseWithVelocity() = default;
  PoseWithVelocity(const geometry_msgs::Pose & pose_value, const double velocity_value)
  : pose(pose_value), velocity(velocity_value) {}
};

// Keep ROS1 aggregate-style {time, pose, velocity} construction used by the
// staged safety runtime.  The source public façade exposes PoseWithVelocity
// separately; changing this storage type to inheritance changes C++17 brace
// initialization and corrupts existing predicted paths.
struct PoseWithVelocityStamped
{
  double time{0.0};
  geometry_msgs::Pose pose{};
  double velocity{0.0};
  PoseWithVelocityStamped() = default;
  PoseWithVelocityStamped(const double time_value, const geometry_msgs::Pose & pose_value, const double velocity_value)
  : time(time_value), pose(pose_value), velocity(velocity_value) {}
};

struct PoseWithVelocityAndPolygonStamped : public PoseWithVelocityStamped
{
  autoware_utils_geometry::Polygon2d polygon{};
  PoseWithVelocityAndPolygonStamped() = default;
  PoseWithVelocityAndPolygonStamped(const double time_value, const geometry_msgs::Pose & pose_value, const double velocity_value, autoware_utils_geometry::Polygon2d polygon_value)
  : PoseWithVelocityStamped(time_value, pose_value, velocity_value), polygon(std::move(polygon_value)) {}
};

struct PredictedPathWithPolygon
{
  float confidence{0.0F};
  std::vector<PoseWithVelocityAndPolygonStamped> path{};
};

struct ExtendedPredictedObject
{
  unique_identifier_msgs::UUID uuid{};
  geometry_msgs::Pose initial_pose{};
  geometry_msgs::Twist initial_twist{};
  autoware_perception_msgs::Shape shape{};
  autoware_perception_msgs::ObjectClassification classification{};
  autoware_utils_geometry::Polygon2d initial_polygon{};
  std::vector<PredictedPathWithPolygon> predicted_paths{};
  double dist_from_ego{0.0};
  ExtendedPredictedObject() = default;
  explicit ExtendedPredictedObject(const autoware_perception_msgs::PredictedObject & object)
  : uuid(object.object_id), initial_pose(object.kinematics.initial_pose_with_covariance.pose),
    initial_twist(object.kinematics.initial_twist_with_covariance.twist), shape(object.shape),
    initial_polygon(autoware_utils_geometry::to_polygon2d(initial_pose, shape))
  {
    const auto best = std::max_element(object.classification.begin(), object.classification.end(),
      [](const auto & left, const auto & right) { return left.probability < right.probability; });
    classification.label = best == object.classification.end() ?
      autoware_perception_msgs::ObjectClassification::UNKNOWN : best->label;
  }
};
using ExtendedPredictedObjects = std::vector<ExtendedPredictedObject>;

struct CollisionCheckDebug
{
  std::string unsafe_reason{};
  // Keep the existing host spelling below for its current consumers, while
  // carrying the complete source debug surface required by scene modules.
  geometry_msgs::Twist current_twist{};
  geometry_msgs::Pose expected_ego_pose{};
  geometry_msgs::Pose current_obj_pose{};
  geometry_msgs::Twist object_twist{};
  geometry_msgs::Pose expected_obj_pose{};
  geometry_msgs::Pose expected_object_pose{};
  double rss_longitudinal{0.0};
  double inter_vehicle_distance{0.0};
  double forward_lon_offset{0.0};
  double backward_lon_offset{0.0};
  double lat_offset{0.0};
  bool is_front{false};
  bool is_safe{true};
  autoware_utils_geometry::Polygon2d extended_ego_polygon{};
  autoware_utils_geometry::Polygon2d extended_obj_polygon{};
  autoware_utils_geometry::Polygon2d extended_object_polygon{};
  std::vector<PoseWithVelocityStamped> ego_predicted_path{};
  std::vector<PoseWithVelocityAndPolygonStamped> obj_predicted_path{};
  std::vector<PoseWithVelocityAndPolygonStamped> object_predicted_path{};
  autoware_perception_msgs::Shape obj_shape{};
};
// ROS1 unique_identifier_msgs::UUID stores its value in boost::array.
using CollisionCheckDebugPair = std::pair<boost::array<uint8_t, 16>, CollisionCheckDebug>;
using CollisionCheckDebugMap = std::map<boost::array<uint8_t, 16>, CollisionCheckDebug>;

struct TargetObjectsOnLane
{
  ExtendedPredictedObjects on_current_lane{};
  ExtendedPredictedObjects on_right_lane{};
  ExtendedPredictedObjects on_left_lane{};
  ExtendedPredictedObjects on_shoulder_lane{};
  ExtendedPredictedObjects on_other_lane{};
};

struct IntegralPredictedPolygonParams
{
  double forward_margin{0.0};
  double backward_margin{0.0};
  double lat_margin{0.0};
  double time_horizon{0.0};
};

struct EgoPredictedPathParams
{
  double min_velocity{0.0};
  double min_acceleration{0.0};
  double acceleration{0.0};
  double max_velocity{0.0};
  double time_horizon_for_front_object{0.0};
  double time_horizon_for_rear_object{0.0};
  double time_resolution{0.0};
  double delay_until_departure{0.0};
};

struct ObjectsFilteringParams
{
  double safety_check_time_horizon{0.0};
  double safety_check_time_resolution{0.0};
  double object_check_forward_distance{0.0};
  double object_check_backward_distance{0.0};
  double ignore_object_velocity_threshold{0.0};
  ObjectTypesToCheck object_types_to_check{};
  ObjectLaneConfiguration object_lane_configuration{};
  bool include_opposite_lane{false};
  bool invert_opposite_lane{false};
  bool check_all_predicted_path{false};
  bool use_all_predicted_path{false};
  bool use_predicted_path_outside_lanelet{false};
};

struct SafetyCheckParams
{
  bool enable_safety_check{false};
  std::string method{"RSS"};
  double keep_unsafe_time{0.0};
  double hysteresis_factor_expand_rate{0.0};
  double collision_check_yaw_diff_threshold{3.1416};
  double backward_path_length{0.0};
  double forward_path_length{0.0};
  RSSparams rss_params{};
  IntegralPredictedPolygonParams integral_predicted_polygon_params{};
  bool publish_debug_marker{false};
};
}
