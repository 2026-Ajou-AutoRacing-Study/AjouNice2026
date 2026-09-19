#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::behavior_path_planner::utils::path_safety_checker::filter
{
bool velocity_filter(const geometry_msgs::Twist & object_twist, const double velocity_threshold, const double max_velocity)
{
  const auto v_norm = std::hypot(object_twist.linear.x, object_twist.linear.y);
  return velocity_threshold < v_norm && v_norm < max_velocity;
}
bool position_filter(const autoware_perception_msgs::PredictedObject & object, const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points, const geometry_msgs::Point & current_pose, const double forward_distance, const double backward_distance)
{
  if (path_points.empty()) return false;
  const auto distance = autoware::motion_utils::calcSignedArcLength(path_points, current_pose, object.kinematics.initial_pose_with_covariance.pose.position);
  return backward_distance < distance && distance < forward_distance;
}
bool is_within_circle(const geometry_msgs::Point & object_position, const geometry_msgs::Point & reference_point, const double search_radius)
{
  return std::hypot(reference_point.x - object_position.x, reference_point.y - object_position.y) < search_radius;
}
bool is_vehicle(const autoware_perception_msgs::ObjectClassification & classification)
{
  switch (classification.label) {
    case autoware_perception_msgs::ObjectClassification::CAR:
    case autoware_perception_msgs::ObjectClassification::TRUCK:
    case autoware_perception_msgs::ObjectClassification::BUS:
    case autoware_perception_msgs::ObjectClassification::TRAILER:
    case autoware_perception_msgs::ObjectClassification::MOTORCYCLE:
      return true;
    default:
      return false;
  }
}
}

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
autoware_perception_msgs::PredictedObjects filter_objects_for_path(
  const autoware_perception_msgs::PredictedObjects & objects,
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points,
  const geometry_msgs::Point & current_pose, const ObjectsFilteringCoreParams & params)
{
  if (objects.objects.empty() || path_points.empty()) return {};

  // Preserve the upstream filterObjects ordering: motion, class, then route
  // position. Later stages intentionally see only objects admitted upstream.
  auto filtered = filter_objects_by_velocity(
    objects, params.ignore_object_velocity_threshold, true);
  filter_objects_by_class(filtered, params.object_types_to_check);
  filter_objects_by_position(
    filtered, path_points, current_pose, params.object_check_forward_distance,
    params.object_check_backward_distance);
  return filtered;
}

autoware_perception_msgs::PredictedObjects filter_objects_by_velocity(const autoware_perception_msgs::PredictedObjects & objects, const double velocity_threshold, const bool remove_above_threshold)
{
  const double minimum = remove_above_threshold ? velocity_threshold : -velocity_threshold;
  const double maximum = remove_above_threshold ? std::numeric_limits<double>::max() : velocity_threshold;
  auto filtered = objects;
  filtered.objects.erase(std::remove_if(filtered.objects.begin(), filtered.objects.end(), [&](const auto & object) {
    return !filter::velocity_filter(object.kinematics.initial_twist_with_covariance.twist, minimum, maximum);
  }), filtered.objects.end());
  return filtered;
}
void filter_objects_by_position(autoware_perception_msgs::PredictedObjects & objects, const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & path_points, const geometry_msgs::Point & current_pose, const double forward_distance, const double backward_distance)
{
  objects.objects.erase(std::remove_if(objects.objects.begin(), objects.objects.end(), [&](const auto & object) {
    return !filter::position_filter(object, path_points, current_pose, forward_distance, -backward_distance);
  }), objects.objects.end());
}
void filter_objects_within_radius(autoware_perception_msgs::PredictedObjects & objects, const geometry_msgs::Point & reference_point, const double search_radius)
{
  objects.objects.erase(std::remove_if(objects.objects.begin(), objects.objects.end(), [&](const auto & object) {
    return !filter::is_within_circle(object.kinematics.initial_pose_with_covariance.pose.position, reference_point, search_radius);
  }), objects.objects.end());
}
bool is_target_object_type(const autoware_perception_msgs::PredictedObject & object, const ObjectTypesToCheck & target_object_types)
{
  const auto highest = std::max_element(object.classification.begin(), object.classification.end(), [](const auto & first, const auto & second) {
    return first.probability < second.probability;
  });
  const auto label = highest == object.classification.end() ? autoware_perception_msgs::ObjectClassification::UNKNOWN : highest->label;
  using Classification = autoware_perception_msgs::ObjectClassification;
  return (label == Classification::CAR && target_object_types.check_car) ||
    (label == Classification::TRUCK && target_object_types.check_truck) ||
    (label == Classification::BUS && target_object_types.check_bus) ||
    (label == Classification::TRAILER && target_object_types.check_trailer) ||
    (label == Classification::UNKNOWN && target_object_types.check_unknown) ||
    (label == Classification::BICYCLE && target_object_types.check_bicycle) ||
    (label == Classification::MOTORCYCLE && target_object_types.check_motorcycle) ||
    (label == Classification::PEDESTRIAN && target_object_types.check_pedestrian);
}
void filter_objects_by_class(autoware_perception_msgs::PredictedObjects & objects, const ObjectTypesToCheck & target_object_types)
{
  objects.objects.erase(std::remove_if(objects.objects.begin(), objects.objects.end(), [&](const auto & object) {
    return !is_target_object_type(object, target_object_types);
  }), objects.objects.end());
}
bool is_centroid_within_lanelet(
  const autoware_perception_msgs::PredictedObject & object,
  const lanelet::ConstLanelet & lanelet, const double yaw_threshold)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const lanelet::BasicPoint2d centroid{object_pose.position.x, object_pose.position.y};
  if (!boost::geometry::within(centroid, lanelet.polygon2d().basicPolygon())) return false;
  const lanelet::BasicPoint3d search_point{
    object_pose.position.x, object_pose.position.y, object_pose.position.z};
  const auto closest_pose = autoware::experimental::lanelet2_utils::get_closest_center_pose(
    lanelet, search_point);
  return std::abs(autoware_utils_geometry::calc_yaw_deviation(closest_pose, object_pose)) < yaw_threshold;
}
bool is_centroid_within_lanelets(const autoware_perception_msgs::PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) return false;
  const auto & point = object.kinematics.initial_pose_with_covariance.pose.position;
  const lanelet::BasicPoint2d centroid{point.x, point.y};
  return std::any_of(target_lanelets.begin(), target_lanelets.end(), [&](const auto & lanelet) {
    return boost::geometry::within(centroid, lanelet.polygon2d().basicPolygon());
  });
}
bool is_polygon_overlap_lanelet(
  const autoware_perception_msgs::PredictedObject & object,
  const lanelet::ConstLanelet & lanelet)
{
  const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
  return !boost::geometry::disjoint(object_polygon, lanelet.polygon2d().basicPolygon());
}
std::pair<std::vector<size_t>, std::vector<size_t>> separate_object_indices_by_lanelets(
  const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const autoware_perception_msgs::PredictedObject &,
                           const lanelet::ConstLanelet &, double)> & condition,
  const double yaw_threshold)
{
  if (target_lanelets.empty()) return {};
  std::vector<size_t> target_indices;
  std::vector<size_t> other_indices;
  for (size_t index = 0; index < objects.objects.size(); ++index) {
    const auto found = std::find_if(target_lanelets.begin(), target_lanelets.end(), [&](const auto & lanelet) {
      return condition(objects.objects.at(index), lanelet, yaw_threshold);
    });
    (found == target_lanelets.end() ? other_indices : target_indices).push_back(index);
  }
  return {target_indices, other_indices};
}
std::pair<autoware_perception_msgs::PredictedObjects, autoware_perception_msgs::PredictedObjects>
separate_objects_by_lanelets(
  const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & target_lanelets,
  const std::function<bool(const autoware_perception_msgs::PredictedObject &,
                           const lanelet::ConstLanelet &, double)> & condition,
  const double yaw_threshold)
{
  autoware_perception_msgs::PredictedObjects target_objects;
  autoware_perception_msgs::PredictedObjects other_objects;
  const auto [target_indices, other_indices] = separate_object_indices_by_lanelets(
    objects, target_lanelets, condition, yaw_threshold);
  target_objects.objects.reserve(target_indices.size());
  other_objects.objects.reserve(other_indices.size());
  for (const auto index : target_indices) target_objects.objects.push_back(objects.objects.at(index));
  for (const auto index : other_indices) other_objects.objects.push_back(objects.objects.at(index));
  return {target_objects, other_objects};
}
TargetObjectsOnLane create_target_objects_on_lane(
  const lanelet::ConstLanelets & current_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware_perception_msgs::PredictedObjects & objects,
  const ObjectLaneConfiguration & configuration, const bool include_opposite,
  const bool invert_opposite, const double time_horizon, const double time_resolution)
{
  TargetObjectsOnLane targets;
  if (!route_handler) return targets;
  lanelet::ConstLanelets left, right, shoulder;
  for (const auto & lane : current_lanes) {
    const auto left_lanes = route_handler->getAllLeftSharedLinestringLanelets(lane, include_opposite, invert_opposite);
    const auto right_lanes = route_handler->getAllRightSharedLinestringLanelets(lane, include_opposite, invert_opposite);
    left.insert(left.end(), left_lanes.begin(), left_lanes.end());
    right.insert(right.end(), right_lanes.begin(), right_lanes.end());
    if (const auto lanelet = route_handler->getLeftShoulderLanelet(lane)) shoulder.push_back(*lanelet);
  }
  const auto append = [&](ExtendedPredictedObjects & output, const lanelet::ConstLanelets & lanes) {
    for (const auto & object : objects.objects) if (is_centroid_within_lanelets(object, lanes))
      output.push_back(transform_extended_predicted_object(object, time_horizon, time_resolution));
  };
  if (configuration.check_current_lane) append(targets.on_current_lane, current_lanes);
  if (configuration.check_left_lane) append(targets.on_left_lane, left);
  if (configuration.check_right_lane) append(targets.on_right_lane, right);
  if (configuration.check_shoulder_lane) append(targets.on_shoulder_lane, shoulder);
  return targets;
}
}
