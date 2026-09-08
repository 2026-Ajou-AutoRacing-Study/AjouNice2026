#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <functional>
#include <limits>
#include <boost/geometry.hpp>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
// Original safety_check.cpp:277-295 and 532-578. Keep source polygon union,
// source first-collision short circuit and all debug writes.
bool checkSafetyWithIntegralPredictedPolygon(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const bool check_all_predicted_path,
  const IntegralPredictedPolygonParams & params, CollisionCheckDebugMap & debug_map)
{
  const auto filtered_ego_path = filterPredictedPathByTimeHorizon(
    ego_predicted_path, params.time_horizon);
  std::vector<Polygon2d> extended_ego_polygons;
  for (const auto & elem : filtered_ego_path) {
    extended_ego_polygons.push_back(autoware_utils::to_footprint(
      elem.pose, vehicle_info.max_longitudinal_offset_m + params.forward_margin,
      vehicle_info.rear_overhang_m + params.backward_margin,
      vehicle_info.vehicle_width_m + params.lat_margin * 2));
  }
  const auto filtered_path_objects = filterObjectPredictedPathByTimeHorizon(
    objects, params.time_horizon, check_all_predicted_path);
  Polygon2d ego_integral_polygon{};
  for (const auto & ego_polygon : extended_ego_polygons) {
    std::vector<Polygon2d> unions{};
    boost::geometry::union_(ego_integral_polygon, ego_polygon, unions);
    if (!unions.empty()) {
      ego_integral_polygon = unions.front();
      boost::geometry::correct(ego_integral_polygon);
    }
  }
  for (const auto & object : filtered_path_objects) {
    auto debug_pair = createObjectDebug(object);
    for (const auto & path : object.predicted_paths) {
      for (const auto & pose_with_poly : path.path) {
        if (boost::geometry::intersects(ego_integral_polygon, pose_with_poly.polygon)) {
          debug_pair.second.ego_predicted_path = ego_predicted_path;
          debug_pair.second.obj_predicted_path = path.path;
          debug_pair.second.object_predicted_path = path.path;
          debug_pair.second.extended_obj_polygon = pose_with_poly.polygon;
          debug_pair.second.extended_object_polygon = pose_with_poly.polygon;
          debug_pair.second.extended_ego_polygon = ego_integral_polygon;
          updateCollisionCheckDebugMap(debug_map, debug_pair, false);
          return false;
        }
      }
    }
    updateCollisionCheckDebugMap(debug_map, debug_pair, true);
  }
  return true;
}
// Original safety_check.cpp:921-954: offset-pose fallback and polygon distance.
double shortest_distance_from_ego_footprint_to_objects_on_path(
  const PathWithLaneId & path, const autoware_perception_msgs::PredictedObjects & objects,
  const BehaviorPathPlannerParameters & parameters, const bool use_offset_ego_pose)
{
  using namespace autoware::motion_utils;
  const auto & p = parameters;
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : objects.objects) {
    const auto & object_point = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto ego_pose = std::invoke([&]() -> geometry_msgs::Pose {
      if (use_offset_ego_pose) {
        const size_t nearest_segment_idx = findNearestSegmentIndex(path.points, object_point);
        const double offset_length =
          calcLongitudinalOffsetToSegment(path.points, nearest_segment_idx, object_point);
        const auto ego_pose_opt =
          calcLongitudinalOffsetPose(path.points, nearest_segment_idx, offset_length);
        if (ego_pose_opt.has_value()) return ego_pose_opt.value();
      }
      const auto ego_nearest_idx = findNearestIndex(path.points, object_point);
      return path.points.at(ego_nearest_idx).point.pose;
    });
    const auto ego_footprint =
      autoware_utils::to_footprint(ego_pose, p.base_link2front, p.base_link2rear, p.vehicle_width);
    const double distance =
      boost::geometry::distance(ego_footprint, autoware_utils::to_polygon2d(object));
    min_distance = std::min(min_distance, distance);
  }
  return min_distance;
}
// Restore the original per-object/per-prediction short circuit and debug writes.
bool checkSafetyWithRSS(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const std::vector<ExtendedPredictedObject> & objects, CollisionCheckDebugMap & debug_map,
  const BehaviorPathPlannerParameters & parameters, const RSSparams & rss_params,
  const bool check_all_predicted_path, const double hysteresis_factor,
  const double yaw_difference_th)
{
  return !std::any_of(objects.begin(), objects.end(), [&](const auto & object) {
    auto debug = createObjectDebug(object);
    const auto paths = get_object_predicted_paths(object, check_all_predicted_path);
    return std::any_of(paths.begin(), paths.end(), [&](const auto & path) {
      const bool collision = !checkCollision(planned_path, ego_predicted_path, object, path,
        parameters, rss_params, hysteresis_factor, yaw_difference_th, debug.second);
      updateCollisionCheckDebugMap(debug_map, debug, !collision);
      return collision;
    });
  });
}

autoware_perception_msgs::PredictedPath convertToPredictedPath(
  const std::vector<PoseWithVelocityStamped> & path, const double time_resolution)
{
  autoware_perception_msgs::PredictedPath output;
  const ros::Duration duration(time_resolution);
  output.time_step.sec = duration.sec;
  output.time_step.nanosec = duration.nsec;
  output.path.reserve(path.size());
  for (const auto & point : path) output.path.push_back(point.pose);
  return output;
}

bool checkCollision(
  const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const RSSparams & rss_parameters,
  const double hysteresis_factor, const double yaw_difference_th, CollisionCheckDebug & debug)
{
  ExtendedPredictedObject one_object = target_object;
  one_object.predicted_paths = {target_object_path};
  CollisionCheckDebugMap evaluated;
  const bool is_safe = check_safety_with_rss(
    planned_path, predicted_ego_path, common_parameters.vehicle_info, {one_object}, evaluated,
    rss_parameters, true, hysteresis_factor, yaw_difference_th);

  const auto found = evaluated.find(target_object.uuid.uuid);
  if (found != evaluated.end()) debug = found->second;
  debug.current_obj_pose = target_object.initial_pose;
  debug.current_twist = target_object.initial_twist;
  debug.obj_shape = target_object.shape;
  debug.obj_predicted_path = target_object_path.path;
  if (!debug.obj_predicted_path.empty()) {
    debug.expected_obj_pose = debug.expected_object_pose;
    debug.extended_obj_polygon = debug.extended_object_polygon;
  }
  return is_safe;
}

CollisionCheckDebugPair createObjectDebug(const ExtendedPredictedObject & object)
{
  CollisionCheckDebug debug;
  debug.current_obj_pose = object.initial_pose;
  debug.current_twist = object.initial_twist;
  debug.obj_shape = object.shape;
  debug.extended_obj_polygon = object.initial_polygon;
  debug.extended_object_polygon = object.initial_polygon;
  return {object.uuid.uuid, std::move(debug)};
}

void updateCollisionCheckDebugMap(
  CollisionCheckDebugMap & debug_map, CollisionCheckDebugPair & object_debug, const bool is_safe)
{
  object_debug.second.is_safe = is_safe;
  debug_map[object_debug.first] = object_debug.second;
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker
