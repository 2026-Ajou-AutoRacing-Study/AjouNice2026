#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/prepared_path_projection.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <tf2/utils.h>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
double calc_rss_distance(const double front_object_velocity, const double rear_object_velocity, const RSSparams & params)
{
  const auto stopping_distance = [](const double velocity, const double acceleration) {
    const double deceleration = acceleration < -1e-3 ? acceleration : -1.0;
    return -std::pow(velocity, 2) / (2.0 * deceleration);
  };
  const double reaction_time = params.rear_vehicle_reaction_time + params.rear_vehicle_safety_time_margin;
  return rear_object_velocity * reaction_time + stopping_distance(rear_object_velocity, params.rear_vehicle_deceleration) - stopping_distance(front_object_velocity, params.front_vehicle_deceleration);
}
double calc_minimum_longitudinal_length(const double front_object_velocity, const double rear_object_velocity, const RSSparams & params)
{
  return params.longitudinal_velocity_delta_time * std::abs(std::max(front_object_velocity, rear_object_velocity)) + params.longitudinal_distance_min_threshold;
}
double calc_obstacle_min_length(const autoware_perception_msgs::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::Shape::BOUNDING_BOX) return std::min(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  if (shape.type == autoware_perception_msgs::Shape::CYLINDER) return shape.dimensions.x / 2.0;
  if (shape.type == autoware_perception_msgs::Shape::POLYGON) {
    double result = std::numeric_limits<double>::max();
    for (const auto & point : shape.footprint.points) result = std::min(result, std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)));
    return result;
  }
  throw std::logic_error("unsupported obstacle shape");
}
double calc_obstacle_max_length(const autoware_perception_msgs::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::Shape::BOUNDING_BOX) return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  if (shape.type == autoware_perception_msgs::Shape::CYLINDER) return shape.dimensions.x / 2.0;
  if (shape.type == autoware_perception_msgs::Shape::POLYGON) {
    double result = 0.0;
    for (const auto & point : shape.footprint.points) result = std::max(result, std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)));
    return result;
  }
  throw std::logic_error("unsupported obstacle shape");
}
bool is_target_object_oncoming(const geometry_msgs::Pose & vehicle_pose, const geometry_msgs::Pose & object_pose, const double angle_threshold)
{
  double difference = tf2::getYaw(vehicle_pose.orientation) - tf2::getYaw(object_pose.orientation);
  while (difference > M_PI) difference -= 2.0 * M_PI;
  while (difference < -M_PI) difference += 2.0 * M_PI;
  return std::abs(difference) > angle_threshold;
}
bool is_target_object_front(const geometry_msgs::Pose & ego_pose, const autoware_utils_geometry::Polygon2d & object_polygon, const double base_to_front)
{
  const double yaw = tf2::getYaw(ego_pose.orientation);
  const double origin_x = ego_pose.position.x + base_to_front * std::cos(yaw);
  const double origin_y = ego_pose.position.y + base_to_front * std::sin(yaw);
  return std::any_of(object_polygon.outer().begin(), object_polygon.outer().end(), [&](const auto & point) {
    return (point.x() - origin_x) * std::cos(yaw) + (point.y() - origin_y) * std::sin(yaw) > 0.0;
  });
}
autoware_utils_geometry::Polygon2d create_extended_ego_polygon(const geometry_msgs::Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double longitudinal_length, const double lateral_margin, const bool is_stopped_object)
{
  auto footprint = vehicle_info.createFootprint(
    lateral_margin, lateral_margin, lateral_margin,
    is_stopped_object ? longitudinal_length / 2.0 : longitudinal_length,
    is_stopped_object ? longitudinal_length / 2.0 : 0.0);
  if (footprint.size() >= 6) {
    footprint.erase(footprint.begin() + 5);
    footprint.erase(footprint.begin() + 2);
  }
  auto transformed = autoware_utils_geometry::transform_vector(footprint, autoware_utils_geometry::pose2transform(base_link_pose));
  autoware_utils_geometry::Polygon2d polygon; polygon.outer() = transformed;
  return autoware_utils_geometry::is_clockwise(polygon) ? polygon : autoware_utils_geometry::inverse_clockwise(polygon);
}
autoware_utils_geometry::Polygon2d create_extended_ego_polygon_along_path(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const geometry_msgs::Pose & base_link_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double longitudinal_length,
  const double lateral_margin, const bool is_stopped_object)
{
  return create_extended_ego_polygon_along_path(planned_path,base_link_pose,vehicle_info,
    longitudinal_length,lateral_margin,is_stopped_object,nullptr);
}
autoware_utils_geometry::Polygon2d create_extended_ego_polygon_along_path(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const geometry_msgs::Pose & base_link_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double longitudinal_length,
  const double lateral_margin, const bool is_stopped_object,
  const PreparedPathProjection * projection)
{
  // A workspace from any other path must never change this path's geometry.
  if(projection&&!projection->matches(planned_path))projection=nullptr;
  const double front_offset = vehicle_info.max_longitudinal_offset_m +
    (is_stopped_object ? longitudinal_length / 2.0 : longitudinal_length);
  const double rear_offset = -vehicle_info.rear_overhang_m -
    (is_stopped_object ? longitudinal_length / 2.0 : 0.0);
  const double lateral_offset = vehicle_info.vehicle_width_m / 2.0 + lateral_margin;
  const auto target_pose = projection?
    projection->offset_pose(base_link_pose.position,longitudinal_length):
    autoware::motion_utils::calcLongitudinalOffsetPose(
      planned_path.points, base_link_pose.position, longitudinal_length);
  if (!target_pose) {
    return create_extended_ego_polygon(
      base_link_pose, vehicle_info, longitudinal_length, lateral_margin, is_stopped_object);
  }
  const size_t start_index = projection?projection->nearest(base_link_pose.position):
    autoware::motion_utils::findNearestSegmentIndex(planned_path.points,base_link_pose.position);
  const size_t end_index = projection?projection->nearest(target_pose->position):
    autoware::motion_utils::findNearestSegmentIndex(planned_path.points,target_pose->position);
  autoware_utils_geometry::Polygon2d polygon;
  const auto append = [&](const geometry_msgs::Pose & pose) {
    polygon.outer().emplace_back(pose.position.x, pose.position.y);
  };
  append(autoware_utils_geometry::calc_offset_pose(base_link_pose, rear_offset, lateral_offset, 0.0));
  for (size_t index = start_index + 1; index < end_index + 1; ++index) {
    const auto pose = autoware_utils_geometry::get_pose(planned_path.points.at(index));
    append(autoware_utils_geometry::calc_offset_pose(pose, 0.0, lateral_offset, 0.0));
  }
  append(autoware_utils_geometry::calc_offset_pose(*target_pose, vehicle_info.max_longitudinal_offset_m, lateral_offset, 0.0));
  append(autoware_utils_geometry::calc_offset_pose(*target_pose, vehicle_info.max_longitudinal_offset_m, -lateral_offset, 0.0));
  for (size_t index = end_index; index > start_index; --index) {
    const auto pose = autoware_utils_geometry::get_pose(planned_path.points.at(index));
    append(autoware_utils_geometry::calc_offset_pose(pose, 0.0, -lateral_offset, 0.0));
  }
  append(autoware_utils_geometry::calc_offset_pose(base_link_pose, rear_offset, -lateral_offset, 0.0));
  append(autoware_utils_geometry::calc_offset_pose(base_link_pose, rear_offset, lateral_offset, 0.0));
  return autoware_utils_geometry::is_clockwise(polygon) ? polygon : autoware_utils_geometry::inverse_clockwise(polygon);
}
autoware_utils_geometry::Polygon2d create_extended_object_polygon(
  const PoseWithVelocityAndPolygonStamped & object_pose, const double longitudinal_length,
  const double lateral_margin, const bool is_stopped_object)
{
  const auto & object_polygon = object_pose.polygon;
  if (object_polygon.outer().empty()) return object_polygon;
  const double yaw = tf2::getYaw(object_pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : object_polygon.outer()) {
    const double dx = point.x() - object_pose.pose.position.x;
    const double dy = point.y() - object_pose.pose.position.y;
    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    min_x = std::min(min_x, local_x); max_x = std::max(max_x, local_x);
    min_y = std::min(min_y, local_y); max_y = std::max(max_y, local_y);
  }
  const double forward = max_x + (is_stopped_object ? longitudinal_length / 2.0 : longitudinal_length);
  const double backward = min_x - (is_stopped_object ? longitudinal_length / 2.0 : 0.0);
  const double left = max_y + lateral_margin;
  const double right = min_y - lateral_margin;
  const auto append_world = [&](autoware_utils_geometry::Polygon2d & polygon, const double x, const double y) {
    polygon.outer().emplace_back(
      object_pose.pose.position.x + c * x - s * y,
      object_pose.pose.position.y + s * x + c * y);
  };
  autoware_utils_geometry::Polygon2d polygon;
  append_world(polygon, forward, left); append_world(polygon, forward, right);
  append_world(polygon, backward, right); append_world(polygon, backward, left);
  polygon.outer().push_back(polygon.outer().front());
  return autoware_utils_geometry::is_clockwise(polygon) ? polygon : autoware_utils_geometry::inverse_clockwise(polygon);
}
std::vector<autoware_utils_geometry::Polygon2d> create_extended_ego_polygons(
  const std::vector<PoseWithVelocityStamped> & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double forward_margin, const double backward_margin, const double lateral_margin)
{
  std::vector<autoware_utils_geometry::Polygon2d> polygons;
  polygons.reserve(predicted_path.size());
  for (const auto & point : predicted_path) {
    const auto footprint = vehicle_info.createFootprint(
      lateral_margin, lateral_margin, lateral_margin, forward_margin, backward_margin);
    const auto transformed = autoware_utils_geometry::transform_vector(
      footprint, autoware_utils_geometry::pose2transform(point.pose));
    autoware_utils_geometry::Polygon2d polygon;
    polygon.outer() = transformed;
    polygons.push_back(autoware_utils_geometry::is_clockwise(polygon) ? polygon :
      autoware_utils_geometry::inverse_clockwise(polygon));
  }
  return polygons;
}
std::optional<PoseWithVelocityStamped> calc_interpolated_pose_with_velocity(const std::vector<PoseWithVelocityStamped> & path, const double relative_time)
{
  if (path.empty() || relative_time < 0.0) return std::nullopt;
  constexpr double epsilon = 1e-6;
  for (size_t index = 1; index < path.size(); ++index) {
    const auto & point = path[index];
    const auto & previous = path[index - 1];
    if (relative_time < point.time + epsilon) {
      const double time_step = point.time - previous.time;
      if (time_step <= 0.0) return std::nullopt;
      const double ratio = std::clamp((relative_time - previous.time) / time_step, 0.0, 1.0);
      return PoseWithVelocityStamped{relative_time, autoware_utils_geometry::calc_interpolated_pose(previous.pose, point.pose, ratio, false), previous.velocity + (point.velocity - previous.velocity) * ratio};
    }
  }
  return std::nullopt;
}
std::vector<PoseWithVelocityStamped> filter_predicted_path_by_time_horizon(const std::vector<PoseWithVelocityStamped> & path, const double time_horizon)
{
  std::vector<PoseWithVelocityStamped> filtered;
  for (const auto & element : path) {
    if (element.time < time_horizon) filtered.push_back(element);
    else break;
  }
  if (const auto interpolated = calc_interpolated_pose_with_velocity(path, time_horizon)) filtered.push_back(*interpolated);
  return filtered;
}
std::vector<PoseWithVelocityStamped> filter_predicted_path_after_target_pose(
  const std::vector<PoseWithVelocityStamped> & path, const geometry_msgs::Pose & target_pose)
{
  if (path.empty()) return {};
  const auto target = std::min_element(path.begin(), path.end(), [&](const auto & first, const auto & second) {
    const auto squared_distance = [&](const auto & candidate) {
      const double dx = candidate.pose.position.x - target_pose.position.x;
      const double dy = candidate.pose.position.y - target_pose.position.y;
      return dx * dx + dy * dy;
    };
    return squared_distance(first) < squared_distance(second);
  });
  return {target, path.end()};
}
std::optional<PoseWithVelocityAndPolygonStamped> calc_interpolated_object_pose(
  const std::vector<PoseWithVelocityAndPolygonStamped> & path, const double relative_time,
  const autoware_perception_msgs::Shape & shape)
{
  std::vector<PoseWithVelocityStamped> poses;
  poses.reserve(path.size());
  for (const auto & point : path) poses.emplace_back(point.time, point.pose, point.velocity);
  const auto interpolated = calc_interpolated_pose_with_velocity(poses, relative_time);
  if (!interpolated) return std::nullopt;
  return PoseWithVelocityAndPolygonStamped{relative_time, interpolated->pose,
    interpolated->velocity, autoware_utils_geometry::to_polygon2d(interpolated->pose, shape)};
}
std::pair<bool, bool> check_objects_collision_rough(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::PredictedObjects & objects,
  const double min_margin_threshold, const double max_margin_threshold,
  const autoware::behavior_path_planner::BehaviorPathPlannerParameters & parameters,
  const bool use_offset_ego_point)
{
  if (path.points.empty()) return {false, false};
  std::pair<bool, bool> has_collision{false, false};
  for (const auto & object : objects.objects) {
    const auto & object_point = object.kinematics.initial_pose_with_covariance.pose.position;
    geometry_msgs::Point ego_point;
    if (use_offset_ego_point && path.points.size() >= 2U) {
      const auto segment = autoware::motion_utils::findNearestSegmentIndex(path.points, object_point);
      const auto offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        path.points, segment, object_point);
      const auto projected = autoware::motion_utils::calcLongitudinalOffsetPoint(
        path.points, segment, offset);
      ego_point = projected ? *projected : path.points.at(
        autoware::motion_utils::findNearestIndex(path.points, object_point)).point.pose.position;
    } else {
      ego_point = path.points.at(
        autoware::motion_utils::findNearestIndex(path.points, object_point)).point.pose.position;
    }
    const double distance = std::hypot(ego_point.x - object_point.x, ego_point.y - object_point.y);
    const double object_min = calc_obstacle_min_length(object.shape);
    const double object_max = calc_obstacle_max_length(object.shape);
    const double ego_min = std::min({parameters.vehicle_width / 2.0,
      parameters.front_overhang / 2.0, parameters.rear_overhang / 2.0});
    const double ego_max = std::max(std::hypot(parameters.vehicle_width / 2.0, parameters.front_overhang),
      std::hypot(parameters.vehicle_width / 2.0, parameters.rear_overhang));
    has_collision.first = has_collision.first || distance - object_max - ego_max < min_margin_threshold;
    has_collision.second = has_collision.second || distance - object_min - ego_min < max_margin_threshold;
  }
  return has_collision;
}
bool check_polygons_intersect(const std::vector<autoware_utils_geometry::Polygon2d> & first, const std::vector<autoware_utils_geometry::Polygon2d> & second)
{
  return std::any_of(first.begin(), first.end(), [&](const auto & left) {
    return std::any_of(second.begin(), second.end(), [&](const auto & right) {
      return boost::geometry::intersects(left, right);
    });
  });
}
std::optional<geometry_msgs::Pose> calc_object_predicted_pose(const autoware_perception_msgs::PredictedPath & path, const double relative_time)
{
  const auto pose = autoware::object_recognition_utils::calcInterpolatedPose(path, relative_time);
  if (!pose) return std::nullopt;
  return *pose;
}
std::vector<PredictedPathWithPolygon> transform_object_predicted_paths(const autoware_perception_msgs::PredictedObject & object, const double safety_check_time_horizon, const double safety_check_time_resolution)
{
  std::vector<PredictedPathWithPolygon> transformed(object.kinematics.predicted_paths.size());
  if (safety_check_time_resolution <= 0.0) return transformed;
  const double velocity = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  for (size_t index = 0; index < object.kinematics.predicted_paths.size(); ++index) {
    const auto & source_path = object.kinematics.predicted_paths[index];
    auto & destination = transformed[index]; destination.confidence = source_path.confidence;
    for (double time = 0.0; time < safety_check_time_horizon + 1e-3; time += safety_check_time_resolution) {
      if (const auto pose = calc_object_predicted_pose(source_path, time)) {
        destination.path.emplace_back(time, *pose, velocity, autoware_utils_geometry::to_polygon2d(*pose, object.shape));
      }
    }
  }
  return transformed;
}
ExtendedPredictedObject transform_extended_predicted_object(
  const autoware_perception_msgs::PredictedObject & object,
  const double safety_check_time_horizon, const double safety_check_time_resolution)
{
  ExtendedPredictedObject transformed{object};
  transformed.predicted_paths = transform_object_predicted_paths(
    object, safety_check_time_horizon, safety_check_time_resolution);
  return transformed;
}
std::vector<PredictedPathWithPolygon> get_object_predicted_paths(
  const ExtendedPredictedObject & object, const bool use_all_predicted_paths)
{
  if (use_all_predicted_paths || object.predicted_paths.empty()) return object.predicted_paths;
  const auto highest = std::max_element(
    object.predicted_paths.begin(), object.predicted_paths.end(),
    [](const auto & left, const auto & right) { return left.confidence < right.confidence; });
  const float confidence = highest->confidence;
  std::vector<PredictedPathWithPolygon> selected;
  std::copy_if(object.predicted_paths.begin(), object.predicted_paths.end(),
    std::back_inserter(selected), [confidence](const auto & path) {
      return path.confidence == confidence;
    });
  return selected;
}
ExtendedPredictedObject filter_extended_predicted_object_by_time_horizon(
  const ExtendedPredictedObject & object, const double time_horizon,
  const bool use_all_predicted_paths)
{
  auto filtered = object;
  filtered.predicted_paths = get_object_predicted_paths(object, use_all_predicted_paths);
  for (auto & predicted : filtered.predicted_paths) {
    std::vector<PoseWithVelocityAndPolygonStamped> cropped;
    for (const auto & point : predicted.path) {
      if (point.time < time_horizon) cropped.push_back(point); else break;
    }
    if (const auto endpoint = calc_interpolated_object_pose(predicted.path, time_horizon, object.shape)) cropped.push_back(*endpoint);
    predicted.path = std::move(cropped);
  }
  return filtered;
}
bool check_safety_with_integral_predicted_polygons(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const bool use_all_predicted_paths,
  const IntegralPredictedPolygonParams & params)
{
  const auto cropped_ego = filter_predicted_path_by_time_horizon(
    ego_predicted_path, params.time_horizon);
  const auto ego_polygons = create_extended_ego_polygons(
    cropped_ego, vehicle_info, params.forward_margin, params.backward_margin, params.lat_margin);
  for (const auto & object : objects) {
    const auto cropped = filter_extended_predicted_object_by_time_horizon(
      object, params.time_horizon, use_all_predicted_paths);
    for (const auto & path : cropped.predicted_paths) {
      for (const auto & pose : path.path) {
        if (std::any_of(ego_polygons.begin(), ego_polygons.end(), [&](const auto & ego) {
              return boost::geometry::intersects(ego, pose.polygon);
            })) return false;
      }
    }
  }
  return true;
}
bool check_safety_with_rss_rectangle(
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const RSSparams & params,
  const bool use_all_predicted_paths, const double hysteresis_factor,
  const double yaw_difference_threshold)
{
  if (params.extended_polygon_policy != "rectangle" || hysteresis_factor < 0.0) return false;
  for (const auto & object : objects) {
    for (const auto & path : get_object_predicted_paths(object, use_all_predicted_paths)) {
      for (const auto & object_pose : path.path) {
        const auto ego = calc_interpolated_pose_with_velocity(ego_predicted_path, object_pose.time);
        if (!ego) continue;
        if (is_target_object_oncoming(ego->pose, object_pose.pose, yaw_difference_threshold)) continue;
        const auto ego_polygon = create_extended_ego_polygon(ego->pose, vehicle_info, 0.0, 0.0, false);
        if (boost::geometry::intersects(ego_polygon, object_pose.polygon)) return false;
        const bool object_is_front = is_target_object_front(
          ego->pose, object_pose.polygon, vehicle_info.max_longitudinal_offset_m);
        const double ego_velocity = ego->velocity;
        const auto velocities = object_is_front ? std::pair{object_pose.velocity, ego_velocity}
                                                : std::pair{ego_velocity, object_pose.velocity};
        const double extension = std::max(
          calc_rss_distance(velocities.first, velocities.second, params),
          calc_minimum_longitudinal_length(velocities.first, velocities.second, params)) * hysteresis_factor;
        const double lateral = params.lateral_distance_max_threshold * hysteresis_factor;
        const bool stopped = object_pose.velocity < 0.3;
        const auto extended_ego = object_is_front
          ? create_extended_ego_polygon(ego->pose, vehicle_info, extension, lateral, stopped)
          : ego_polygon;
        const auto extended_object = object_is_front ? object_pose.polygon
          : create_extended_object_polygon(object_pose, extension, lateral, stopped);
        if (boost::geometry::intersects(extended_ego, extended_object)) return false;
      }
    }
  }
  return true;
}
bool check_safety_with_rss(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, const RSSparams & params,
  const bool use_all_predicted_paths, const double hysteresis_factor,
  const double yaw_difference_threshold)
{
  if ((params.extended_polygon_policy != "rectangle" &&
       params.extended_polygon_policy != "along_path") || hysteresis_factor < 0.0) return false;
  for (const auto & object : objects) {
    for (const auto & path : get_object_predicted_paths(object, use_all_predicted_paths)) {
      for (const auto & object_pose : path.path) {
        const auto ego = calc_interpolated_pose_with_velocity(ego_predicted_path, object_pose.time);
        if (!ego || is_target_object_oncoming(ego->pose, object_pose.pose, yaw_difference_threshold)) continue;
        const auto ego_polygon = create_extended_ego_polygon(ego->pose, vehicle_info, 0.0, 0.0, false);
        if (boost::geometry::intersects(ego_polygon, object_pose.polygon)) return false;
        const bool object_is_front = is_target_object_front(
          ego->pose, object_pose.polygon, vehicle_info.max_longitudinal_offset_m);
        const auto velocities = object_is_front ? std::pair{object_pose.velocity, ego->velocity}
                                                : std::pair{ego->velocity, object_pose.velocity};
        const double extension = std::max(
          calc_rss_distance(velocities.first, velocities.second, params),
          calc_minimum_longitudinal_length(velocities.first, velocities.second, params)) * hysteresis_factor;
        const double lateral = params.lateral_distance_max_threshold * hysteresis_factor;
        const bool stopped = object_pose.velocity < 0.3;
        const auto extended_ego = !object_is_front ? ego_polygon :
          (params.extended_polygon_policy == "along_path"
            ? create_extended_ego_polygon_along_path(
                planned_path, ego->pose, vehicle_info, extension, lateral, stopped)
            : create_extended_ego_polygon(ego->pose, vehicle_info, extension, lateral, stopped));
        const auto extended_object = object_is_front ? object_pose.polygon :
          create_extended_object_polygon(object_pose, extension, lateral, stopped);
        if (boost::geometry::intersects(extended_ego, extended_object)) return false;
      }
    }
  }
  return true;
}
bool check_safety_with_rss(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, CollisionCheckDebugMap & debug_map,
  const RSSparams & params, const bool use_all_predicted_paths,
  const double hysteresis_factor, const double yaw_difference_threshold)
{
  return check_safety_with_rss(planned_path,ego_predicted_path,vehicle_info,objects,debug_map,
    params,use_all_predicted_paths,hysteresis_factor,yaw_difference_threshold,nullptr);
}
bool check_safety_with_rss(
  const autoware_internal_planning_msgs::PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ExtendedPredictedObjects & objects, CollisionCheckDebugMap & debug_map,
  const RSSparams & params, const bool use_all_predicted_paths,
  const double hysteresis_factor, const double yaw_difference_threshold,
  const PreparedPathProjection * projection)
{
  if ((params.extended_polygon_policy != "rectangle" &&
       params.extended_polygon_policy != "along_path") || hysteresis_factor < 0.0) return false;
  for (const auto & object : objects) {
    CollisionCheckDebug debug;
    debug.expected_object_pose = object.initial_pose;
    debug.extended_object_polygon = object.initial_polygon;
    debug.ego_predicted_path = ego_predicted_path;
    const auto selected_paths = get_object_predicted_paths(object, use_all_predicted_paths);
    for (const auto & path : selected_paths) {
      debug.object_predicted_path = path.path;
      for (const auto & object_pose : path.path) {
        const auto ego = calc_interpolated_pose_with_velocity(ego_predicted_path, object_pose.time);
        if (!ego || is_target_object_oncoming(ego->pose, object_pose.pose, yaw_difference_threshold)) continue;

        const auto ego_polygon = create_extended_ego_polygon(ego->pose, vehicle_info, 0.0, 0.0, false);
        if (boost::geometry::intersects(ego_polygon, object_pose.polygon)) {
          debug.is_safe = false;
          debug.unsafe_reason = "overlap_polygon";
          debug.expected_ego_pose = ego->pose;
          debug.expected_object_pose = object_pose.pose;
          debug.extended_ego_polygon = ego_polygon;
          debug.extended_object_polygon = object_pose.polygon;
          debug_map[object.uuid.uuid] = std::move(debug);
          return false;
        }

        const bool object_is_front = is_target_object_front(
          ego->pose, object_pose.polygon, vehicle_info.max_longitudinal_offset_m);
        const auto velocities = object_is_front ? std::pair{object_pose.velocity, ego->velocity}
                                                : std::pair{ego->velocity, object_pose.velocity};
        const double extension = std::max(
          calc_rss_distance(velocities.first, velocities.second, params),
          calc_minimum_longitudinal_length(velocities.first, velocities.second, params)) * hysteresis_factor;
        const double lateral = params.lateral_distance_max_threshold * hysteresis_factor;
        const bool stopped = object_pose.velocity < 0.3;
        const auto extended_ego = !object_is_front ? ego_polygon :
          (params.extended_polygon_policy == "along_path"
            ? create_extended_ego_polygon_along_path(
                planned_path, ego->pose, vehicle_info, extension, lateral, stopped,projection)
            : create_extended_ego_polygon(ego->pose, vehicle_info, extension, lateral, stopped));
        const auto extended_object = object_is_front ? object_pose.polygon :
          create_extended_object_polygon(object_pose, extension, lateral, stopped);
        if (boost::geometry::intersects(extended_ego, extended_object)) {
          debug.is_safe = false;
          debug.unsafe_reason = "overlap_extended_polygon";
          debug.expected_ego_pose = ego->pose;
          debug.expected_object_pose = object_pose.pose;
          debug.extended_ego_polygon = extended_ego;
          debug.extended_object_polygon = extended_object;
          debug_map[object.uuid.uuid] = std::move(debug);
          return false;
        }
      }
    }
    debug.is_safe = true;
    debug_map[object.uuid.uuid] = std::move(debug);
  }
  return true;
}
}
