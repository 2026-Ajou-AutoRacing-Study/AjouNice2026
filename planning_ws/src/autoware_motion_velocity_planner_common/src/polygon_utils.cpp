#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::motion_velocity_planner::polygon_utils
{
namespace
{
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

Point2d point2d(const geometry_msgs::Point & point) { return {point.x, point.y}; }

Polygon2d createPoseFootprint(
  const geometry_msgs::Pose & pose, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double left_margin, const double right_margin)
{
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const auto point0 = autoware_utils_geometry::calc_offset_pose(
    pose, vehicle_info.max_longitudinal_offset_m, half_width + left_margin, 0.0).position;
  const auto point1 = autoware_utils_geometry::calc_offset_pose(
    pose, vehicle_info.max_longitudinal_offset_m, -half_width - right_margin, 0.0).position;
  const auto point2 = autoware_utils_geometry::calc_offset_pose(
    pose, -vehicle_info.rear_overhang_m, -half_width - right_margin, 0.0).position;
  const auto point3 = autoware_utils_geometry::calc_offset_pose(
    pose, -vehicle_info.rear_overhang_m, half_width + left_margin, 0.0).position;
  Polygon2d polygon;
  polygon.outer() = {point2d(point0), point2d(point1), point2d(point2), point2d(point3), point2d(point0)};
  boost::geometry::correct(polygon);
  return polygon;
}

std::optional<std::pair<geometry_msgs::Point, double>> findMaxPenetrationPoint(
  const std::vector<Polygon2d> & collision_polygons, const geometry_msgs::Pose & bumper_pose,
  const double trajectory_z)
{
  std::optional<std::pair<geometry_msgs::Point, double>> result;
  for (const auto & polygon : collision_polygons) {
    for (const auto & point : polygon.outer()) {
      geometry_msgs::Point collision;
      collision.x = point.x(); collision.y = point.y(); collision.z = trajectory_z;
      const double distance = std::abs(autoware_utils_geometry::inverse_transform_point(collision, bumper_pose).x);
      if (!result || distance > result->second) result = std::make_pair(collision, distance);
    }
  }
  return result;
}

std::vector<geometry_msgs::Pose> calculateErrorPosesImpl(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & current_ego_pose, const double time_to_convergence)
{
  if (trajectory.empty() || time_to_convergence <= 0.0) return {};
  const size_t nearest_index = autoware::motion_utils::findNearestSegmentIndex(trajectory, current_ego_pose.position);
  const auto ego_error = autoware_utils_geometry::inverse_transform_pose(current_ego_pose, trajectory.at(nearest_index).pose);
  const double lateral_error = ego_error.position.y;
  const double yaw_error = tf2::getYaw(ego_error.orientation);
  double elapsed = 0.0;
  std::vector<geometry_msgs::Pose> output;
  output.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size() && elapsed < time_to_convergence; ++i) {
    const double remaining = (time_to_convergence - elapsed) / time_to_convergence;
    geometry_msgs::Pose indexed_error;
    indexed_error.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw_error * remaining);
    indexed_error.position = autoware_utils_geometry::create_point(0.0, lateral_error * remaining, 0.0);
    output.push_back(autoware_utils_geometry::transform_pose(indexed_error, trajectory.at(i).pose));
    if (trajectory.at(i).longitudinal_velocity_mps != 0.0F && i + 1 < trajectory.size()) {
      elapsed += autoware_utils_geometry::calc_distance2d(trajectory.at(i).pose.position, trajectory.at(i + 1).pose.position) /
                 std::abs(trajectory.at(i).longitudinal_velocity_mps);
    } else {
      elapsed = std::numeric_limits<double>::max();
    }
  }
  return output;
}

}

std::vector<geometry_msgs::Pose> calculateErrorPoses(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & current_ego_pose, const double time_to_convergence)
{
  return calculateErrorPosesImpl(trajectory, current_ego_pose, time_to_convergence);
}

std::optional<std::pair<geometry_msgs::Point, double>> getCollisionPoint(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const std::vector<Polygon2d> & trajectory_polygons, const geometry_msgs::Point & object_position,
  const Polygon2d & object_polygon, const double x_offset_to_bumper)
{
  if (trajectory.size() != trajectory_polygons.size()) return std::nullopt;
  const double object_maximum_length = boost::geometry::perimeter(object_polygon) * 0.5;
  std::optional<std::pair<geometry_msgs::Point, double>> nearest;
  for (size_t i = 0; i < trajectory_polygons.size(); ++i) {
    const double ego_maximum_length = boost::geometry::perimeter(trajectory_polygons.at(i)) * 0.5;
    if (autoware_utils_geometry::calc_distance2d(trajectory.at(i).pose.position, object_position) >
      object_maximum_length + ego_maximum_length) continue;
    const double arc_length = autoware::motion_utils::calcSignedArcLength(trajectory, 0, i);
    if (nearest && std::abs(arc_length - x_offset_to_bumper) > std::abs(nearest->second)) break;
    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(trajectory_polygons.at(i), object_polygon, collision_polygons);
    if (collision_polygons.empty()) continue;
    const auto bumper_pose = autoware_utils_geometry::calc_offset_pose(
      trajectory.at(i).pose, x_offset_to_bumper, 0.0, 0.0);
    const auto penetration = findMaxPenetrationPoint(collision_polygons, bumper_pose, trajectory.at(i).pose.position.z);
    if (!penetration) continue;
    const double distance_to_collision = arc_length - penetration->second;
    if (!nearest || std::abs(distance_to_collision) < std::abs(nearest->second)) {
      nearest = std::make_pair(penetration->first, distance_to_collision);
    }
  }
  return nearest;
}

std::vector<double> calcFrontOuterWheelOffTracking(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  auto curvature = autoware::motion_utils::calcCurvature(trajectory);
  if (!autoware::motion_utils::isDrivingForward(trajectory)) {
    for (auto & value : curvature) value = -value;
  }
  for (auto & value : curvature) {
    const double outer = value / (1.0 + std::abs(value) * vehicle_info.vehicle_width_m / 2.0);
    value = -vehicle_info.wheel_base_m * std::tan(0.5 * std::atan(outer * vehicle_info.wheel_base_m));
  }
  return curvature;
}

std::vector<Polygon2d> createOneStepPolygons(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::Pose & current_ego_pose, const double lateral_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double additional_front_outer_wheel_off_track_scale)
{
  const auto error_poses = enable_to_consider_current_pose
    ? calculateErrorPosesImpl(trajectory, current_ego_pose, time_to_convergence) : std::vector<geometry_msgs::Pose>{};
  const auto off_tracking = additional_front_outer_wheel_off_track_scale > 0.0
    ? calcFrontOuterWheelOffTracking(trajectory, vehicle_info) : std::vector<double>(trajectory.size(), 0.0);
  std::vector<Polygon2d> output;
  Polygon2d previous;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const double left_margin = lateral_margin + additional_front_outer_wheel_off_track_scale * std::max(off_tracking.at(i), 0.0);
    const double right_margin = lateral_margin + additional_front_outer_wheel_off_track_scale * std::max(-off_tracking.at(i), 0.0);
    std::vector<geometry_msgs::Pose> poses{trajectory.at(i).pose};
    if (i < error_poses.size()) poses.push_back(error_poses.at(i));
    Polygon2d indexed;
    for (const auto & pose : poses) {
      if (i == 0 && trajectory.at(i).longitudinal_velocity_mps > 1e-3F) {
        const double half_width = vehicle_info.vehicle_width_m / 2.0;
        const auto point0 = autoware_utils_geometry::calc_offset_pose(
          pose, vehicle_info.max_longitudinal_offset_m, half_width + left_margin, 0.0).position;
        const auto point1 = autoware_utils_geometry::calc_offset_pose(
          pose, vehicle_info.max_longitudinal_offset_m, -half_width - right_margin, 0.0).position;
        const auto point2 = autoware_utils_geometry::calc_offset_pose(
          pose, -vehicle_info.rear_overhang_m, -half_width, 0.0).position;
        const auto point3 = autoware_utils_geometry::calc_offset_pose(
          pose, -vehicle_info.rear_overhang_m, half_width, 0.0).position;
        indexed.outer().insert(indexed.outer().end(), {point2d(point0), point2d(point1), point2d(point2), point2d(point3), point2d(point0)});
      } else {
        const auto footprint = createPoseFootprint(pose, vehicle_info, left_margin, right_margin);
        indexed.outer().insert(indexed.outer().end(), footprint.outer().begin(), footprint.outer().end());
      }
    }
    Polygon2d merge_input = previous;
    merge_input.outer().insert(merge_input.outer().end(), indexed.outer().begin(), indexed.outer().end());
    Polygon2d hull;
    boost::geometry::convex_hull(merge_input, hull);
    boost::geometry::correct(hull);
    output.push_back(hull);
    previous = indexed;
  }
  return output;
}

std::vector<PointWithStamp> getCollisionPoints(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const std::vector<Polygon2d> & trajectory_polygons, const ros::Time & obstacle_stamp,
  const autoware_perception_msgs::PredictedPath & predicted_path,
  const autoware_perception_msgs::Shape & shape, const ros::Time & current_time,
  const bool is_driving_forward, std::vector<size_t> & collision_index,
  const double max_dist, const double max_prediction_time_for_collision_check)
{
  if (trajectory.size() != trajectory_polygons.size()) return {};

  std::vector<PointWithStamp> output;
  const double time_step = static_cast<double>(predicted_path.time_step.sec) +
                           static_cast<double>(predicted_path.time_step.nanosec) * 1e-9;
  for (size_t path_index = 0; path_index < predicted_path.path.size(); ++path_index) {
    const double predicted_time = time_step * static_cast<double>(path_index);
    if (predicted_time > max_prediction_time_for_collision_check) break;
    const ros::Time object_time = obstacle_stamp + ros::Duration(predicted_time);
    if ((object_time - current_time).toSec() < 0.0) continue;

    const auto object_polygon = autoware_utils_geometry::to_polygon2d(
      predicted_path.path.at(path_index), shape);
    for (size_t trajectory_index = 0; trajectory_index < trajectory_polygons.size(); ++trajectory_index) {
      const auto & trajectory_point = trajectory.at(trajectory_index);
      const double approximate_distance = autoware_utils_geometry::calc_distance2d(
        trajectory_point.pose.position, predicted_path.path.at(path_index).position);
      if (approximate_distance > max_dist) continue;

      std::vector<Polygon2d> intersections;
      boost::geometry::intersection(
        trajectory_polygons.at(trajectory_index), object_polygon, intersections);
      std::vector<PointWithStamp> candidates;
      for (const auto & intersection : intersections) {
        if (boost::geometry::area(intersection) <= 0.0) continue;
        for (const auto & point : intersection.outer()) {
          PointWithStamp candidate;
          candidate.stamp = object_time;
          candidate.point.x = point.x();
          candidate.point.y = point.y();
          candidate.point.z = trajectory_point.pose.position.z;
          candidates.push_back(candidate);
        }
      }
      if (candidates.empty()) continue;

      const size_t previous_index = trajectory_index == 0 ? 0 : trajectory_index - 1;
      const size_t next_index = trajectory_index == 0 ? std::min<size_t>(1, trajectory.size() - 1) : trajectory_index;
      std::vector<geometry_msgs::Pose> segment{
        trajectory.at(previous_index).pose, trajectory.at(next_index).pose};
      if (!is_driving_forward) std::reverse(segment.begin(), segment.end());
      const auto nearest = std::min_element(
        candidates.begin(), candidates.end(), [&](const auto & lhs, const auto & rhs) {
          return autoware::motion_utils::calcLongitudinalOffsetToSegment(segment, 0, lhs.point) <
                 autoware::motion_utils::calcLongitudinalOffsetToSegment(segment, 0, rhs.point);
        });
      output.push_back(*nearest);
      collision_index.push_back(trajectory_index);
      break;
    }
  }
  return output;
}
}
