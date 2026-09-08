#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_planning_msgs/PathPoint.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace autoware::behavior_path_planner::utils
{
std::vector<double> calcPathArcLengthArray(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const size_t start,
  const size_t end, const double offset)
{
  const size_t bounded_end = std::min(end, path.points.size());
  if (start >= bounded_end) return {};
  std::vector<double> result{offset};
  for (size_t index = start + 1; index < bounded_end; ++index) {
    const auto & first = path.points.at(index - 1).point.pose.position;
    const auto & second = path.points.at(index).point.pose.position;
    result.push_back(result.back() + std::hypot(second.x - first.x, second.y - first.y));
  }
  return result;
}

autoware_internal_planning_msgs::PathWithLaneId resamplePathWithSpline(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const double interval,
  const bool keep_input_points, const std::pair<double, double> target_section)
{
  if (path.points.size() < 2 || interval <= 0.0) return path;
  std::vector<autoware_planning_msgs::PathPoint> transformed(path.points.size());
  for (size_t index = 0; index < path.points.size(); ++index) transformed.at(index) = path.points.at(index).point;
  const auto arc = autoware::motion_utils::calcSignedArcLengthPartialSum(transformed, 0, transformed.size());
  // NaN arc length makes std::min(max_double, arc.back()) select max_double,
  // creating an effectively unbounded sampling loop. Reject invalid geometry.
  if (!std::isfinite(interval) ||
      !std::all_of(arc.begin(), arc.end(), [](double s) { return std::isfinite(s); })) {
    throw std::invalid_argument("Cannot resample non-finite path geometry or interval");
  }
  const auto near_values = [](const std::vector<double> & values, const double value) {
    std::vector<size_t> indices;
    for (size_t index = 0; index < values.size(); ++index)
      if (std::abs(values.at(index) - value) < 0.2) indices.push_back(index);
    return indices;
  };
  std::vector<double> samples;
  std::unordered_set<int64_t> seen_lane_ids;
  for (size_t index = 0; index < path.points.size(); ++index) {
    for (const auto lane_id : path.points.at(index).lane_ids) {
      if (!keep_input_points && !seen_lane_ids.insert(lane_id).second) continue;
      if (near_values(samples, arc.at(index)).empty()) samples.push_back(arc.at(index));
    }
  }
  const double start = std::max(target_section.first, 0.0);
  const double end = std::min(target_section.second, arc.back());
  for (double s = start; s < end; s += interval)
    if (near_values(samples, s).empty()) samples.push_back(s);
  if (near_values(samples, end).empty()) samples.push_back(end);
  const auto stop_distance = autoware::motion_utils::calcDistanceToForwardStopPoint(transformed);
  if (stop_distance) {
    const auto duplicates = near_values(samples, *stop_distance);
    if (duplicates.empty()) samples.push_back(*stop_distance);
    else {
      samples.at(duplicates.front()) = *stop_distance;
      for (auto it = duplicates.rbegin(); it != duplicates.rend(); ++it)
        if (*it != duplicates.front()) samples.erase(samples.begin() + *it);
    }
  }
  if (samples.size() < 2) return path;
  std::sort(samples.begin(), samples.end());
  return autoware::motion_utils::resamplePath(path, samples);
}

size_t getIdxByArclength(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const size_t target_idx,
  const double signed_arc)
{
  if (path.points.empty() || target_idx >= path.points.size()) throw std::runtime_error("path points must be nonempty");
  double sum{};
  const auto distance = [](const auto & first, const auto & second) {
    const auto & a = first.point.pose.position;
    const auto & b = second.point.pose.position;
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  if (signed_arc >= 0.0) {
    for (size_t index = target_idx; index + 1 < path.points.size(); ++index) {
      sum += distance(path.points.at(index), path.points.at(index + 1));
      if (sum > signed_arc) return index + 1;
    }
    return path.points.size() - 1;
  }
  for (size_t index = target_idx; index > 0; --index) {
    sum -= distance(path.points.at(index), path.points.at(index - 1));
    if (sum < signed_arc) return index - 1;
  }
  return 0;
}

void clipPathLength(
  autoware_internal_planning_msgs::PathWithLaneId & path, const size_t target_idx,
  const double forward, const double backward)
{
  if (path.points.size() < 3 || target_idx >= path.points.size()) return;
  const size_t start = getIdxByArclength(path, target_idx, -backward);
  const size_t end = getIdxByArclength(path, target_idx, forward);
  path.points = {path.points.begin() + start, path.points.begin() + end + 1};
}

std::vector<size_t> getReversingIndices(const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  std::vector<size_t> indices;
  for (size_t index = 0; index + 1 < path.points.size(); ++index)
    if (path.points.at(index).point.longitudinal_velocity_mps * path.points.at(index + 1).point.longitudinal_velocity_mps < 0.0)
      indices.push_back(index);
  return indices;
}

PathWithLaneId convertWayPointsToPathWithLaneId(
  const autoware::freespace_planning_algorithms::PlannerWaypoints & waypoints,
  const double velocity, const lanelet::ConstLanelets & lanelets)
{
  PathWithLaneId path;
  path.header = waypoints.header;
  for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
    const auto & waypoint = waypoints.waypoints.at(i);
    PathPointWithLaneId point{};
    point.point.pose = waypoint.pose.pose;
    for (const auto & lane : lanelets) {
      if (autoware::experimental::lanelet2_utils::is_in_lanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
      }
    }
    if (point.lane_ids.empty() && i > 0) point.lane_ids = path.points.at(i - 1).lane_ids;
    point.point.longitudinal_velocity_mps = (waypoint.is_back ? -1.0 : 1.0) * velocity;
    path.points.push_back(point);
  }
  return path;
}

std::vector<autoware_internal_planning_msgs::PathWithLaneId> dividePath(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const std::vector<size_t> & indices)
{
  if (indices.empty()) return {path};
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> output;
  size_t start{};
  for (const auto end : indices) {
    if (end >= path.points.size() || end < start) throw std::runtime_error("invalid reversal index");
    auto partial = path;
    partial.points = {path.points.begin() + start, path.points.begin() + end + 1};
    output.push_back(partial);
    start = end;
  }
  auto partial = path;
  partial.points = {path.points.begin() + start, path.points.end()};
  output.push_back(partial);
  return output;
}

void correctDividedPathVelocity(std::vector<autoware_internal_planning_msgs::PathWithLaneId> & paths)
{
  for (auto & path : paths) {
    if (path.points.empty()) continue;
    const auto forward = autoware::motion_utils::isDrivingForward(path.points);
    if (!forward) continue;
    for (auto & point : path.points) point.point.longitudinal_velocity_mps = *forward ?
      std::abs(point.point.longitudinal_velocity_mps) : -std::abs(point.point.longitudinal_velocity_mps);
    path.points.back().point.longitudinal_velocity_mps = 0.0;
  }
}

std::vector<double> spline_two_points(
  const std::vector<double> & base_s, const std::vector<double> & base_x, const double begin_diff,
  const double end_diff, const std::vector<double> & new_s)
{
  assert(base_s.size() == 2 && base_x.size() == 2);
  const double h = base_s.at(1) - base_s.at(0), c = begin_diff, d = base_x.at(0);
  const double a = (end_diff * h - 2.0 * base_x.at(1) + c * h + 2.0 * d) / std::pow(h, 3);
  const double b = (3.0 * base_x.at(1) - end_diff * h - 2.0 * c * h - 3.0 * d) / std::pow(h, 2);
  std::vector<double> output;
  for (const double s : new_s) { const double ds = s - base_s.at(0); output.push_back(d + (c + (b + a * ds) * ds) * ds); }
  return output;
}

std::vector<geometry_msgs::Pose> interpolatePose(
  const geometry_msgs::Pose & start, const geometry_msgs::Pose & end, const double interval)
{
  const double distance = std::hypot(end.position.x - start.position.x, end.position.y - start.position.y);
  if (interval <= 0.0 || distance <= 0.6) return {};
  std::vector<double> samples;
  for (double s = 0.3; s < distance - 0.3; s += interval) samples.push_back(s);
  const std::vector<double> base_s{0.0, distance};
  const auto x = spline_two_points(base_s, {start.position.x, end.position.x}, std::cos(tf2::getYaw(start.orientation)), std::cos(tf2::getYaw(end.orientation)), samples);
  const auto y = spline_two_points(base_s, {start.position.y, end.position.y}, std::sin(tf2::getYaw(start.orientation)), std::sin(tf2::getYaw(end.orientation)), samples);
  std::vector<geometry_msgs::Pose> poses;
  for (size_t index = 0; index < x.size(); ++index) {
    geometry_msgs::Pose pose; pose.position.x = x.at(index); pose.position.y = y.at(index); pose.position.z = end.position.z;
    poses.push_back(pose);
  }
  for (size_t index = 0; index < poses.size(); ++index) {
    const auto & next = index + 1 < poses.size() ? poses.at(index + 1).position : end.position;
    const double yaw = std::atan2(next.y - poses.at(index).position.y, next.x - poses.at(index).position.x);
    tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw); poses.at(index).orientation = tf2::toMsg(q);
  }
  return poses;
}

geometry_msgs::Pose getUnshiftedEgoPose(
  const geometry_msgs::Pose & ego_pose,
  const autoware::behavior_path_planner::ShiftedPath & previous_path)
{
  if (previous_path.path.points.empty() || previous_path.shift_length.size() != previous_path.path.points.size()) {
    return ego_pose;
  }
  const size_t closest = autoware::motion_utils::findNearestIndex(previous_path.path.points, ego_pose.position);
  auto pose = autoware::motion_utils::calcInterpolatedPoint(previous_path.path, ego_pose).point.pose;
  pose = autoware_utils_geometry::calc_offset_pose(pose, 0.0, -previous_path.shift_length.at(closest), 0.0);
  pose.orientation = ego_pose.orientation;
  return pose;
}

autoware_internal_planning_msgs::PathWithLaneId calcCenterLinePath(
  const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> & planner_data,
  const geometry_msgs::Pose & reference_pose, const double longest_distance_to_shift_line,
  const std::optional<autoware_internal_planning_msgs::PathWithLaneId> & previous_module_path)
{
  if (!planner_data || !planner_data->route_handler || !planner_data->route_handler->isMapMsgReady() ||
      !planner_data->route_handler->isHandlerReady()) return {};
  const auto backward = std::max(
    planner_data->parameters.backward_path_length, longest_distance_to_shift_line + 10.0);
  const auto lanes = previous_module_path
    ? getCurrentLanesFromPath(*previous_module_path, planner_data)
    : calcLaneAroundPose(
        planner_data->route_handler, reference_pose, planner_data->parameters.forward_path_length,
        backward);
  auto path = getCenterLinePath(
    *planner_data->route_handler, lanes, reference_pose, backward,
    planner_data->parameters.forward_path_length, planner_data->parameters);
  path.header = planner_data->route_handler->getRouteHeader();
  return path;
}

autoware_internal_planning_msgs::PathWithLaneId combinePath(
  const autoware_internal_planning_msgs::PathWithLaneId & path1,
  const autoware_internal_planning_msgs::PathWithLaneId & path2)
{
  if (path1.points.empty()) return path2;
  if (path2.points.empty()) return path1;
  auto output = path1;
  output.points.insert(output.points.end(), std::next(path2.points.begin()), path2.points.end());
  output.points = autoware::motion_utils::removeOverlapPoints(output.points);
  return output;
}
}  // namespace autoware::behavior_path_planner::utils
