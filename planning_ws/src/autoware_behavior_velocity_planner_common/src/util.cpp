#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::behavior_velocity_planner::planning_utils
{
bool isAheadOf(const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin)
{
  const double dx = target.position.x - origin.position.x;
  const double dy = target.position.y - origin.position.y;
  const double yaw = tf2::getYaw(origin.orientation);
  return std::cos(yaw) * dx + std::sin(yaw) * dy > 0.0;
}

double calcJudgeLineDistWithAccLimit(const double velocity, const double max_stop_acceleration, const double delay_response_time)
{
  return velocity * velocity / (2.0 * -max_stop_acceleration) + delay_response_time * velocity;
}

double calcJudgeLineDistWithJerkLimit(const double velocity, const double acceleration, const double max_stop_acceleration, const double max_stop_jerk, const double delay_response_time)
{
  if (velocity <= 0.0) return 0.0;
  const double x1 = velocity * delay_response_time;
  const double v2 = velocity + (max_stop_acceleration * max_stop_acceleration - acceleration * acceleration) / (2.0 * max_stop_jerk);
  if (v2 <= 0.0) {
    const double t2 = -(max_stop_acceleration + std::sqrt(acceleration * acceleration - 2.0 * max_stop_jerk * velocity)) / max_stop_jerk;
    return std::max(0.0, x1 + velocity * t2 + acceleration * t2 * t2 / 2.0 + max_stop_jerk * t2 * t2 * t2 / 6.0);
  }
  const double t2 = (max_stop_acceleration - acceleration) / max_stop_jerk;
  const double x2 = velocity * t2 + acceleration * t2 * t2 / 2.0 + max_stop_jerk * t2 * t2 * t2 / 6.0;
  const double x3 = -v2 * v2 / (2.0 * max_stop_acceleration);
  return std::max(0.0, x1 + x2 + x3);
}

namespace
{
double findReachTime(const double jerk, const double accel, const double velocity, const double distance, double lower, double upper)
{
  const auto f = [&](const double time) { return jerk * time * time * time / 6.0 + accel * time * time / 2.0 + velocity * time - distance; };
  if (f(lower) > 0.0 || f(upper) < 0.0) throw std::logic_error("invalid reach-time range");
  for (int i = 0; i != 200; ++i) {
    const double mid = 0.5 * (lower + upper);
    if (std::abs(f(mid)) < 1e-5) return mid;
    if (f(mid) > 0.0) upper = mid; else lower = mid;
  }
  return 0.5 * (lower + upper);
}
}

double calcDecelerationVelocityFromDistanceToTarget(const double jerk, const double max_accel, const double current_accel, const double current_velocity, const double distance)
{
  if (jerk > 0.0 || max_accel > 0.0) throw std::logic_error("slowdown jerk and accel must be negative");
  if (distance <= 0.0) return current_velocity;
  const auto displacement = [](double t, double j, double a, double v) { return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t; };
  const auto velocity = [](double t, double j, double a, double v) { return j * t * t / 2.0 + a * t + v; };
  const double t_jerk = (max_accel - current_accel) / jerk;
  const double remaining = distance - displacement(t_jerk, jerk, current_accel, current_velocity);
  if (remaining < 0.0) return velocity(findReachTime(jerk, current_accel, current_velocity, distance, 0.0, t_jerk), jerk, current_accel, current_velocity);
  const double v1 = velocity(t_jerk, jerk, current_accel, current_velocity);
  const double discriminant = 2.0 * max_accel * remaining + v1 * v1;
  if (discriminant <= 0.0) return 0.0;
  const double t_acc = (-v1 + std::sqrt(discriminant)) / max_accel;
  return velocity(t_acc, 0.0, max_accel, v1);
}

std::optional<geometry_msgs::Pose> insertDecelPoint(
  const geometry_msgs::Point & stop_point, autoware_internal_planning_msgs::PathWithLaneId & output,
  const float target_velocity)
{
  const auto base = autoware::motion_utils::findNearestSegmentIndex(output.points, stop_point);
  const auto inserted = autoware::motion_utils::insertTargetPoint(base, stop_point, output.points, 5e-2);
  if (!inserted) return std::nullopt;
  for (size_t i = *inserted; i < output.points.size(); ++i) {
    output.points[i].point.longitudinal_velocity_mps = std::min(
      output.points[i].point.longitudinal_velocity_mps, target_velocity);
  }
  return output.points[*inserted].point.pose;
}

std::optional<geometry_msgs::Pose> insertStopPoint(
  const geometry_msgs::Point & stop_point, autoware_internal_planning_msgs::PathWithLaneId & output)
{
  const auto base = autoware::motion_utils::findNearestSegmentIndex(output.points, stop_point);
  const auto inserted = autoware::motion_utils::insertStopPoint(base, stop_point, output.points);
  return inserted ? std::optional<geometry_msgs::Pose>(output.points[*inserted].point.pose) : std::nullopt;
}

std::optional<geometry_msgs::Pose> insertStopPoint(
  const geometry_msgs::Point & stop_point, const size_t stop_seg_idx,
  autoware_internal_planning_msgs::PathWithLaneId & output)
{
  const auto inserted = autoware::motion_utils::insertStopPoint(stop_seg_idx, stop_point, output.points);
  return inserted ? std::optional<geometry_msgs::Pose>(output.points[*inserted].point.pose) : std::nullopt;
}

geometry_msgs::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  if (path.points.empty() || start_idx >= path.points.size()) return {};
  double accumulated = 0.0;
  for (size_t i = start_idx; i + 1 < path.points.size(); ++i) {
    const auto & a = path.points[i].point.pose;
    const auto & b = path.points[i + 1].point.pose;
    const auto length = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
    if (accumulated + length > ahead_dist && length > 0.0) {
      const auto ratio = (ahead_dist - accumulated) / length;
      geometry_msgs::Pose result = a;
      result.position.x += ratio * (b.position.x - a.position.x);
      result.position.y += ratio * (b.position.y - a.position.y);
      result.position.z += ratio * (b.position.z - a.position.z);
      tf2::Quaternion q0;
      tf2::Quaternion q1;
      tf2::fromMsg(a.orientation, q0);
      tf2::fromMsg(b.orientation, q1);
      result.orientation = tf2::toMsg(q0.slerp(q1, ratio));
      return result;
    }
    accumulated += length;
  }
  return path.points.back().point.pose;
}

autoware_utils_geometry::Point2d calculateOffsetPoint2d(
  const geometry_msgs::Pose & pose, const double offset_x, const double offset_y)
{
  const auto result = autoware_utils_geometry::calc_offset_pose(pose, offset_x, offset_y, 0.0);
  return {result.position.x, result.position.y};
}

size_t calcSegmentIndexFromPointIndex(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
  const geometry_msgs::Point & point, const size_t idx)
{
  if (points.size() < 3 || idx == 0) return 0;
  if (idx >= points.size() - 1) return points.size() - 2;
  const auto & a = points[idx].point.pose.position;
  const auto & b = points[idx + 1].point.pose.position;
  const auto projection = (point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y);
  return projection > 0.0 ? idx : idx - 1;
}

std::vector<int64_t> getSortedLaneIdsFromPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  std::vector<int64_t> sorted_lane_ids;
  for (const auto & path_point : path.points) {
    for (const auto lane_id : path_point.lane_ids) {
      if (std::find(sorted_lane_ids.begin(), sorted_lane_ids.end(), lane_id) ==
          sorted_lane_ids.end()) {
        sorted_lane_ids.emplace_back(lane_id);
      }
    }
  }
  return sorted_lane_ids;
}

std::vector<int64_t> getSubsequentLaneIdsSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const int64_t base_lane_id)
{
  const auto all_lane_ids = getSortedLaneIdsFromPath(path);
  const auto base_index = std::find(all_lane_ids.begin(), all_lane_ids.end(), base_lane_id);
  if (base_index == all_lane_ids.end()) return {};
  return {base_index, all_lane_ids.end()};
}

std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose)
{
  if (!lanelet_map) return {};
  const auto nearest_lane_id = getNearestLaneId(path, lanelet_map, current_pose);
  const auto lane_ids = nearest_lane_id ? getSubsequentLaneIdsSetOnPath(path, *nearest_lane_id)
                                        : getSortedLaneIdsFromPath(path);
  std::vector<lanelet::ConstLanelet> lanelets;
  lanelets.reserve(lane_ids.size());
  for (const auto lane_id : lane_ids) {
    if (lanelet_map->laneletLayer.exists(lane_id)) lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
  }
  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose)
{
  std::set<int64_t> lane_id_set;
  for (const auto & lane : getLaneletsOnPath(path, lanelet_map, current_pose)) lane_id_set.insert(lane.id());
  return lane_id_set;
}

autoware_utils_geometry::LineString2d extendSegmentToBounds(
  const lanelet::BasicLineString2d & segment, const std::vector<geometry_msgs::Point> & bound1,
  const std::vector<geometry_msgs::Point> & bound2)
{
  using autoware_utils_geometry::LineString2d;
  using autoware_utils_geometry::Point2d;
  LineString2d original;
  for (const auto & point : segment) original.push_back(Point2d(point.x(), point.y()));
  if (segment.size() != 2 || bound1.size() < 2 || bound2.size() < 2) return original;
  constexpr double epsilon = 1e-6;
  const auto intersection = [&](const std::vector<geometry_msgs::Point> & bound)
    -> std::optional<Point2d> {
    std::optional<Point2d> found;
    double closest = std::numeric_limits<double>::max();
    for (auto it = std::next(bound.begin()); it != bound.end(); ++it) {
      const Eigen::Vector2d p2{std::prev(it)->x, std::prev(it)->y};
      const Eigen::Vector2d p3{it->x, it->y};
      const Eigen::Vector2d v01 = segment[1] - segment[0];
      const Eigen::Vector2d v23 = p3 - p2;
      const double cross = v23.x() * v01.y() - v23.y() * v01.x();
      if (std::abs(cross) < epsilon) continue;
      const Eigen::Vector2d v20 = segment[0] - p2;
      const double t = (v20.x() * v01.y() - v20.y() * v01.x()) / cross;
      if (t < 0.0 || t > 1.0) continue;
      const Eigen::Vector2d point = p2 + t * v23;
      const double distance = std::min(
        (point - segment[0]).norm(), (point - segment[1]).norm());
      if (!found || distance <= closest) { found = Point2d(point.x(), point.y()); closest = distance; }
    }
    return found;
  };
  const auto first = intersection(bound1);
  const auto second = intersection(bound2);
  if (!first || !second) return original;
  const LineString2d extended = ((*second - *first).dot(segment[1] - segment[0]) < 0.0)
    ? LineString2d{*second, *first} : LineString2d{*first, *second};
  const auto on_line = [](const LineString2d & line, const Point2d & point) {
    const Eigen::Vector2d a{line[0].x(), line[0].y()};
    const Eigen::Vector2d b{line[1].x(), line[1].y()};
    const Eigen::Vector2d p{point.x(), point.y()};
    const Eigen::Vector2d delta = b - a;
    const double length_sq = delta.squaredNorm();
    const double ratio = length_sq > 0.0 ? std::max(0.0, std::min(1.0, (p - a).dot(delta) / length_sq)) : 0.0;
    return (p - (a + ratio * delta)).norm() < 0.1;
  };
  const std::array<Point2d, 2> original_points{Point2d(segment[0].x(), segment[0].y()), Point2d(segment[1].x(), segment[1].y())};
  const std::array<Point2d, 2> extended_points{extended[0], extended[1]};
  if (!std::all_of(original_points.begin(), original_points.end(), [&](const auto & point) { return on_line(extended, point); }) &&
      !std::all_of(extended_points.begin(), extended_points.end(), [&](const auto & point) { return on_line(original, point); })) return original;
  return extended;
}

lanelet::Ids collectConnectedLaneIds(
  const int64_t lane_id, const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  lanelet::Ids ids{lane_id};
  if (!route_handler) return ids;
  const auto map = route_handler->getLaneletMapPtr();
  const auto graph = route_handler->getRoutingGraphPtr();
  if (!map || !graph || !map->laneletLayer.exists(lane_id)) return ids;
  const auto lanelet = map->laneletLayer.get(lane_id);
  for (const auto & next : graph->following(lanelet)) ids.push_back(next.id());
  for (const auto & previous : graph->previous(lanelet)) ids.push_back(previous.id());
  return ids;
}

std::optional<int64_t> getNearestLaneId(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr & lanelet_map, const geometry_msgs::Pose & current_pose)
{
  if (!lanelet_map) return std::nullopt;
  const lanelet::BasicPoint2d ego{current_pose.position.x, current_pose.position.y};
  std::optional<int64_t> nearest;
  double nearest_distance = std::numeric_limits<double>::max();
  for (const auto lane_id : getSortedLaneIdsFromPath(path)) {
    if (!lanelet_map->laneletLayer.exists(lane_id)) continue;
    const double distance = lanelet::geometry::distance2d(lanelet_map->laneletLayer.get(lane_id), ego);
    if (distance < nearest_distance) { nearest_distance = distance; nearest = lane_id; }
  }
  return nearest;
}
}  // namespace autoware::behavior_velocity_planner::planning_utils
