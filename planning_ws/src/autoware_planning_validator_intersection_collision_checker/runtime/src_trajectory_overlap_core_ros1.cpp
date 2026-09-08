#include "autoware/planning_validator_intersection_collision_checker/trajectory_overlap_core_ros1.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <limits>

namespace autoware::planning_validator::intersection_collision_checker
{
namespace
{
double seconds(const builtin_interfaces::Duration & value)
{
  return static_cast<double>(value.sec) + static_cast<double>(value.nanosec) * 1e-9;
}

std::size_t nearest_index(
  const autoware_planning_msgs::Trajectory & trajectory,
  const autoware_utils_geometry::Point2d & point)
{
  std::size_t result = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    const autoware_utils_geometry::Point2d candidate(
      trajectory.points[index].pose.position.x, trajectory.points[index].pose.position.y);
    const double distance = boost::geometry::distance(candidate, point);
    if (distance < best_distance) {
      best_distance = distance;
      result = index;
    }
  }
  return result;
}
}  // namespace

std::optional<TrajectoryOverlap> trajectory_overlap_for_lanelet(
  const lanelet::ConstLanelet & lanelet, const autoware_planning_msgs::Trajectory & trajectory,
  const std::size_t ego_front_index)
{
  if (trajectory.points.size() < 2U || ego_front_index >= trajectory.points.size()) {
    return std::nullopt;
  }
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : lanelet.polygon2d()) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  boost::geometry::correct(polygon);
  autoware_utils_geometry::LineString2d trajectory_line;
  for (const auto & point : trajectory.points) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }
  autoware_utils_geometry::MultiLineString2d overlap_lines;
  boost::geometry::intersection(polygon, trajectory_line, overlap_lines);
  if (overlap_lines.empty() || overlap_lines.front().empty()) return std::nullopt;

  auto first_index = nearest_index(trajectory, overlap_lines.front().front());
  auto last_index = nearest_index(trajectory, overlap_lines.front().back());
  if (last_index < first_index) std::swap(first_index, last_index);
  if (first_index < ego_front_index) return std::nullopt;
  return TrajectoryOverlap{
    first_index, last_index, seconds(trajectory.points[first_index].time_from_start),
    seconds(trajectory.points[last_index].time_from_start)};
}
}  // namespace autoware::planning_validator::intersection_collision_checker
