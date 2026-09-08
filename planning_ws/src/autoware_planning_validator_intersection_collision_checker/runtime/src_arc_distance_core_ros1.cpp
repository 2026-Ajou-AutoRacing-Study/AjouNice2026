#include "autoware/planning_validator_intersection_collision_checker/arc_distance_core_ros1.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>

#include <limits>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
std::optional<ArcDistanceCandidate> nearest_approaching_point(
  const pcl::PointCloud<pcl::PointXYZ> & clustered_points,
  const lanelet::ConstLanelets & target_lanelets, const geometry_msgs::Pose & overlap_pose)
{
  if (target_lanelets.empty()) return std::nullopt;
  const auto overlap_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanelets, overlap_pose);
  double min_distance = std::numeric_limits<double>::max();
  std::optional<ArcDistanceCandidate> candidate;
  for (const auto & point : clustered_points) {
    geometry_msgs::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    pose.orientation.w = 1.0;
    const auto arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(target_lanelets, pose);
    const double distance = overlap_arc.length - arc.length;
    if (distance < std::numeric_limits<double>::epsilon() || distance > min_distance) continue;
    min_distance = distance;
    candidate = ArcDistanceCandidate{point, distance};
  }
  return candidate;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
