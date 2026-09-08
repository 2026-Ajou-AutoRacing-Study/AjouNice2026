#pragma once

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
struct ArcDistanceCandidate
{
  pcl::PointXYZ point;
  double distance_to_overlap{};
};

// Rendering of get_pcd_object's inner arc-distance choice.  Input points must
// already be inside the target-lane polygon and pass clustering, exactly as in
// the source call order.  A point at/past the overlap is not an approaching
// object and is therefore excluded.
std::optional<ArcDistanceCandidate> nearest_approaching_point(
  const pcl::PointCloud<pcl::PointXYZ> & clustered_points,
  const lanelet::ConstLanelets & target_lanelets, const geometry_msgs::Pose & overlap_pose);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
