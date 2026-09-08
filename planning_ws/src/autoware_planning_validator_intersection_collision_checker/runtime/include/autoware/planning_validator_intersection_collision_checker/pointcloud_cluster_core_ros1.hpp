#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
struct ClusterParameters
{
  double tolerance{};
  double min_height{};
  int min_size{};
  int max_size{};
};

// ROS1 rendering of cluster_pointcloud: Euclidean clusters are retained only
// if one point is above ego-base-relative min_height, then their 2D hull is
// emitted as the source does.
pcl::PointCloud<pcl::PointXYZ> cluster_points_above_height(
  const pcl::PointCloud<pcl::PointXYZ> & input, double ego_base_z,
  const ClusterParameters & parameters);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
