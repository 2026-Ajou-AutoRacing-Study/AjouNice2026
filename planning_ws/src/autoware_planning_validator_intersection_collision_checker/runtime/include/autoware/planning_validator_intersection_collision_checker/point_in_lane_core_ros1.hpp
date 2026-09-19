#pragma once

#include <lanelet2_core/primitives/Lanelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
pcl::PointCloud<pcl::PointXYZ> points_within_lanelet(
  const pcl::PointCloud<pcl::PointXYZ> & input, const lanelet::ConstLanelet & lanelet);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
