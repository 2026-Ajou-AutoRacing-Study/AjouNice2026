#include "autoware/planning_validator_intersection_collision_checker/point_in_lane_core_ros1.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
pcl::PointCloud<pcl::PointXYZ> points_within_lanelet(
  const pcl::PointCloud<pcl::PointXYZ> & input, const lanelet::ConstLanelet & lanelet)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  for (const auto & point : input) {
    if (lanelet::geometry::inside(lanelet, lanelet::BasicPoint2d{point.x, point.y})) output.push_back(point);
  }
  return output;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
