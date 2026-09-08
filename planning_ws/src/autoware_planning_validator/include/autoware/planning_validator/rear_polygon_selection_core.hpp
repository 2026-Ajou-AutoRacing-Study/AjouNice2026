#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_POLYGON_SELECTION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_POLYGON_SELECTION_CORE_HPP_

#include "autoware/planning_validator/rear_pointcloud_core.hpp"

#include <vector>

namespace autoware::planning_validator
{
// A ROS1-neutral copy of a lanelet detection polygon. The adapter must retain
// source lanelet vertex order and z values when converting to this boundary.
using RearDetectionPolygon = std::vector<pcl::PointXYZ>;

// Exact two-stage source selection from utils::get_obstacle_points: first
// reject by the minimum enclosing circle, then use strict Boost within on the
// detection polygon. Boundary points are deliberately excluded.
RearPointCloud::Ptr select_rear_points_in_detection_polygons(
  const std::vector<RearDetectionPolygon> & polygons, const RearPointCloud & points);
}  // namespace autoware::planning_validator

#endif
