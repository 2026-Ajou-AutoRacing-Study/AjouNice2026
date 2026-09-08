#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <boost/geometry/index/rtree.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
using autoware_internal_planning_msgs::PathPointWithLaneId;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_perception_msgs::PredictedObjects;
using autoware_planning_msgs::PathPoint;
using geometry_msgs::Point;
using geometry_msgs::Pose;

using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::MultiLineString2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
using autoware_utils_geometry::Segment2d;

// ROS1's staged geometry shim cannot instantiate the source rstar reinsertion
// strategy because that path asks Boost for an unavailable point-distance
// strategy. Quadratic splitting preserves the segment-set API and values.
using SegmentRtree = boost::geometry::index::rtree<Segment2d, boost::geometry::index::quadratic<16>>;

struct PointDistance
{
  Point2d point{};
  double distance{};
};
struct Projection
{
  Point2d projected_point{};
  double distance{};
  double arc_length{};
};
enum Side { LEFT, RIGHT };
struct Expansion
{
  std::vector<double> left_distances;
  std::vector<double> right_distances;
  std::vector<size_t> left_bound_indexes;
  std::vector<PointDistance> left_projections;
  std::vector<size_t> right_bound_indexes;
  std::vector<PointDistance> right_projections;
  std::vector<double> min_lane_widths;
};
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

#endif  // AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
