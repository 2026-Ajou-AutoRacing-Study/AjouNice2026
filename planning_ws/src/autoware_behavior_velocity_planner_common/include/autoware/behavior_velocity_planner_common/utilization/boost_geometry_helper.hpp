// Copyright 2020 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0

#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_BOOST_GEOMETRY_HELPER_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_BOOST_GEOMETRY_HELPER_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <vector>

// These registrations are the ROS1 equivalents of the frozen ROS2 helpers;
// the field layouts are unchanged by the generated-message boundary.
BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::Point, double, cs::cartesian, x, y, z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::Pose, double, cs::cartesian, position.x, position.y, position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::PoseWithCovarianceStamped, double, cs::cartesian, pose.pose.position.x,
  pose.pose.position.y, pose.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_planning_msgs::PathPoint, double, cs::cartesian, pose.position.x, pose.position.y,
  pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_internal_planning_msgs::PathPointWithLaneId, double, cs::cartesian,
  point.pose.position.x, point.pose.position.y, point.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_planning_msgs::TrajectoryPoint, double, cs::cartesian, pose.position.x,
  pose.position.y, pose.position.z)

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

using Point2d = autoware_utils_geometry::Point2d;
using LineString2d = autoware_utils_geometry::LineString2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;

template <class T>
Point2d to_bg2d(const T & point)
{
  return Point2d(bg::get<0>(point), bg::get<1>(point));
}

template <class T>
LineString2d to_bg2d(const std::vector<T> & points)
{
  LineString2d line;
  for (const auto & point : points) line.push_back(to_bg2d(point));
  return line;
}

Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line);
Polygon2d upScalePolygon(
  const geometry_msgs::Point & position, const Polygon2d & polygon, double scale);
geometry_msgs::Polygon toGeomPoly(const Polygon2d & polygon);
}  // namespace autoware::behavior_velocity_planner

#endif
