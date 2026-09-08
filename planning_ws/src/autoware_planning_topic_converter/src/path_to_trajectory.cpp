// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/planning_topic_converter/path_to_trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <vector>

namespace autoware
{
namespace planning_topic_converter
{
namespace
{
autoware_planning_msgs::TrajectoryPoint convert_to_trajectory_point(
  const autoware_planning_msgs::PathPoint & point)
{
  autoware_planning_msgs::TrajectoryPoint trajectory_point;
  trajectory_point.pose = autoware_utils_geometry::get_pose(point);
  trajectory_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  trajectory_point.lateral_velocity_mps = point.lateral_velocity_mps;
  trajectory_point.heading_rate_rps = point.heading_rate_rps;
  return trajectory_point;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> convert_to_trajectory_points(
  const std::vector<autoware_planning_msgs::PathPoint> & points)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory_points;
  trajectory_points.reserve(points.size());
  for (std::vector<autoware_planning_msgs::PathPoint>::const_iterator point = points.begin();
       point != points.end(); ++point) {
    trajectory_points.push_back(convert_to_trajectory_point(*point));
  }
  return trajectory_points;
}
}  // namespace

PathToTrajectory::PathToTrajectory(ros::NodeHandle & node, ros::NodeHandle & private_node)
: ConverterBase(node, private_node, "path_to_trajectory_converter")
{
}

void PathToTrajectory::process(const autoware_planning_msgs::Path::ConstPtr & msg)
{
  const std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory_points =
    convert_to_trajectory_points(msg->points);
  const autoware_planning_msgs::Trajectory output =
    autoware::motion_utils::convertToTrajectory(trajectory_points, msg->header);
  pub_.publish(output);
}

}  // namespace planning_topic_converter
}  // namespace autoware
