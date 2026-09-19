#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_TEST_UTILS_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_TEST_UTILS_HPP_

#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>

namespace test
{
inline autoware_internal_planning_msgs::PathWithLaneId generatePath(
  double x0, double y0, double x, double y, int count)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  if (count < 2) return path;
  for (int i = 0; i < count; ++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x0 + (x - x0) * i / (count - 1);
    point.point.pose.position.y = y0 + (y - y0) * i / (count - 1);
    path.points.push_back(point);
  }
  return path;
}
inline geometry_msgs::Pose generatePose(double x) { geometry_msgs::Pose pose; pose.position.x = x; pose.position.z = 1.0; pose.orientation.w = 1.0; return pose; }
}  // namespace test
#endif
