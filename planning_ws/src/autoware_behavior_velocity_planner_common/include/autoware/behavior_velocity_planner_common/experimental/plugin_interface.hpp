#pragma once

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::PathPointWithLaneId>;

// ROS1 counterpart of the ROS2 experimental plugin boundary.  It remains
// distinct from the legacy PathWithLaneId plugin ABI: callers own a validated
// trajectory object and pass planner data explicitly for every invocation.
class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(ros::NodeHandle & node) = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() = 0;
  virtual void plan(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data) = 0;
  virtual void updateSceneModuleInstances(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) = 0;
  virtual const char * getModuleName() = 0;
};
}  // namespace autoware::behavior_velocity_planner::experimental
