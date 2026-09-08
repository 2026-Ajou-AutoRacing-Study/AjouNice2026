#pragma once

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <ros/ros.h>

#include <memory>

namespace autoware::behavior_velocity_planner
{

// ROS1 equivalent of the ROS2 planner plugin boundary.  NodeHandle replaces
// rclcpp::Node while preserving the plugin lifecycle and mutable path ABI.
class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(ros::NodeHandle & node) = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() = 0;
  virtual void plan(autoware_internal_planning_msgs::PathWithLaneId * path) = 0;
  virtual void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_internal_planning_msgs::PathWithLaneId & path) = 0;
  virtual const char * getModuleName() = 0;
};

}  // namespace autoware::behavior_velocity_planner
