#ifndef AUTOWARE_PLANNING_VALIDATOR__DEBUG_MARKER_ROS1_HPP_
#define AUTOWARE_PLANNING_VALIDATOR__DEBUG_MARKER_ROS1_HPP_

#include <autoware_planning_validator/PlanningValidatorStatus.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <string>

namespace autoware::planning_validator
{
class DebugMarkerRos1
{
public:
  explicit DebugMarkerRos1(ros::NodeHandle private_nh);
  static std::string status_string(const autoware_planning_validator::PlanningValidatorStatus & status);
  void publish_invalid(const geometry_msgs::Pose & ego_pose,
                       const autoware_planning_validator::PlanningValidatorStatus & status);

private:
  ros::Publisher marker_pub_;
  ros::Publisher virtual_wall_pub_;
};
}  // namespace autoware::planning_validator

#endif
