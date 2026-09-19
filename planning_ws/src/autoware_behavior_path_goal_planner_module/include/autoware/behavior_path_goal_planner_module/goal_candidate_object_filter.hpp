#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_FILTER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_FILTER_HPP_

#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/Pose.h>

namespace autoware::behavior_path_planner
{

// ROS1 ownership of Goal Planner's lateral object-selection policy used before
// its longitudinal-margin collision check.
autoware_perception_msgs::PredictedObjects filterGoalCandidateObjectsByLateralDistance(
  const geometry_msgs::Pose & ego_pose, double vehicle_width,
  const autoware_perception_msgs::PredictedObjects & objects, double distance_threshold,
  bool filter_inside);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_OBJECT_FILTER_HPP_
