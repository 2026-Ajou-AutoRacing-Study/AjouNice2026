#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__OBJECT_DISTANCE_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__OBJECT_DISTANCE_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

namespace autoware::behavior_path_planner::utils
{

// Narrow ROS1 port of the upstream utils.cpp object-distance surface.  Route,
// lanelet and goal-modification utilities intentionally remain outside this
// dependency-light slice.
double l2Norm(const geometry_msgs::Vector3 & vector);

bool checkCollisionBetweenPathFootprintsAndObjects(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const autoware_internal_planning_msgs::PathWithLaneId & ego_path,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects, double margin);

bool checkCollisionBetweenFootprintAndObjects(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const geometry_msgs::Pose & ego_pose,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects, double margin);

double calcLateralDistanceFromEgoToObject(
  const geometry_msgs::Pose & ego_pose, double vehicle_width,
  const autoware_perception_msgs::PredictedObject & dynamic_object);

double calcLongitudinalDistanceFromEgoToObject(
  const geometry_msgs::Pose & ego_pose, double base_link2front, double base_link2rear,
  const autoware_perception_msgs::PredictedObject & dynamic_object);

double calcLongitudinalDistanceFromEgoToObjects(
  const geometry_msgs::Pose & ego_pose, double base_link2front, double base_link2rear,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects);

}  // namespace autoware::behavior_path_planner::utils

#endif
