#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>
#include <autoware_perception_msgs/PredictedObjects.h>

namespace autoware::behavior_path_planner
{
struct StartGoalPlannerData
{
  autoware_perception_msgs::PredictedObjects filtered_objects{};
  utils::path_safety_checker::TargetObjectsOnLane target_objects_on_lane{};
  std::vector<utils::path_safety_checker::PoseWithVelocityStamped> ego_predicted_path{};
  utils::path_safety_checker::CollisionCheckDebugMap collision_check{};
};
}
#endif
