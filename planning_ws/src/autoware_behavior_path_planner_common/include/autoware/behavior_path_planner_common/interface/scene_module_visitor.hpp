#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_VISITOR_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_VISITOR_HPP_
#include <autoware_internal_debug_msgs/StringStamped.h>
#include <tier4_planning_msgs/AvoidanceDebugMsgArray.h>
#include <memory>
namespace autoware::behavior_path_planner {
class StaticObstacleAvoidanceModule; class StartPlannerModule;
class SceneModuleVisitor {
public:
  void visitAvoidanceModule(const StaticObstacleAvoidanceModule * module) const;
  void visitStartPlannerModule(const StartPlannerModule * module) const;
  std::shared_ptr<tier4_planning_msgs::AvoidanceDebugMsgArray> getAvoidanceModuleDebugMsg() const;
  std::shared_ptr<autoware_internal_debug_msgs::StringStamped> getStartPlannerModuleDebugMsg() const;
private:
  mutable std::shared_ptr<tier4_planning_msgs::AvoidanceDebugMsgArray> avoidance_visitor_;
  mutable std::shared_ptr<autoware_internal_debug_msgs::StringStamped> start_planner_visitor_;
}; }
#endif
