#include <autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp>
namespace autoware::behavior_path_planner {
std::shared_ptr<tier4_planning_msgs::AvoidanceDebugMsgArray> SceneModuleVisitor::getAvoidanceModuleDebugMsg() const { return avoidance_visitor_; }
std::shared_ptr<autoware_internal_debug_msgs::StringStamped> SceneModuleVisitor::getStartPlannerModuleDebugMsg() const { return start_planner_visitor_; }
}
