#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_bpp_doa_slot_init");
  ros::NodeHandle private_node("aw149_bpp_doa_slot_init");
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&private_node);
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  return (core.isInitialized() && core.dynamicObstacleAvoidanceEnabled()) ? 0 : 1;
#else
  return 1;
#endif
}
