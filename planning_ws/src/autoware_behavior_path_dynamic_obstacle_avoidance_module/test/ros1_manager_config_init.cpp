#include <autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_doa_manager_config_init");
  ros::NodeHandle private_node("aw149_doa_manager_config_init");
  autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager manager;

  // The caller must rosparam-load the value-parity checked ROS1 artifact into
  // this private namespace.  Unlike the missing-config regression, success
  // here proves every typed declaration can be consumed by the candidate.
  return manager.init(&private_node) ? 0 : 1;
}
