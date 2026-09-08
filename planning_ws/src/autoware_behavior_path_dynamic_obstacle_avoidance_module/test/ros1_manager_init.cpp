#include <autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_doa_manager_init");
  ros::NodeHandle private_node("aw149_doa_manager_init");
  autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager manager;
  // A missing ROS1 parameter translation must fail closed, never throw or
  // manufacture an unsafe planner configuration.
  return manager.init(&private_node) ? 1 : 0;
}
