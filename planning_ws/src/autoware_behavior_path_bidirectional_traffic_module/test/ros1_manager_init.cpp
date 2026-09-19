#include <autoware/behavior_path_bidirectional_traffic_module/manager.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_bidirectional_traffic_manager_init");
  ros::NodeHandle private_node("aw149_bidirectional_traffic_manager_init");
  autoware::behavior_path_planner::BidirectionalTrafficModuleManager manager;
  if (!manager.init(&private_node)) return 1;
  manager.updateIdleModuleInstance();
  return manager.takeIdleModule() ? 0 : 2;
}
