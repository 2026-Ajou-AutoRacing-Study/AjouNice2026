#include <autoware/behavior_path_bidirectional_traffic_module/parameter.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_bidirectional_traffic_parameter_init");
  ros::NodeHandle private_node("aw149_bidirectional_traffic_parameter_init");
  autoware::behavior_path_planner::BidirectionalTrafficModuleParameters parameters;
  return parameters.init_from_node(&private_node) ? 0 : 1;
}
