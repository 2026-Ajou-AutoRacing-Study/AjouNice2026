#include "route_selector_ros1.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "route_selector");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  autoware::mission_planner_universe::RouteSelector selector(node, private_node);
  ros::spin();
  return 0;
}
