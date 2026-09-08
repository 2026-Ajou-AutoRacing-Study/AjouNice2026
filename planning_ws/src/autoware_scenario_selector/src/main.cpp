#include "autoware/scenario_selector/node.hpp"

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "scenario_selector");
  ros::NodeHandle node;
  autoware::scenario_selector::ScenarioSelectorNode selector(node);
  ros::spin();
  return 0;
}
