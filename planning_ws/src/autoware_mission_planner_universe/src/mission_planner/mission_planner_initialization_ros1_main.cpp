#include "mission_planner_initialization_ros1.hpp"

#include <ros/ros.h>

#include <exception>
#include <iostream>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mission_planner");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  try {
    autoware::mission_planner_universe::MissionPlannerInitializationRos1 planner(node, private_node);
    ros::spin();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "mission_planner initialization error: " << error.what() << std::endl;
    return 1;
  }
}
