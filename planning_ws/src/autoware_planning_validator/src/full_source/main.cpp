#include <autoware/planning_validator/full_source/node.hpp>
#include <exception>
#include <iostream>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"planning_validator");
  try {
    autoware::planning_validator::full_source::PlanningValidatorNode node{ros::NodeHandle("~")};
    ros::spin();
  } catch(const std::exception & error) {
    // The last NodeHandle may already have shut ROS logging down while unwinding.
    std::cerr << "Full planning validator startup/execution failed: " << error.what() << std::endl;
    ROS_FATAL("Full planning validator startup/execution failed: %s",error.what());
    return 1;
  }
  return 0;
}
