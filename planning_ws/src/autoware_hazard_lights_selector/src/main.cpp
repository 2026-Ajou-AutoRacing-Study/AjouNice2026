#include "autoware/hazard_lights_selector/node.hpp"
#include <ros/ros.h>
int main(int argc, char ** argv) { ros::init(argc, argv, "autoware_hazard_lights_selector"); ros::NodeHandle nh, private_nh("~"); autoware::hazard_lights_selector::HazardLightsSelector node(nh, private_nh); ros::spin(); return 0; }
