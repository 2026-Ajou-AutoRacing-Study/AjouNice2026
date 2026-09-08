#include "autoware/remaining_distance_time_calculator/node.hpp"
#include <ros/ros.h>
int main(int argc, char ** argv) { ros::init(argc, argv, "remaining_distance_time_calculator"); ros::NodeHandle node; autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode calculator(node); ros::spin(); return 0; }
