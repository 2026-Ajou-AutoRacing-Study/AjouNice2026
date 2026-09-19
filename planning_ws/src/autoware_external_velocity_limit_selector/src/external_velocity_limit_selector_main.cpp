#include "autoware/external_velocity_limit_selector/external_velocity_limit_selector_node.hpp"
#include <ros/ros.h>
int main(int argc, char ** argv) {
  ros::init(argc, argv, "external_velocity_limit_selector");
  ros::NodeHandle nh; ros::NodeHandle private_nh("~");
  autoware::external_velocity_limit_selector::ExternalVelocityLimitSelectorNode node(nh, private_nh);
  ros::spin(); return 0;
}
