#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "component_interface_specs_qos", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
