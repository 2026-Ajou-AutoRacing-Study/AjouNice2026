#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "autoware_behavior_path_static_obstacle_avoidance_module_test");
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
