#include <ros/ros.h>
#include <gtest/gtest.h>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"full_motion_obstacle_test");ros::NodeHandle owner;
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
