#include <ros/ros.h>
#include <gtest/gtest.h>
int main(int argc, char ** argv) { ros::init(argc, argv, "autoware_utils_rclcpp_test"); testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
