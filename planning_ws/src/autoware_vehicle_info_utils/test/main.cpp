#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_vehicle_info_utils");
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
