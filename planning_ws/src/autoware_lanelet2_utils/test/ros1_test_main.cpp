#include <gtest/gtest.h>
#include <ros/time.h>

int main(int argc, char ** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
