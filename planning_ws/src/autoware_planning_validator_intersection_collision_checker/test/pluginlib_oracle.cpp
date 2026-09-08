#include <autoware/planning_validator/plugin_interface.hpp>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

TEST(IntersectionCollisionCheckerPluginlib, LoadsCandidateR004Provider)
{
  pluginlib::ClassLoader<autoware::planning_validator::PluginInterface> loader(
    "autoware_planning_validator", "autoware::planning_validator::PluginInterface");
  ASSERT_TRUE(loader.isClassAvailable("autoware::planning_validator::IntersectionCollisionCheckerRos1"));
  const auto checker = loader.createInstance("autoware::planning_validator::IntersectionCollisionCheckerRos1");
  ros::NodeHandle private_nh("~icc_r004_pluginlib_oracle");
  checker->initialize(private_nh, "autoware::planning_validator::IntersectionCollisionCheckerRos1");
  EXPECT_EQ(checker->module_name(), "autoware::planning_validator::IntersectionCollisionCheckerRos1");
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "icc_r004_pluginlib_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
