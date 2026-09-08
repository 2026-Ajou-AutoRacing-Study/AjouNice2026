#include "autoware/planning_validator/planning_validator_manager.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(PlanningValidatorFullProviderManager, DiscoversAllFourCandidateProviders)
{
  ros::NodeHandle private_nh("~full_provider_manager_oracle");
  autoware::planning_validator::PlanningValidatorManager manager;
  for (const auto & class_name : {
      "autoware::planning_validator::TrajectoryCheckerRos1",
      "autoware::planning_validator::LatencyCheckerRos1",
      "autoware::planning_validator::IntersectionCollisionCheckerRos1",
      "autoware::planning_validator::RearCollisionCheckerRos1"}) {
    EXPECT_TRUE(manager.load_plugin(private_nh, class_name)) << class_name;
  }
  EXPECT_EQ(manager.loaded_plugin_count(), 4U);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_validator_full_provider_manager_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
