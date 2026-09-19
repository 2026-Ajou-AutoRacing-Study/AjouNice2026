#include "autoware/planning_validator_intersection_collision_checker/intersection_collision_checker_plugin.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(IntersectionCollisionCheckerProvider, FailsClosedBeforeFullSourcePathIsConnected)
{
  ros::NodeHandle private_nh("~icc_r004_provider_oracle");
  autoware::planning_validator::IntersectionCollisionCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::IntersectionCollisionCheckerRos1");
  autoware_planning_msgs::Trajectory trajectory;
  autoware::planning_validator::TrajectoryCheckerInput input{trajectory};
  autoware_planning_validator::PlanningValidatorStatus status;
  status.is_valid_intersection_collision_check = true;
  checker.validate(input, status);
  EXPECT_FALSE(status.is_valid_intersection_collision_check);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "icc_r004_provider_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
