#include <autoware/planning_validator/planning_validator_manager.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RearCollisionCheckerManager, LoadsExternalProviderAndRetainsFailClosedResult)
{
  ros::NodeHandle private_nh("~rear_collision_checker_manager_oracle");
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(
    private_nh, "autoware::planning_validator::RearCollisionCheckerRos1"));
  ASSERT_EQ(manager.loaded_plugin_count(), 1U);

  autoware_planning_msgs::Trajectory trajectory;
  const auto status = manager.validate({trajectory});
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rear_collision_checker_manager_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
