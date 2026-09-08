#include <autoware/planning_validator/planning_validator_manager.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace
{
TEST(TrajectoryCheckerManager, LoadsExternalProviderAndKeepsUnrenderedCollisionChecksFalse)
{
  ros::NodeHandle private_nh("~trajectory_checker_manager_oracle");
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(private_nh, "autoware::planning_validator::TrajectoryCheckerRos1"));
  ASSERT_EQ(manager.loaded_plugin_count(), 1U);
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points.resize(3);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points.at(i).pose.position.x = static_cast<double>(i) * 5.0;
    trajectory.points.at(i).pose.orientation.w = 1.0;
  }
  const auto status = manager.validate({trajectory});
  EXPECT_TRUE(status.is_valid_size);
  EXPECT_TRUE(status.is_valid_finite_value);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
  EXPECT_FALSE(status.is_valid_intersection_collision_check);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_checker_manager_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
