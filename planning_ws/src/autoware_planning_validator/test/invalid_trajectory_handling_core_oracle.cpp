#include "autoware/planning_validator/invalid_trajectory_handling_core.hpp"

#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::Trajectory trajectory(const double final_x, const float final_velocity)
{
  autoware_planning_msgs::Trajectory result;
  for (int index = 0; index != 3; ++index) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = index == 2 ? final_x : static_cast<double>(index);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = index == 2 ? final_velocity : 4.0F;
    result.points.push_back(point);
  }
  return result;
}

TEST(InvalidTrajectoryHandling, MirrorsTheThreeSourcePolicies)
{
  const auto current = trajectory(2.0, 8.0F);
  const auto previous = trajectory(20.0, 3.0F);
  using Type = autoware::planning_validator::InvalidTrajectoryHandlingType;
  EXPECT_EQ(
    autoware::planning_validator::select_invalid_trajectory(current, &previous, Type::PublishAsIs)
      ->points.back().pose.position.x,
    2.0);
  EXPECT_EQ(
    autoware::planning_validator::select_invalid_trajectory(current, &previous, Type::UsePreviousResult)
      ->points.back().pose.position.x,
    20.0);
  EXPECT_FALSE(autoware::planning_validator::select_invalid_trajectory(
    current, nullptr, Type::UsePreviousResult).has_value());
}

TEST(InvalidTrajectoryHandling, SoftStopEndsAtZeroVelocity)
{
  auto parameters = autoware::planning_validator::InvalidTrajectoryHandlingParameters{};
  const auto output = autoware::planning_validator::make_soft_stop_trajectory(
    trajectory(20.0, 4.0F), 0U, 4.0, 0.0, parameters);
  ASSERT_FALSE(output.points.empty());
  EXPECT_FLOAT_EQ(output.points.back().longitudinal_velocity_mps, 0.0F);
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
