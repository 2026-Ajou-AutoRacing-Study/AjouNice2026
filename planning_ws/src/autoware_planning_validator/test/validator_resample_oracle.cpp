#include <autoware/motion_utils/resample/resample.hpp>

#include <gtest/gtest.h>

TEST(PlanningValidatorContext, UsesSourceOneMetreTrajectoryResampling)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (int i = 0; i != 3; ++i) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 1.0F;
    trajectory.points.push_back(point);
  }

  const auto result = autoware::motion_utils::resampleTrajectory(trajectory, 1.0);
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_DOUBLE_EQ(result.points.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.points.back().pose.position.x, 2.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
