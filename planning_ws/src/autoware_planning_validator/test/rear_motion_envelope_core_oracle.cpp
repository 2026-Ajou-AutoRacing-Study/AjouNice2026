#include "autoware/planning_validator/rear_motion_envelope_core.hpp"

#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::Trajectory straight_trajectory()
{
  autoware_planning_msgs::Trajectory trajectory;
  for (double x : {0.0, 10.0, 20.0, 30.0, 40.0}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(RearMotionEnvelopeCore, CalculatesSourceShapedLookaheadAndStopLines)
{
  const auto trajectory = straight_trajectory();
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  vehicle.max_longitudinal_offset_m = 1.0;

  const auto result = autoware::planning_validator::compute_rear_motion_envelope(
    {trajectory, odometry, acceleration, vehicle}, {});
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->lookahead_distance, 8.0);
  EXPECT_GT(result->lookahead_left.position.x, 0.0);
  EXPECT_GT(result->predicted_stop_left.position.x, 0.0);
  EXPECT_GT(result->lookahead_left.position.y, result->lookahead_right.position.y);
}

TEST(RearMotionEnvelopeCore, RefusesTrajectoryTooShortForEitherSourceLine)
{
  autoware_planning_msgs::Trajectory trajectory;
  autoware_planning_msgs::TrajectoryPoint point;
  point.pose.orientation.w = 1.0;
  trajectory.points.push_back(point);
  nav_msgs::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  geometry_msgs::AccelWithCovarianceStamped acceleration;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.vehicle_width_m = 2.0;
  EXPECT_FALSE(autoware::planning_validator::compute_rear_motion_envelope(
    {trajectory, odometry, acceleration, vehicle}, {}));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
