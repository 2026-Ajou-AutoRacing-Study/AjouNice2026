#include <autoware/behavior_path_planner_common/avoidance_object_ownership.hpp>
#include <gtest/gtest.h>
#include <limits>

using autoware::behavior_path_planner::AvoidanceObjectOwnership;
namespace
{
void observe(AvoidanceObjectOwnership & policy, double t, double vx, double vy = 0.0, double x = 0.0)
{
  autoware_perception_msgs::PredictedObjects message;
  message.header.stamp = ros::Time(t);
  message.objects.resize(1);
  auto & object = message.objects.front();
  object.kinematics.initial_twist_with_covariance.twist.linear.x = vx;
  object.kinematics.initial_twist_with_covariance.twist.linear.y = vy;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  policy.update(message);
}
unique_identifier_msgs::UUID id;
TEST(AvoidanceObjectOwnership, ExactBoundariesSwitchOnFirstObservation)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.01, 0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
  observe(policy, 1.02, 0.2);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, BandRetainsEachPreviousOwnerWithoutElapsedTime)
{
  AvoidanceObjectOwnership stationary, moving;
  observe(stationary, 1.0, 0.3);
  observe(moving, 1.0, 0.5);
  for (double t : {1.01, 100.0, 10000.0}) {
    observe(stationary, t, 0.3);
    observe(moving, t, 0.3);
    EXPECT_EQ(stationary.isDynamic(id), false);
    EXPECT_EQ(moving.isDynamic(id), true);
  }
}
TEST(AvoidanceObjectOwnership, UsesPlanarMagnitudeIncludingReverseAndCrossingMotion)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0, -0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
  observe(policy, 2.0, 0.0);
  observe(policy, 3.0, -0.5);
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, PositionJitterCannotChangeOwnership)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  observe(policy, 1.01, 0.0, 0.0, 10.0);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, SameInputStampDoesNotAdvanceDecision)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 0.0);
  observe(policy, 1.0, 1.0);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.01, 1.0);
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, MissingDetectionRetainsOnlyOwnerAndClearResetsIt)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 1.0, 1.0);
  autoware_perception_msgs::PredictedObjects empty;
  empty.header.stamp = ros::Time(2.0);
  policy.update(empty);
  observe(policy, 3.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), true);
  policy.clear();
  observe(policy, 4.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), false);
}
TEST(AvoidanceObjectOwnership, RewindStartsFreshAndInvalidVelocityKeepsCollisionOwnership)
{
  AvoidanceObjectOwnership policy;
  observe(policy, 2.0, 1.0);
  observe(policy, 1.0, 0.3);
  EXPECT_EQ(policy.isDynamic(id), false);
  observe(policy, 1.1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(policy.isDynamic(id), true);
}
TEST(AvoidanceObjectOwnership, ThresholdValidation)
{
  EXPECT_THROW(AvoidanceObjectOwnership(0.2, 0.5), std::invalid_argument);
  EXPECT_THROW(AvoidanceObjectOwnership(0.5, -0.1), std::invalid_argument);
}
}  // namespace
