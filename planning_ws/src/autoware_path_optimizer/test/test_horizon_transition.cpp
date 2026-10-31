#include <autoware/path_optimizer/horizon_transition.hpp>
#include <gtest/gtest.h>

namespace autoware::path_optimizer
{
std::vector<TrajectoryPoint> straightHorizon(const double phase = 0.0)
{
  std::vector<TrajectoryPoint> result;
  for (int i = -25; i <= 200; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = 0.2 * i + phase;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 8.0;
    result.push_back(point);
  }
  return result;
}

TEST(HorizonTransition, ResamplingPhaseAndVelocityDoNotPreventReturn)
{
  const auto previous = straightHorizon();
  auto candidate = straightHorizon(0.09);
  for (auto & point : candidate) point.longitudinal_velocity_mps = 0.0;
  geometry_msgs::Pose ego;
  ego.position.x = 0.08;
  EXPECT_TRUE(isHorizonReturnContinuous(previous, candidate, ego, 20.0));
}

TEST(HorizonTransition, RejectsNearFieldTargetShiftOnOtherwiseFeasibleShortPath)
{
  const auto previous = straightHorizon();
  auto candidate = previous;
  for (auto & point : candidate) {
    const double x = point.pose.position.x;
    if (x > 1.0 && x < 5.0) point.pose.position.y = 0.157;
  }
  EXPECT_FALSE(isHorizonReturnContinuous(previous, candidate, geometry_msgs::Pose(), 20.0));
}

TEST(HorizonTransition, RejectsHeadingJumpEvenWhenPositionsOverlap)
{
  const auto previous = straightHorizon();
  auto candidate = previous;
  for (auto & point : candidate) {
    point.pose.orientation.z = std::sin(0.02 / 2.0);
    point.pose.orientation.w = std::cos(0.02 / 2.0);
  }
  EXPECT_FALSE(isHorizonReturnContinuous(previous, candidate, geometry_msgs::Pose(), 20.0));
}

TEST(HorizonTransition, DoesNotLatchWindowForChangesBeyondComparedNearField)
{
  const auto previous = straightHorizon();
  auto candidate = previous;
  for (auto & point : candidate) {
    if (point.pose.position.x > 25.0) point.pose.position.y = 1.0;
  }
  EXPECT_TRUE(isHorizonReturnContinuous(previous, candidate, geometry_msgs::Pose(), 20.0));
}

TEST(HorizonTransition, RequiresUsablePreviousAndCandidateGeometry)
{
  const auto previous = straightHorizon();
  EXPECT_FALSE(isHorizonReturnContinuous({}, previous, geometry_msgs::Pose(), 20.0));
  EXPECT_FALSE(isHorizonReturnContinuous(previous, {}, geometry_msgs::Pose(), 20.0));
}
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
