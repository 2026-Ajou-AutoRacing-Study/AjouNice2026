#include <gtest/gtest.h>
#include <trajectory_follower/distance_speed_limit.hpp>

using trajectory_follower::distanceSpeedLimit;
using trajectory_follower::brakingPreviewDistance;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DistanceSpeedLimit, DistantStopDoesNotRequestImmediateStandstill)
{
  const double current = 5.0 / 3.6;
  EXPECT_GT(distanceSpeedLimit(0.0, 5.0, current, 2.0, 0.3, 0.2), current);
}

TEST(DistanceSpeedLimit, NearStopRequiresDeceleration)
{
  EXPECT_LT(distanceSpeedLimit(0.0, 2.0, 5.0, 2.0, 0.3, 0.2), 5.0);
  EXPECT_DOUBLE_EQ(distanceSpeedLimit(0.0, 0.1, 5.0, 2.0, 0.3, 0.2), 0.0);
}

TEST(DistanceSpeedLimit, DistanceBudgetIncludesResponseDelay)
{
  const double allowed = distanceSpeedLimit(3.0, 10.0, 8.0, 2.0, 0.3, 0.2);
  EXPECT_NEAR((allowed * allowed - 9.0) / 4.0 + 8.0 * 0.3 + 0.2, 10.0, 1e-10);
}

TEST(DistanceSpeedLimit, ReachesFutureSpeedAtRestriction)
{
  EXPECT_DOUBLE_EQ(distanceSpeedLimit(3.0, 0.0, 5.0, 2.0, 0.3, 0.2), 3.0);
}

TEST(DistanceSpeedLimit, IncreasingDistanceRelaxesCurrentBound)
{
  EXPECT_GT(distanceSpeedLimit(2.0, 10.0, 5.0, 2.0, 0.3, 0.2),
    distanceSpeedLimit(2.0, 3.0, 5.0, 2.0, 0.3, 0.2));
}

TEST(DistanceSpeedLimit, FastApproachLooksBeyondOldThirtyMetreHorizon)
{
  const double horizon = brakingPreviewDistance(20.0, 2.0, 0.3, 0.2);
  EXPECT_GT(horizon, 100.0);
  EXPECT_NEAR(distanceSpeedLimit(0.0, horizon, 20.0, 2.0, 0.3, 0.2), 20.0, 1e-10);
}
