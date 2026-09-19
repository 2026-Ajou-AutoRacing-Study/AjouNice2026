#include "trajectory_follower/steering_rate_limiter.hpp"

#include <cmath>

#include <gtest/gtest.h>

TEST(SteeringRateLimiter, LimitsPositiveAndNegativeChangesAtSixtyDegreesPerSecond)
{
  const double maximum_rate = 60.0 * M_PI / 180.0;
  constexpr double dt = 0.02;
  const double maximum_delta = 1.2 * M_PI / 180.0;

  EXPECT_NEAR(
    trajectory_follower::limitSteeringRate(M_PI / 4.5, 0.0, maximum_rate, dt),
    maximum_delta, 1e-12);
  EXPECT_NEAR(
    trajectory_follower::limitSteeringRate(-M_PI / 4.5, 0.0, maximum_rate, dt),
    -maximum_delta, 1e-12);
}

TEST(SteeringRateLimiter, PassesTargetsInsideThePerTickLimit)
{
  const double maximum_rate = 60.0 * M_PI / 180.0;
  constexpr double dt = 0.02;
  const double previous = 5.0 * M_PI / 180.0;
  const double target = 5.5 * M_PI / 180.0;

  EXPECT_DOUBLE_EQ(
    trajectory_follower::limitSteeringRate(target, previous, maximum_rate, dt), target);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
