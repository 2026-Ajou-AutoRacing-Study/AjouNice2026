#include "trajectory_follower/longitudinal_pi_utils.hpp"

#include <gtest/gtest.h>
#include <limits>

TEST(LongitudinalPiUtils, FeedforwardIsZeroForDisabledBrakingAndInvalidInputs)
{
  using trajectory_follower::accelerationFeedforward;
  EXPECT_DOUBLE_EQ(accelerationFeedforward(false, true, 3.7, 3.7, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, false, 3.7, 3.7, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, -2.0, 3.7, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, 0.0, 3.7, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true,
    std::numeric_limits<double>::quiet_NaN(), 3.7, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, 3.7, 0.0, 1.0), 0.0);
}

TEST(LongitudinalPiUtils, FeedforwardUsesAccelerationAndCapsPedal)
{
  using trajectory_follower::accelerationFeedforward;
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, 1.85, 3.7, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, 3.7, 3.7, 0.5), 0.5);
  EXPECT_DOUBLE_EQ(accelerationFeedforward(true, true, 7.4, 3.7, 1.0), 1.0);
}

TEST(LongitudinalPiUtils, FeedforwardRiseNeverDelaysBraking)
{
  using trajectory_follower::limitFeedforwardRise;
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.75, 0.0, 0.5, 0.02), 0.01);
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.75, 0.74, 0.5, 0.02), 0.75);
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.2, 0.75, 0.5, 0.02), 0.2);
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.0, 0.75, 0.5, 0.02), 0.0);
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.75, 0.0, 0.0, 0.02), 0.75);
  EXPECT_DOUBLE_EQ(limitFeedforwardRise(0.75, 0.0, 0.5, 0.0), 0.0);
}

TEST(LongitudinalPiUtils, IncreasingProfileUsesCurrentPositionAndNeverExtrapolates)
{
  using trajectory_follower::interpolateIncreasingVelocity;
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(1., 3., .5, 1.), std::sqrt(5.));
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(1., 3., -1., 1.), 1.);
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(1., 3., 2., 1.), 3.);
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(3., 1., .5, 2.), 2.);
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(0., 1., .5, 0.), 0.);
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(3., 3., .5, 2.), 2.);
  EXPECT_DOUBLE_EQ(interpolateIncreasingVelocity(
    1., 3., std::numeric_limits<double>::quiet_NaN(), 2.), 2.);
}

TEST(LongitudinalPiUtils, ResetsAccelerationIntegralWhenBrakingBegins)
{
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(-0.2, 2.5, true), 0.0);
}

TEST(LongitudinalPiUtils, PreservesBrakeIntegralWhenErrorBrieflyTurnsPositive)
{
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(0.2, -2.5, true), -2.5);
}

TEST(LongitudinalPiUtils, PreservesIntegralWhenItSupportsTheCurrentError)
{
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(0.2, 2.5, true), 2.5);
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(-0.2, -2.5, true), -2.5);
}

TEST(LongitudinalPiUtils, PreservesIntegralAtZeroError)
{
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(0.0, 2.5, true), 2.5);
}

TEST(LongitudinalPiUtils, CanBeDisabledForControlledComparisons)
{
  EXPECT_DOUBLE_EQ(
    trajectory_follower::resetAccelerationIntegralOnBraking(-0.2, 2.5, false), 2.5);
}

TEST(LongitudinalPiUtils, BrakeHysteresisEntersOnOverspeed)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {}, -0.08, -1.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.hard_exit_elapsed_s, 0.0);
}

TEST(LongitudinalPiUtils, BrakeHysteresisIgnoresSmallErrorBeforeEntry)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {}, -0.07, -5.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_FALSE(result.active);
}

TEST(LongitudinalPiUtils, BrakeHysteresisStaysActiveDuringPlannedDeceleration)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {true, 0.0}, 0.16, -5.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_TRUE(result.active);
}

TEST(LongitudinalPiUtils, BrakeHysteresisExitsWhenErrorRecovers)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {true, 0.0}, 0.16, -0.2, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_FALSE(result.active);
}

TEST(LongitudinalPiUtils, BrakeHysteresisDebouncesHardUnderspeedExit)
{
  const auto transient = trajectory_follower::updateBrakeHysteresis(
    {true, 0.0}, 0.5, -5.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_TRUE(transient.active);
  EXPECT_DOUBLE_EQ(transient.hard_exit_elapsed_s, 0.02);

  const auto sustained = trajectory_follower::updateBrakeHysteresis(
    {true, 0.08}, 0.5, -5.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_FALSE(sustained.active);
  EXPECT_DOUBLE_EQ(sustained.hard_exit_elapsed_s, 0.0);
}

TEST(LongitudinalPiUtils, BrakeHysteresisResetsHardExitTimerWhenErrorRecovers)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {true, 0.08}, 0.49, -5.0, 0.02, true, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.hard_exit_elapsed_s, 0.0);
}

TEST(LongitudinalPiUtils, BrakeHysteresisCanBeDisabled)
{
  const auto result = trajectory_follower::updateBrakeHysteresis(
    {true, 0.08}, -1.0, -5.0, 0.02, false, 0.08, 0.15, 0.5, 0.1, 0.5);
  EXPECT_FALSE(result.active);
  EXPECT_DOUBLE_EQ(result.hard_exit_elapsed_s, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
