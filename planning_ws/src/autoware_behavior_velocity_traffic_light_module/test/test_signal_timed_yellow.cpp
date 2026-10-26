// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Pure decision tests: no ROS master, simulator, signal command, or live control.
#include <autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>

namespace compliance = autoware::behavior_velocity_planner::signal_compliance;
using compliance::Observation;

namespace
{
void greenSequence(compliance::Guard & guard, double last_source = 10.)
{
  guard.observe(Observation::AuthorizedGreen, last_source-.2, last_source-.2, .5);
  guard.observe(Observation::AuthorizedGreen, last_source-.1, last_source-.1, .5);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, last_source, last_source, .5));
}

auto candidate(double distance = 10., double velocity = 10., double acceleration = 0.,
               double now = 10.15, double minimum_yellow = 2., double margin = .2)
{
  return compliance::timedYellowDecision(
    true, true, false, velocity, acceleration, 1., distance, now, 10.,
    minimum_yellow, margin);
}
}  // namespace

TEST(SignalTimedYellowGuard, OneTwoThreeDistinctGreenObservationsAreConfigurable)
{
  for (std::size_t count = 1; count <= 3; ++count) {
    SCOPED_TRACE(count);
    compliance::Guard guard;
    guard.setRequiredGreenObservations(count);
    for (std::size_t index = 0; index < count; ++index) {
      const double stamp = 10.+index*.1;
      EXPECT_EQ(guard.observe(Observation::AuthorizedGreen, stamp, stamp, .5), index+1 < count);
      // Twenty extra callbacks from the same source never make a new frame.
      for (int repeat = 0; repeat < 20; ++repeat) {
        EXPECT_EQ(guard.observe(Observation::AuthorizedGreen, stamp, stamp+.01, .5), index+1 < count);
      }
      EXPECT_EQ(guard.greenConfirmationCount(), index+1);
    }
  }
}

TEST(SignalTimedYellowGuard, InvalidConfirmationCountsThrowAndChangingCountResetsEvidence)
{
  compliance::Guard guard;
  EXPECT_THROW(guard.setRequiredGreenObservations(0), std::invalid_argument);
  EXPECT_THROW(guard.setRequiredGreenObservations(4), std::invalid_argument);
  greenSequence(guard);
  guard.observe(Observation::Yellow, 10.1, 10.1, .5);
  ASSERT_TRUE(guard.yellowStartLowerBoundSec());
  guard.setRequiredGreenObservations(2);
  EXPECT_FALSE(guard.yellowTransitionEligible());
  EXPECT_FALSE(guard.yellowStartLowerBoundSec());
  EXPECT_EQ(guard.greenConfirmationCount(), 0U);
}

TEST(SignalTimedYellowGuard, TimeBudgetStartsAtLastAuthorizedImageNotFirstYellowReceipt)
{
  compliance::Guard guard;
  greenSequence(guard);
  EXPECT_FALSE(guard.yellowStartLowerBoundSec().has_value());
  EXPECT_TRUE(guard.observe(Observation::Yellow, 10.1, 10.15, .5));
  ASSERT_TRUE(guard.yellowTransitionEligible());
  ASSERT_TRUE(guard.yellowStartLowerBoundSec());
  EXPECT_DOUBLE_EQ(*guard.yellowStartLowerBoundSec(), 10.);
  const auto result = compliance::timedYellowDecision(
    true, guard.yellowTransitionEligible(), false, 10., 0., 1., 10.,
    10.15, guard.yellowStartLowerBoundSec(), 2., .2);
  EXPECT_TRUE(result.allow);
  EXPECT_NEAR(result.remaining_sec, 1.85, 1e-10);
  EXPECT_NEAR(result.arrival_sec, 1., 1e-10);
}

TEST(SignalTimedYellowGuard, RepeatedYellowNeverRenewsTheStartTime)
{
  compliance::Guard guard;
  greenSequence(guard);
  for (int index = 1; index <= 40; ++index) {
    const double stamp = 10.+index*.1;
    guard.observe(Observation::Yellow, stamp, stamp+.01, .5);
    ASSERT_TRUE(guard.yellowTransitionEligible());
    ASSERT_TRUE(guard.yellowStartLowerBoundSec());
    EXPECT_DOUBLE_EQ(*guard.yellowStartLowerBoundSec(), 10.);
  }
  const auto expired = compliance::timedYellowDecision(
    true, true, false, 10., 0., 1., .1, 14.01, guard.yellowStartLowerBoundSec(), 2., .2);
  EXPECT_FALSE(expired.allow);
}

TEST(SignalTimedYellowGuard, FirstYellowAndUnconfirmedGreenCannotInventAStartTime)
{
  for (const bool one_unconfirmed_green : {false, true}) {
    compliance::Guard guard;
    if (one_unconfirmed_green) {
      guard.observe(Observation::AuthorizedGreen, 10., 10., .5);
    }
    guard.observe(Observation::Yellow, 10.1, 10.1, .5);
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_FALSE(guard.yellowStartLowerBoundSec());
  }
}

TEST(SignalTimedYellowGuard, RedMissingStaleUnknownAndExplicitResetClearTimeEvidence)
{
  for (const auto interruption : {Observation::Stop, Observation::Missing,
                                  Observation::Stale, Observation::Unknown}) {
    compliance::Guard guard;
    greenSequence(guard);
    guard.observe(Observation::Yellow, 10.1, 10.1, .5);
    ASSERT_TRUE(guard.yellowStartLowerBoundSec());
    guard.observe(interruption, 10.2, 10.2, .5);
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_FALSE(guard.yellowStartLowerBoundSec());
    guard.observe(Observation::Yellow, 10.3, 10.3, .5);
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_FALSE(guard.yellowStartLowerBoundSec());
  }
  compliance::Guard guard;
  greenSequence(guard);
  guard.observe(Observation::Yellow, 10.1, 10.1, .5);
  guard.reset();
  EXPECT_FALSE(guard.yellowStartLowerBoundSec());
}

TEST(SignalTimedYellowGuard, StaleSourceSourceRollbackAndClockRollbackCannotCarryDeadline)
{
  for (int invalidity = 0; invalidity < 3; ++invalidity) {
    compliance::Guard guard;
    greenSequence(guard);
    guard.observe(Observation::Yellow, 10.1, 10.1, .5);
    if (invalidity == 0) guard.observe(Observation::Yellow, 10.1, 10.7, .5);
    if (invalidity == 1) guard.observe(Observation::Yellow, 10.05, 10.2, .5);
    if (invalidity == 2) guard.observe(Observation::Yellow, 9.9, 9.9, .5);
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_FALSE(guard.yellowStartLowerBoundSec());
  }
}

TEST(SignalTimedYellowDecision, MinimumDurationOneTwoThreeSecondsChangesReachability)
{
  EXPECT_FALSE(candidate(12.5, 10., 0., 10.1, 1.).allow);
  EXPECT_TRUE(candidate(12.5, 10., 0., 10.1, 2.).allow);
  EXPECT_TRUE(candidate(12.5, 10., 0., 10.1, 3.).allow);
}

TEST(SignalTimedYellowDecision, CandidateDoesNotRequirePhysicalInabilityToStop)
{
  // At 5m/s the previous stopping model says ~6m is enough to stop. A new
  // candidate may nevertheless pass 7m away because its time budget allows it;
  // no stopping-distance argument exists in this explicitly different policy.
  EXPECT_TRUE(candidate(7., 5.).allow);
  EXPECT_FALSE(candidate(9., 5.).allow);  // 1.8+.2 > remaining 1.85.
}

TEST(SignalTimedYellowDecision, ActualElapsedTimeAndExplicitMarginConsumeBudget)
{
  EXPECT_TRUE(candidate(10., 10., 0., 10.15).allow);
  EXPECT_FALSE(candidate(10., 10., 0., 11.).allow);
  EXPECT_TRUE(candidate(12.5, 10., 0., 10.5, 2., .25).allow);
  EXPECT_FALSE(candidate(12.50001, 10., 0., 10.5, 2., .25).allow);
  EXPECT_FALSE(candidate(1., 10., 0., 11.9, 2., .2).allow);
}

TEST(SignalTimedYellowDecision, PositiveAccelerationDoesNotOptimisticallyShortenArrival)
{
  const auto zero = candidate(10., 5., 0.);
  const auto positive = candidate(10., 5., 3.);
  EXPECT_FALSE(zero.allow);
  EXPECT_FALSE(positive.allow);
  EXPECT_DOUBLE_EQ(positive.arrival_sec, zero.arrival_sec);
}

TEST(SignalTimedYellowDecision, CurrentDecelerationMakesArrivalLaterThanDistanceOverSpeed)
{
  const auto result = candidate(10., 10., -2.);
  EXPECT_TRUE(result.allow);
  EXPECT_GT(result.arrival_sec, 1.);
  EXPECT_NEAR(result.arrival_sec, (10.-std::sqrt(60.))/2., 1e-9);
  EXPECT_FALSE(candidate(20., 10., -2.).allow);
}

TEST(SignalTimedYellowDecision, VehicleThatStopsBeforeLineCannotClaimAnArrival)
{
  EXPECT_FALSE(candidate(17., 10., -3.).allow);
  EXPECT_FALSE(candidate(10., 1., -1.).allow);
  // Reaching the line with zero speed is not crossing it.
  EXPECT_FALSE(candidate(10., 10., -5., 10.1, 5.).allow);
}

TEST(SignalTimedYellowDecision, DisabledUnobservedPreviouslyStoppedAndSlowCannotGo)
{
  EXPECT_FALSE(compliance::timedYellowDecision(false, true, false, 10., 0., 1., 1., 10.1, 10., 2., .2).allow);
  EXPECT_FALSE(compliance::timedYellowDecision(true, false, false, 10., 0., 1., 1., 10.1, 10., 2., .2).allow);
  EXPECT_FALSE(compliance::timedYellowDecision(true, true, true, 10., 0., 1., 1., 10.1, 10., 2., .2).allow);
  EXPECT_FALSE(candidate(.1, 0.).allow);
  EXPECT_FALSE(candidate(.1, .9).allow);
  EXPECT_FALSE(candidate(.1, -1.).allow);
}

TEST(SignalTimedYellowDecision, MissingFutureAndExpiredStartEvidenceCannotGo)
{
  EXPECT_FALSE(compliance::timedYellowDecision(true, true, false, 10., 0., 1., 1., 10.1, std::nullopt, 2., .2).allow);
  EXPECT_FALSE(compliance::timedYellowDecision(true, true, false, 10., 0., 1., 1., 9.9, 10., 2., .2).allow);
  EXPECT_FALSE(candidate(.1, 10., 0., 12.1).allow);
}

TEST(SignalTimedYellowDecision, InvalidGeometryAndConfigurationFailClosed)
{
  // An already-moving axle exactly at the line has zero arrival time. The
  // scene's independent crossed-line handling owns negative signed distance.
  EXPECT_TRUE(candidate(0.).allow);
  EXPECT_FALSE(candidate(-1.).allow);
  EXPECT_FALSE(candidate(1., 10., 0., 10.1, 0.).allow);
  EXPECT_FALSE(candidate(1., 10., 0., 10.1, -1.).allow);
  EXPECT_FALSE(candidate(1., 10., 0., 10.1, 2., -.1).allow);
  EXPECT_FALSE(candidate(1., 10., 0., 10.1, 2., 2.).allow);
  EXPECT_FALSE(compliance::timedYellowDecision(true, true, false, 10., 0., 0., 1., 10.1, 10., 2., .2).allow);
}

TEST(SignalTimedYellowDecision, EveryNonfiniteInputFailsClosed)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double invalid : {nan, inf, -inf}) {
    EXPECT_FALSE(candidate(1., invalid).allow);
    EXPECT_FALSE(candidate(1., 10., invalid).allow);
    EXPECT_FALSE(candidate(invalid).allow);
    EXPECT_FALSE(candidate(1., 10., 0., invalid).allow);
    EXPECT_FALSE(candidate(1., 10., 0., 10.1, invalid).allow);
    EXPECT_FALSE(candidate(1., 10., 0., 10.1, 2., invalid).allow);
    EXPECT_FALSE(compliance::timedYellowDecision(true, true, false, 10., 0., invalid, 1., 10.1, 10., 2., .2).allow);
    EXPECT_FALSE(compliance::timedYellowDecision(true, true, false, 10., 0., 1., 1., 10.1, invalid, 2., .2).allow);
  }
}
