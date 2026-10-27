// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Pure candidate tests: no ROS master, live control, or simulator input.
#include <autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace compliance = autoware::behavior_velocity_planner::signal_compliance;
using compliance::DecisionLineLatch;
using compliance::DecisionLineState;
using compliance::Observation;

namespace
{
auto update(DecisionLineLatch & latch, Observation observation, bool stop,
            double distance = 5.0, double speed = 10.0, double stamp = 10.0,
            bool vehicle_stopped = false, bool fresh = true, double now = 0.0)
{
  return latch.update(observation, fresh, stop, speed, vehicle_stopped,
    distance, *compliance::jerkStoppingDistance(speed, 8.0, 16.0, 0.0),
    stamp, now > 0.0 ? now : stamp);
}
}  // namespace

TEST(SignalDecisionLineDistance, MatchesSuppliedFormulaExamples)
{
  const double speeds_kph[] = {20, 30, 40, 50, 60};
  const double expected[] = {3.234567901234568, 6.340277777777778,
                            10.410493827160494, 15.445216049382717,
                            21.444444444444446};
  for (int index = 0; index < 5; ++index) {
    const auto value = compliance::jerkStoppingDistance(speeds_kph[index]/3.6, 8, 16, 0);
    ASSERT_TRUE(value);
    EXPECT_NEAR(*value, expected[index], 1e-10);
  }
}

TEST(SignalDecisionLineDistance, TriangularBranchAndBoundaryAreContinuous)
{
  EXPECT_DOUBLE_EQ(*compliance::jerkStoppingDistance(0, 8, 16, 0), 0);
  EXPECT_NEAR(*compliance::jerkStoppingDistance(1, 8, 16, 0),
              (2.0/3.0)*std::sqrt(1.0/8.0), 1e-12);
  EXPECT_NEAR(*compliance::jerkStoppingDistance(2, 8, 16, 0), 2.0/3.0, 1e-12);
  EXPECT_NEAR(*compliance::jerkStoppingDistance(2-1e-7, 8, 16, 0),
              *compliance::jerkStoppingDistance(2+1e-7, 8, 16, 0), 2e-7);
}

TEST(SignalDecisionLineDistance, DelayRemainsSeparateFromJerk)
{
  const auto zero = compliance::jerkStoppingDistance(10, 8, 16, 0);
  const auto delayed = compliance::jerkStoppingDistance(10, 8, 16, .2);
  ASSERT_TRUE(zero);
  ASSERT_TRUE(delayed);
  EXPECT_NEAR(*delayed-*zero, 2, 1e-12);
  EXPECT_GT(*zero, 10.0*10.0/16.0);
}

TEST(SignalDecisionLineDistance, ParkedSignedOdometryNoiseIsZeroNotReverseBraking)
{
  for (const double velocity : {-1e-5, -.05, -.1, 0.0}) {
    const auto distance = compliance::jerkStoppingDistance(velocity, 8, 16, 0);
    ASSERT_TRUE(distance);
    EXPECT_DOUBLE_EQ(*distance, 0.0);
  }
  EXPECT_FALSE(compliance::jerkStoppingDistance(-.100001, 8, 16, 0));
}

TEST(SignalDecisionLineDistance, InvalidAndUnrepresentableInputsFailExplicitly)
{
  const double invalid[] = {-1, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()};
  for (const double value : invalid) {
    EXPECT_FALSE(compliance::jerkStoppingDistance(value, 8, 16, 0));
    EXPECT_FALSE(compliance::jerkStoppingDistance(10, value, 16, 0));
    EXPECT_FALSE(compliance::jerkStoppingDistance(10, 8, value, 0));
    EXPECT_FALSE(compliance::jerkStoppingDistance(10, 8, 16, value));
  }
  EXPECT_FALSE(compliance::jerkStoppingDistance(10, 0, 16, 0));
  EXPECT_FALSE(compliance::jerkStoppingDistance(10, 8, 0, 0));
  EXPECT_FALSE(compliance::jerkStoppingDistance(std::numeric_limits<double>::max(), 8, 16, 0));
}

TEST(SignalDecisionLineLatch, ExistingGuardControlsApproachBeforeBoundary)
{
  DecisionLineLatch latch;
  EXPECT_FALSE(update(latch, Observation::AuthorizedGreen, false, 30).require_stop);
  EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  EXPECT_FALSE(update(latch, Observation::Unknown, false, 20, 10, 10.1).require_stop);
  EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  EXPECT_TRUE(update(latch, Observation::Yellow, true, 20, 10, 10.2).require_stop);
  EXPECT_TRUE(latch.pendingStop());
}

TEST(SignalDecisionLineLatch, BoundaryUsesLessOrEqualAndDoesNotNeedExactEquality)
{
  for (const double distance : {8.666666666666666, 8.5, -0.25}) {
    DecisionLineLatch latch;
    const auto result = update(latch, Observation::AuthorizedGreen, false, distance);
    EXPECT_FALSE(result.require_stop);
    EXPECT_TRUE(result.committed_go);
    EXPECT_EQ(result.state, DecisionLineState::Go);
    EXPECT_DOUBLE_EQ(result.stopping_target_distance_m, distance);
    EXPECT_NEAR(result.stopping_distance_m, 8.666666666666666, 1e-12);
  }
}

TEST(SignalDecisionLineLatch, DirectionAuthorizationAbstractlySupportsABCD)
{
  // The caller maps straight green (A/B), G+left (C), and R+left (D)
  // to the same route-direction authorization; raw lamp color is not used here.
  for (const int mission : {201351, 201365, 201290, 201326}) {
    SCOPED_TRACE(mission);
    DecisionLineLatch latch;
    EXPECT_TRUE(update(latch, Observation::AuthorizedGreen, false).committed_go);
  }
}

TEST(SignalDecisionLineLatch, FirstYellowOrRedChoosesStopEvenInsideBoundary)
{
  for (const auto observation : {Observation::Yellow, Observation::Stop}) {
    DecisionLineLatch latch;
    EXPECT_TRUE(update(latch, observation, true, 1).require_stop);
    EXPECT_EQ(latch.state(), DecisionLineState::Stop);
    EXPECT_TRUE(update(latch, observation, true, -.5, 2, 10.1).require_stop);
    EXPECT_TRUE(latch.pendingStop());
  }
}

TEST(SignalDecisionLineLatch, StopSurvivesVirtualOvershootUntilFreshDirectionalGrant)
{
  for (const double distance : {2.0, -.5}) {
    DecisionLineLatch latch;
    ASSERT_TRUE(update(latch, Observation::Stop, true, distance).require_stop);
    EXPECT_TRUE(update(latch, Observation::Unknown, false, distance, 0, 10.1, true).require_stop);
    EXPECT_TRUE(update(latch, Observation::AuthorizedGreen, true, distance, 0, 10.2, true).require_stop);
    EXPECT_TRUE(update(latch, Observation::AuthorizedGreen, false, distance, 0, 10.3, true, false).require_stop);
    const auto released = update(latch, Observation::AuthorizedGreen, false, distance, 0, 10.4, true);
    EXPECT_FALSE(released.require_stop);
    EXPECT_FALSE(released.committed_go);
    EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  }
}

TEST(SignalDecisionLineLatch, OneShotGoRetainsThroughLaterYellowRedAndMissingGeometry)
{
  DecisionLineLatch latch;
  ASSERT_TRUE(update(latch, Observation::AuthorizedGreen, false).committed_go);
  EXPECT_FALSE(update(latch, Observation::Yellow, true, 4, 10, 10.1).require_stop);
  EXPECT_FALSE(update(latch, Observation::Stop, true, 3, 10, 10.2).require_stop);
  EXPECT_FALSE(latch.update(Observation::Unknown, false, true, 10, false,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    0, 10.3).require_stop);
  EXPECT_TRUE(latch.lastDecision().committed_go);
}

TEST(SignalDecisionLineLatch, UnknownStaleMissingAndUnconfirmedGrantCannotCommit)
{
  for (const auto observation : {Observation::Unknown, Observation::Stale, Observation::Missing}) {
    DecisionLineLatch latch;
    EXPECT_FALSE(update(latch, observation, false).committed_go);
    EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  }
  DecisionLineLatch latch;
  EXPECT_FALSE(update(latch, Observation::AuthorizedGreen, true).committed_go);
  EXPECT_FALSE(update(latch, Observation::AuthorizedGreen, false, 5, 10, 10.1, false, false).committed_go);
}

TEST(SignalDecisionLineLatch, InvalidGeometryNeverInventsCommitButFreshGreenCanReleaseStop)
{
  DecisionLineLatch latch;
  ASSERT_TRUE(update(latch, Observation::Stop, true).require_stop);
  const auto result = latch.update(Observation::AuthorizedGreen, true, false, 10, false,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), 10.1, 10.1);
  EXPECT_FALSE(result.require_stop);
  EXPECT_FALSE(result.committed_go);
  EXPECT_EQ(result.state, DecisionLineState::Approach);
}

TEST(SignalDecisionLineLatch, StoppedVehicleNeverKeepsOrCreatesCommittedGo)
{
  DecisionLineLatch latch;
  EXPECT_FALSE(update(latch, Observation::AuthorizedGreen, false, -1, 0, 10, true).committed_go);
  ASSERT_TRUE(update(latch, Observation::AuthorizedGreen, false, 1, 10, 10.1).committed_go);
  EXPECT_TRUE(update(latch, Observation::Stop, true, -.5, 0, 10.2, true).require_stop);
  EXPECT_EQ(latch.state(), DecisionLineState::Stop);
  const auto released = update(latch, Observation::AuthorizedGreen, false, -.5, 0, 10.3, true);
  EXPECT_FALSE(released.require_stop);
  EXPECT_EQ(released.state, DecisionLineState::Approach);
}

TEST(SignalDecisionLineLatch, ClockOrSourceRewindRevokesGrantBeforeReassessment)
{
  for (const bool source_only : {false, true}) {
    DecisionLineLatch latch;
    ASSERT_TRUE(update(latch, Observation::AuthorizedGreen, false).committed_go);
    const auto rewound = update(latch, Observation::AuthorizedGreen, false,
      4, 10, 9, false, true, source_only ? 10.1 : 9);
    EXPECT_TRUE(rewound.require_stop);
    EXPECT_EQ(rewound.state, DecisionLineState::Stop);
    EXPECT_STREQ(rewound.reason, "clock_rewind");
    EXPECT_FALSE(update(latch, Observation::AuthorizedGreen, false, 4, 10,
      source_only ? 10.2 : 9.1).require_stop);
  }
}

TEST(SignalDecisionLineLatch, MissionResetDropsCommitAndPendingStop)
{
  DecisionLineLatch latch;
  ASSERT_TRUE(update(latch, Observation::AuthorizedGreen, false).committed_go);
  latch.reset();
  EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  EXPECT_FALSE(latch.lastDecision().committed_go);
  ASSERT_TRUE(update(latch, Observation::Stop, true).require_stop);
  latch.reset();
  EXPECT_FALSE(latch.pendingStop());
}

TEST(SignalDecisionLineLatch, ParkedNoiseDoesNotInventInitialUnknownStop)
{
  for (const double velocity : {-1e-5, -.05, -.1}) {
    DecisionLineLatch latch;
    const auto initial = update(latch, Observation::Unknown, false, 200, velocity, 10, true);
    EXPECT_FALSE(initial.require_stop);
    EXPECT_FALSE(initial.committed_go);
    EXPECT_FALSE(latch.pendingStop());
    EXPECT_EQ(latch.state(), DecisionLineState::Approach);
    ASSERT_TRUE(update(latch, Observation::Stop, true, 5, velocity, 10.1, true).require_stop);
    const auto release = update(latch, Observation::AuthorizedGreen, false, 5, velocity, 10.2, true);
    EXPECT_FALSE(release.require_stop);
    EXPECT_FALSE(release.committed_go);
    EXPECT_EQ(latch.state(), DecisionLineState::Approach);
  }
  DecisionLineLatch reverse;
  const auto invalid = reverse.update(Observation::Unknown, true, false, -.100001, true,
    200, 0, 10, 10);
  EXPECT_TRUE(invalid.require_stop);
  EXPECT_STREQ(invalid.reason, "invalid_velocity");
}
