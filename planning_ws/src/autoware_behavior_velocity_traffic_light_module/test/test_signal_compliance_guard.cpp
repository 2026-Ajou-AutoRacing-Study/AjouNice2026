// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>

namespace compliance = autoware::behavior_velocity_planner::signal_compliance;
using compliance::Observation;

void confirmGreen(compliance::Guard & guard, double last_source)
{
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, last_source-.2, last_source-.2, 1.));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, last_source-.1, last_source-.1, 1.));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, last_source, last_source, 1.));
}

TEST(SignalComplianceGuard, MissingStaleUnknownStopNeverAuthorizeMotion)
{
  for (const auto phase : {Observation::Missing, Observation::Stale, Observation::Unknown, Observation::Stop}) {
    compliance::Guard guard;
    confirmGreen(guard, 10.);
    EXPECT_TRUE(guard.observe(phase, 10.1, 10.1, 1.));
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_TRUE(guard.observe(Observation::Yellow, 10.2, 10.2, 1.));
    EXPECT_FALSE(guard.yellowTransitionEligible());
  }
}

TEST(SignalComplianceGuard, OnlyFreshDirectionAuthorizedGreenToYellowQualifies)
{
  compliance::Guard guard;
  EXPECT_TRUE(guard.observe(Observation::Yellow, 10., 10., 1.));
  EXPECT_FALSE(guard.yellowTransitionEligible());
  confirmGreen(guard, 11.);
  EXPECT_TRUE(guard.observe(Observation::Yellow, 11.1, 11.1, 1.));
  EXPECT_TRUE(guard.yellowTransitionEligible());
  EXPECT_TRUE(guard.observe(Observation::Stop, 11.2, 11.2, 1.));
  EXPECT_FALSE(guard.yellowTransitionEligible());
}

TEST(SignalComplianceGuard, YellowDurationIsUnknownNotAResettableThreeSecondTimer)
{
  for (const double duration : {.3, 1., 2.75, 3., 5., 10.}) {
    compliance::Guard guard;
    confirmGreen(guard, 10.);
    guard.observe(Observation::Yellow, 10.1, 10.1, 1.);
    for (double elapsed = .2; elapsed <= duration; elapsed += .1) {
      EXPECT_TRUE(guard.observe(Observation::Yellow, 10.+elapsed, 10.+elapsed, 1.));
      EXPECT_TRUE(guard.yellowTransitionEligible());
      // It remains STOP whenever the physical stopping distance permits it,
      // irrespective of the true/unknown yellow duration.
      EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, guard.yellowTransitionEligible(),
                                                       false, 5., 1., 10., 20.));
    }
    EXPECT_TRUE(guard.observe(Observation::Stop, 11.+duration, 11.+duration, 1.));
    EXPECT_FALSE(guard.yellowTransitionEligible());
  }
}

TEST(SignalComplianceGuard, FirstObservationInMiddleOfYellowNeverCreatesGo)
{
  for (const double first_observation : {10., 12.5, 20., 100.}) {
    compliance::Guard guard;
    guard.observe(Observation::Yellow, first_observation, first_observation, 1.);
    EXPECT_FALSE(guard.yellowTransitionEligible());
    EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, guard.yellowTransitionEligible(),
                                                    false, 10., 1., 50., 1.));
  }
}

TEST(SignalComplianceGuard, MissingDoesNotRefreshGreenLeaseOrYellowEligibility)
{
  compliance::Guard guard;
  confirmGreen(guard, 10.);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 11.01, 1.));
  guard.observe(Observation::Yellow, 11.1, 11.1, 1.);
  EXPECT_FALSE(guard.yellowTransitionEligible());
  confirmGreen(guard, 12.);
  guard.observe(Observation::Yellow, 12.1, 12.1, 1.);
  EXPECT_TRUE(guard.yellowTransitionEligible());
  guard.observe(Observation::Missing, 12.1, 12.2, 1.);
  EXPECT_FALSE(guard.yellowTransitionEligible());
}

TEST(SignalComplianceGuard, RepeatedOrRegressedSourceCannotFabricateGreenToYellowTransition)
{
  compliance::Guard guard;
  confirmGreen(guard, 10.);
  guard.observe(Observation::Yellow, 10., 10.1, 1.);
  EXPECT_FALSE(guard.yellowTransitionEligible());
  confirmGreen(guard, 11.);
  guard.observe(Observation::Yellow, 10.9, 11.1, 1.);
  EXPECT_FALSE(guard.yellowTransitionEligible());
  confirmGreen(guard, 20.);
  guard.observe(Observation::Yellow, 19.9, 19.9, 1.);
  EXPECT_FALSE(guard.yellowTransitionEligible());
}

TEST(SignalComplianceGuard, PassingNeedsMovingFreshYellowAndPhysicalInabilityToStop)
{
  EXPECT_TRUE(compliance::yellowPassthroughAllowed(true, true, false, 10., 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(false, true, false, 10., 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, false, false, 10., 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, true, 10., 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, false, 0., 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, false, .9, 1., 20., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, false, 10., 1., 9., 10.));
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, false, 10., 1.,
    std::numeric_limits<double>::quiet_NaN(), 10.));
}

TEST(SignalComplianceGuard, FrontAxleCrossingIsNotFrontBumperStopTarget)
{
  // Explicitly qualified rear-axle base_link; axle=3m and bumper=3.845m.
  auto front_distance = compliance::frontAxleDistanceFromBumperStop(0., 0., 3.845, 3.);
  ASSERT_TRUE(front_distance);
  EXPECT_DOUBLE_EQ(*front_distance, .845);
  EXPECT_GT(*compliance::frontAxleDistanceFromBumperStop(-.5, 0., 3.845, 3.), 0.);
  EXPECT_LT(*compliance::frontAxleDistanceFromBumperStop(-.9, 0., 3.845, 3.), 0.);
  EXPECT_FALSE(compliance::frontAxleDistanceFromBumperStop(0., 0., 3.845, -1.));
  EXPECT_FALSE(compliance::frontAxleDistanceFromBumperStop(0., 0., 3.845, 4.));
  EXPECT_FALSE(compliance::frontAxleDistanceFromBumperStop(0., -1., 3.845, 3.));
}

TEST(SignalComplianceGuard, CanStopBeforeFrontAxleLineEvenWhenBumperTargetCannotBeMet)
{
  const auto front_line = compliance::frontAxleDistanceFromBumperStop(.2, 0., 3.845, 3.);
  ASSERT_TRUE(front_line);
  EXPECT_NEAR(*front_line, 1.045, 1e-12);
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, true, false, 2., 1., .5, *front_line));
  // Counterfactual old mistake: comparing .5m against the .2m bumper target
  // would have allowed pass despite enough space before the official axle line.
  EXPECT_TRUE(compliance::yellowPassthroughAllowed(true, true, false, 2., 1., .5, .2));
}

TEST(SignalComplianceGuard, ThreeDistinctFreshSourcesNotRepeatedPlannerCallbacksGrantGreen)
{
  compliance::Guard guard;
  EXPECT_EQ(compliance::Guard::requiredGreenObservations, 3U);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  for (int i=0; i<20; ++i) {
    EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 10.05, 1.));
  }
  EXPECT_EQ(guard.greenConfirmationCount(), 1U);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10.1, 10.1, 1.));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10.2, 10.2, 1.));
  EXPECT_EQ(guard.greenConfirmationCount(), 3U);
}

TEST(SignalComplianceGuard, UnconfirmedSingleGreenCannotAuthorizeYellowDilemmaPassing)
{
  compliance::Guard guard;
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  EXPECT_TRUE(guard.observe(Observation::Yellow, 10.1, 10.1, 1.));
  EXPECT_EQ(guard.greenConfirmationCount(), 0U);
  EXPECT_FALSE(guard.yellowTransitionEligible());
  EXPECT_FALSE(compliance::yellowPassthroughAllowed(true, guard.yellowTransitionEligible(),
                                                  false, 10., 1., 50., 1.));
}

TEST(SignalComplianceGuard, InterruptionRequiresThreeNewGreenSourcesAgain)
{
  for (const auto interruption : {Observation::Stop, Observation::Yellow, Observation::Missing,
                                  Observation::Stale, Observation::Unknown}) {
    compliance::Guard guard;
    confirmGreen(guard, 10.);
    EXPECT_TRUE(guard.observe(interruption, 10.1, 10.1, 1.));
    EXPECT_EQ(guard.greenConfirmationCount(), 0U);
    confirmGreen(guard, 10.5);
  }
}

TEST(SignalComplianceGuard, StaleGapAndClockRollbackCannotCarryGreenConfirmations)
{
  compliance::Guard guard;
  confirmGreen(guard, 10.);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 11.2, 11.2, 1.));
  EXPECT_EQ(guard.greenConfirmationCount(), 1U);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 11.3, 11.3, 1.));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 11.4, 11.4, 1.));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 11.35, 11.5, 1.));
  EXPECT_EQ(guard.greenConfirmationCount(), 1U);
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 11.45, 11.55, 1.));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 11.55, 11.65, 1.));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 11., 11., 1.));
  EXPECT_EQ(guard.greenConfirmationCount(), 1U);
}

TEST(SignalComplianceUnknownGrace, DefaultZeroAndImplicitUnknownEligibilityNeverChangeStops)
{
  compliance::Guard guard;
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.05, 10.05, 1., false));
  EXPECT_FALSE(guard.unknownGraceActive());
  guard.setUnknownGraceSec(.2);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 11., 11., 1.));
  // Default has_color_evidence=true is deliberately fail-closed for old callers.
  EXPECT_TRUE(guard.observe(Observation::Unknown, 11.05, 11.05, 1.));
  EXPECT_FALSE(guard.unknownGraceActive());
}

TEST(SignalComplianceUnknownGrace, SetterIsFiniteBoundedAndConfigurationChangesResetGrant)
{
  compliance::Guard guard;
  for (const double invalid : {-.001, .300001, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(guard.setUnknownGraceSec(invalid), std::invalid_argument);
  }
  guard.setUnknownGraceSec(.3);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  guard.setUnknownGraceSec(.1);
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.05, 10.05, 1., false));
  guard.setUnknownGraceSec(0.);
}

TEST(SignalComplianceUnknownGrace, ThreeTenthsLeaseStillExpiresAndRealColorCannotBeHidden)
{
  for (const int count : {1,2}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.3);
    guard.setRequiredGreenObservations(count);
    for (int n=0; n<count; ++n) {
      const double source=10.-(count-1-n)*.01;
      EXPECT_EQ(guard.observe(Observation::AuthorizedGreen,source,source+.02,1.),n<count-1);
    }
    EXPECT_FALSE(guard.observe(Observation::Unknown,10.15,10.16,1.,false));
    EXPECT_FALSE(guard.observe(Observation::Unknown,10.28,10.29,1.,false));
    EXPECT_TRUE(guard.unknownGraceActive());
    EXPECT_TRUE(guard.observe(Observation::Unknown,10.30,10.301,1.,false));
    EXPECT_FALSE(guard.unknownGraceActive());
  }
  // Fresh UNKNOWN receipt cannot refresh the actual green's older source.
  compliance::Guard aged;
  aged.setUnknownGraceSec(.3);
  aged.setRequiredGreenObservations(1);
  EXPECT_FALSE(aged.observe(Observation::AuthorizedGreen,10.,10.25,1.));
  EXPECT_TRUE(aged.observe(Observation::Unknown,10.30,10.31,1.,false));
  for (const auto phase : {Observation::Stop,Observation::Yellow,Observation::Unknown}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.3);
    guard.setRequiredGreenObservations(1);
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen,10.,10.,1.));
    EXPECT_FALSE(guard.observe(Observation::Unknown,10.1,10.1,1.,false));
    EXPECT_TRUE(guard.observe(phase,10.2,10.2,1.,true));
    EXPECT_FALSE(guard.unknownGraceActive());
    EXPECT_TRUE(guard.observe(Observation::Unknown,10.21,10.21,1.,false));
  }
}

TEST(SignalComplianceUnknownGrace, RepeatedUnknownCannotExtendOriginalSourceOrProcessingLease)
{
  for (const int count : {1, 2, 3}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(count);
    for (int i=0; i<count; ++i) {
      const double source=10.+i*.01;
      EXPECT_EQ(guard.observe(Observation::AuthorizedGreen, source, source+.03, 1.), i<count-1);
    }
    const double last=10.+(count-1)*.01;
    EXPECT_FALSE(guard.observe(Observation::Unknown, last+.1, last+.11, 1., false));
    EXPECT_TRUE(guard.unknownGraceActive());
    EXPECT_EQ(guard.greenConfirmationCount(), static_cast<std::size_t>(count));
    EXPECT_FALSE(guard.observe(Observation::Unknown, last+.15, last+.18, 1., false));
    EXPECT_TRUE(guard.unknownGraceActive());
    EXPECT_TRUE(guard.observe(Observation::Unknown, last+.19, last+.201, 1., false));
    EXPECT_FALSE(guard.unknownGraceActive());
    EXPECT_EQ(guard.greenConfirmationCount(), 0U);
  }
}

TEST(SignalComplianceUnknownGrace, OldRealGreenIsNotMadeFreshByCurrentUnknownImage)
{
  compliance::Guard guard;
  guard.setUnknownGraceSec(.2);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10.15, 1.));
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.21, 10.22, 1., false));
  EXPECT_FALSE(guard.unknownGraceActive());
}

TEST(SignalComplianceUnknownGrace, DuplicateRealGreenDoesNotRenewProcessingLease)
{
  compliance::Guard guard;
  guard.setUnknownGraceSec(.2);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10.19, 1.));
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.20, 10.21, 1., false));
  EXPECT_FALSE(guard.unknownGraceActive());
}

TEST(SignalComplianceUnknownGrace, UnconfirmedGrantNeverGetsUnknownGrace)
{
  for (const int count : {2, 3}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(count);
    EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
    EXPECT_TRUE(guard.observe(Observation::Unknown, 10.05, 10.05, 1., false));
    EXPECT_EQ(guard.greenConfirmationCount(), 0U);
    for (int i=0; i<count; ++i) {
      const double source=10.1+i*.01;
      EXPECT_EQ(guard.observe(Observation::AuthorizedGreen, source, source, 1.), i<count-1);
    }
  }
}

TEST(SignalComplianceUnknownGrace, NewStopYellowOrKnownColorUnknownIsImmediate)
{
  for (const auto phase : {Observation::Stop, Observation::Yellow, Observation::Unknown}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(1);
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
    EXPECT_FALSE(guard.observe(Observation::Unknown, 10.05, 10.05, 1., false));
    EXPECT_TRUE(guard.observe(phase, 10.1, 10.1, 1., true));
    EXPECT_FALSE(guard.unknownGraceActive());
    EXPECT_TRUE(guard.observe(Observation::Unknown, 10.12, 10.12, 1., false));
  }
}

TEST(SignalComplianceUnknownGrace, MissingStaleInvalidSourceAndResetCannotReuseGrant)
{
  for (const auto phase : {Observation::Missing, Observation::Stale}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(1);
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
    EXPECT_TRUE(guard.observe(phase, 10.05, 10.05, 1., false));
    EXPECT_TRUE(guard.observe(Observation::Unknown, 10.1, 10.1, 1., false));
  }
  for (const double source : {0., 9., 10.2, std::numeric_limits<double>::quiet_NaN()}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(1);
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
    EXPECT_TRUE(guard.observe(Observation::Unknown, source, 10.1, 1., false));
    EXPECT_TRUE(guard.observe(Observation::Unknown, 10.15, 10.15, 1., false));
  }
  compliance::Guard guard;
  guard.setUnknownGraceSec(.2);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  guard.reset();
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.1, 10.1, 1., false));
}

TEST(SignalComplianceUnknownGrace, UnknownHighwaterRejectsOldGreenAndDuplicateCannotRegrant)
{
  compliance::Guard guard;
  guard.setUnknownGraceSec(.2);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  EXPECT_FALSE(guard.observe(Observation::Unknown, 10.1, 10.1, 1., false));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10.05, 10.12, 1.));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10.1, 10.13, 1.));
  EXPECT_TRUE(guard.observe(Observation::Unknown, 10.14, 10.14, 1., false));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10.15, 10.15, 1.));
}

TEST(SignalComplianceUnknownGrace, ClockReversalMustStopEvenWithOneConfirmation)
{
  compliance::Guard guard;
  guard.setUnknownGraceSec(.2);
  guard.setRequiredGreenObservations(1);
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
  EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 9., 9., 1.));
  EXPECT_TRUE(guard.observe(Observation::Unknown, 9.05, 9.05, 1., false));
  EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 9.1, 9.1, 1.));
}

TEST(SignalComplianceUnknownGrace, UnknownDoesNotMoveYellowOnsetBoundAndRealRecoveryDoes)
{
  for (const bool recover : {false, true}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(2);
    EXPECT_TRUE(guard.observe(Observation::AuthorizedGreen, 10., 10., 1.));
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10.02, 10.02, 1.));
    EXPECT_FALSE(guard.observe(Observation::Unknown, 10.1, 10.1, 1., false));
    EXPECT_FALSE(guard.yellowStartLowerBoundSec());
    if (recover) EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10.12, 10.12, 1.));
    EXPECT_TRUE(guard.observe(Observation::Yellow, 10.15, 10.15, 1.));
    EXPECT_TRUE(guard.yellowTransitionEligible());
    ASSERT_TRUE(guard.yellowStartLowerBoundSec());
    EXPECT_DOUBLE_EQ(*guard.yellowStartLowerBoundSec(), recover ? 10.12 : 10.02);
    EXPECT_TRUE(guard.observe(Observation::Yellow, 10.2, 10.2, 1.));
    EXPECT_DOUBLE_EQ(*guard.yellowStartLowerBoundSec(), recover ? 10.12 : 10.02);
  }
}

TEST(SignalComplianceUnknownGrace, DuplicateUnknownSourceReclassifiedGreenCannotMoveYellowBound)
{
  for (const int count : {1,2}) {
    compliance::Guard guard;
    guard.setUnknownGraceSec(.2);
    guard.setRequiredGreenObservations(count);
    for (int n=0; n<count; ++n) {
      const double source=10.-(count-1-n)*.01;
      EXPECT_EQ(guard.observe(Observation::AuthorizedGreen, source, source, 1.),n<count-1);
    }
    EXPECT_FALSE(guard.observe(Observation::Unknown, 10.05, 10.05, 1., false));
    // Same original image now claims green: it may not supply a new source,
    // confirmation, processing lease or a later lower bound for yellow onset.
    EXPECT_FALSE(guard.observe(Observation::AuthorizedGreen, 10.05, 10.06, 1.));
    EXPECT_EQ(guard.greenConfirmationCount(),static_cast<std::size_t>(count));
    ASSERT_TRUE(guard.lastRealGreenSourceSec());
    EXPECT_DOUBLE_EQ(*guard.lastRealGreenSourceSec(),10.);
    ASSERT_TRUE(guard.lastRealGreenProcessedSec());
    EXPECT_DOUBLE_EQ(*guard.lastRealGreenProcessedSec(),10.);
    EXPECT_TRUE(guard.observe(Observation::Yellow,10.1,10.1,1.));
    EXPECT_TRUE(guard.yellowTransitionEligible());
    ASSERT_TRUE(guard.yellowStartLowerBoundSec());
    EXPECT_DOUBLE_EQ(*guard.yellowStartLowerBoundSec(),10.);
  }
}

namespace
{
// Independent time-step oracle: integrate acceleration and velocity directly,
// without the quadratic root or piecewise displacement formula used by production.
// Like the legacy judge, the delay term assumes constant velocity during delay.
double integratedStoppingDistance(double velocity, double acceleration, double limit,
                                  double jerk, double delay)
{
  double distance=velocity*delay;
  constexpr double dt=1e-5;
  for(int step=0;velocity>0. && step<1000000;++step) {
    const double next_acceleration=std::max(limit,acceleration+jerk*dt);
    const double next_velocity=velocity+(acceleration+next_acceleration)*dt/2.;
    if(next_velocity<=0.) {
      const double partial=dt*velocity/(velocity-next_velocity);
      return distance+velocity*partial/2.;
    }
    distance+=(velocity+next_velocity)*dt/2.;
    velocity=next_velocity;
    acceleration=next_acceleration;
  }
  EXPECT_LE(velocity,0.);
  return distance;
}
}  // namespace

TEST(SignalComplianceBraking, TriangularDistanceMatchesIndependentTimeStepOracle)
{
  for(const double acceleration:{-1.,0.,1.}) {
    for(const double velocity:{.01,.5,1.,1.5,1.74}) {
      for(const double delay:{0.,.5}) {
        SCOPED_TRACE(acceleration);
        SCOPED_TRACE(velocity);
        SCOPED_TRACE(delay);
        const auto actual=compliance::candidateStoppingDistance(
          velocity,acceleration,-6.,-10.,delay,0.);
        ASSERT_TRUE(actual);
        EXPECT_NEAR(*actual,integratedStoppingDistance(velocity,acceleration,-6.,-10.,delay),1e-6);
      }
    }
  }
  EXPECT_NEAR(*compliance::candidateStoppingDistance(1.5,0.,-6.,-10.,.5,0.),
              1.297722557505166,1e-12);
}

TEST(SignalComplianceBraking, TriangularTrapezoidalBoundaryIsContinuousAndMonotone)
{
  for(const double acceleration:{-1.,0.,1.}) {
    const double boundary=(36.-acceleration*acceleration)/20.;
    double previous=-1.;
    for(const double delta:{-1e-5,0.,1e-5}) {
      const double velocity=boundary+delta;
      const double oracle=integratedStoppingDistance(velocity,acceleration,-6.,-10.,.5);
      const auto actual=compliance::candidateStoppingDistance(
        velocity,acceleration,-6.,-10.,.5,oracle);
      ASSERT_TRUE(actual);
      EXPECT_NEAR(*actual,oracle,1e-6);
      EXPECT_GT(*actual,previous);
      if(previous>=0.) {
        EXPECT_LT(*actual-previous,1e-4);
      }
      previous=*actual;
    }
  }
  EXPECT_NEAR(*compliance::candidateStoppingDistance(1.8,0.,-6.,-10.,.5,0.),1.62,1e-12);
}

TEST(SignalComplianceBraking, TrapezoidalBranchReturnsExactLegacyValueOnly)
{
  // A sentinel demonstrates that the helper does not replace/recompute the
  // common library's non-triangular branch, even to improve its approximation.
  for(const double velocity:{1.80001,2.,5.,20.}) {
    const auto actual=compliance::candidateStoppingDistance(velocity,0.,-6.,-10.,.5,123.456);
    ASSERT_TRUE(actual);
    EXPECT_DOUBLE_EQ(*actual,123.456);
  }
}

TEST(SignalComplianceBraking, InvalidBrakingInputsCannotAuthorizeYellowPass)
{
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for(const auto invalid:{nan,inf}) {
    EXPECT_FALSE(compliance::candidateStoppingDistance(invalid,0.,-6.,-10.,.5,1.));
    EXPECT_FALSE(compliance::candidateStoppingDistance(1.,invalid,-6.,-10.,.5,1.));
    EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,invalid,-10.,.5,1.));
    EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,invalid,.5,1.));
    EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,-10.,invalid,1.));
    EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,-10.,.5,invalid));
  }
  EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,0.,-10.,.5,1.));
  EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,0.,.5,1.));
  EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,-10.,-.1,1.));
  EXPECT_FALSE(compliance::candidateStoppingDistance(1.,0.,-6.,-10.,.5,-1.));
  EXPECT_DOUBLE_EQ(*compliance::candidateStoppingDistance(0.,0.,-6.,-10.,.5,0.),0.);
}

TEST(SignalLaneletYellow, SimpleDistanceDelayAndStrictBoundary)
{
  const auto exact=compliance::laneletYellowDecision(true,true,false,8.,1.,.5,8.,8.);
  EXPECT_FALSE(exact.allow);
  EXPECT_DOUBLE_EQ(exact.stopping_distance_m,8.);
  EXPECT_DOUBLE_EQ(exact.stopping_target_distance_m,8.);
  EXPECT_STREQ(exact.reason,"lanelet_target_stoppable");
  EXPECT_TRUE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,8.,7.999).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,8.,8.001).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,0.,8.,7.).allow);
  EXPECT_TRUE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,8.,7.).allow);
  EXPECT_TRUE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,4.,10.).allow);
  // Signed target may be behind ego, while the independent original crossing
  // line remains ahead. Never reinterpret it as a negative invalid line ETA.
  EXPECT_TRUE(compliance::laneletYellowDecision(true,true,false,2.,1.,.5,8.,-.1).allow);
}

TEST(SignalLaneletYellow, EligibilityAndStoppedDepartureCannotBeBypassed)
{
  EXPECT_FALSE(compliance::laneletYellowDecision(false,true,false,8.,1.,.5,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,false,false,8.,1.,.5,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,true,8.,1.,.5,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,.99,1.,.5,8.,-.1).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,0.,1.,.5,8.,-.1).allow);
}

TEST(SignalLaneletYellow, InvalidAndNonfiniteInputsFailClosed)
{
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  for (const double invalid : {nan,inf,-inf}) {
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,invalid,1.,.5,8.,1.).allow);
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,invalid,.5,8.,1.).allow);
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,invalid,8.,1.).allow);
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,invalid,1.).allow);
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,8.,invalid).allow);
  }
  for (const double invalid : {0.,-8.})
    EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,.5,invalid,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,-1.,1.,.5,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,0.,.5,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,8.,1.,-.1,8.,1.).allow);
  EXPECT_FALSE(compliance::laneletYellowDecision(true,true,false,1e308,1.,.5,8.,1.).allow);
}
