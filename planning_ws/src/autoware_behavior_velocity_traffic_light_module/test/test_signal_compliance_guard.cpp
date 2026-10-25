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
