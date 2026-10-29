// Copyright 2026 AjouNice2026
// SPDX-License-Identifier: Apache-2.0

#include "go_after_stop_latch.hpp"
#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <gtest/gtest.h>
#include <limits>

namespace bvp = autoware::behavior_velocity_planner;
namespace
{
void observeStop(bvp::GoAfterStopLatch & latch, double start = 1.0, double x = 0.0)
{
  for (int tick = 0; tick <= 10; ++tick) {
    ASSERT_FALSE(latch.update(true, 0.0, false, x, 0.0, start + tick * 0.1));
  }
  ASSERT_TRUE(latch.stoppedObserved());
}
}  // namespace

TEST(GoAfterStopLatch, MovingGoDoesNotCommit)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 5.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.update(true, 0.02, true, 0.1, 0.0, 1.1));
  EXPECT_FALSE(latch.stoppedObserved());
}

TEST(GoAfterStopLatch, SingleStoppedTickDoesNotArm)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_FALSE(latch.update(true, 0.0, true, 0.0, 0.0, 1.1));
  EXPECT_FALSE(latch.update(true, 0.2, true, 0.01, 0.0, 1.2));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 0.0);
}

TEST(GoAfterStopLatch, RequiresOneFullSecondAndStillWaitsForGo)
{
  bvp::GoAfterStopLatch latch;
  for (int tick = 0; tick < 10; ++tick) {
    EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0 + tick * 0.1));
    EXPECT_FALSE(latch.stoppedObserved());
  }
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.99));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 2.0));
  EXPECT_TRUE(latch.stoppedObserved());
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 1.0);
  EXPECT_FALSE(latch.isLatched());
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
}

TEST(GoAfterStopLatch, MovementInterruptsUnfinishedHold)
{
  bvp::GoAfterStopLatch latch;
  latch.update(true, 0.0, false, 0.0, 0.0, 1.0);
  latch.update(true, 0.0, false, 0.0, 0.0, 1.4);
  latch.update(true, 0.02, false, 0.01, 0.0, 1.5);
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 0.0);
  for (double time : {1.6, 2.0, 2.4, 2.59}) {
    EXPECT_FALSE(latch.update(true, 0.0, false, 0.01, 0.0, time));
    EXPECT_FALSE(latch.stoppedObserved());
  }
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.01, 0.0, 2.61));
}

TEST(GoAfterStopLatch, EarlySafeGoMayFinishHoldOnlyWhileStillStationary)
{
  bvp::GoAfterStopLatch latch;
  latch.update(true, 0.0, false, 0.0, 0.0, 1.0);
  EXPECT_FALSE(latch.update(true, 0.0, true, 0.0, 0.0, 1.4));
  EXPECT_FALSE(latch.update(true, 0.0, true, 0.0, 0.0, 1.8));
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.0));
}

TEST(GoAfterStopLatch, ConfirmedStopRetainsGoDuringRenewedRisk)
{
  bvp::GoAfterStopLatch latch;
  observeStop(latch);
  EXPECT_TRUE(latch.update(true, 0.2, true, 0.01, 0.0, 2.1));
  EXPECT_TRUE(latch.update(true, 0.5, false, 0.04, 0.0, 2.2));
  EXPECT_TRUE(latch.update(true, 0.0, false, 0.05, 0.0, 2.3));
}

TEST(GoAfterStopLatch, InitiallySafeStationaryVehicleDoesNotStartHold)
{
  bvp::GoAfterStopLatch latch;
  for (int tick = 0; tick <= 20; ++tick) {
    EXPECT_FALSE(latch.update(true, 0.0, true, 0.0, 0.0, 1.0 + tick * 0.1));
  }
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 0.0);
}

TEST(GoAfterStopLatch, NumericalZeroUsesAbsoluteMeasuredVelocity)
{
  bvp::GoAfterStopLatch latch;
  for (int tick = 0; tick <= 10; ++tick) {
    EXPECT_FALSE(latch.update(true, -0.01, false, 0.0, 0.0, 1.0 + tick * 0.1));
  }
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
}

TEST(GoAfterStopLatch, ExistingCollisionHoldStillApplies)
{
  bvp::GoAfterStopLatch latch;
  observeStop(latch);
  bvp::StateMachine collision;
  collision.setMarginTime(0.1);
  collision.setState(bvp::StateMachine::State::STOP);
  const auto update = [&](double time, bool raw_collision) {
    collision.setStateWithMarginTime(
      raw_collision ? bvp::StateMachine::State::STOP : bvp::StateMachine::State::GO,
      ros::Time(time));
    return latch.update(true, 0.0,
      !raw_collision && collision.getState() == bvp::StateMachine::State::GO,
      0.0, 0.0, time);
  };
  EXPECT_FALSE(update(2.0, true));
  EXPECT_FALSE(update(2.05, false));
  EXPECT_FALSE(update(2.10, false));
  EXPECT_TRUE(update(2.16, false));
  EXPECT_TRUE(update(2.20, true));
  EXPECT_EQ(collision.getState(), bvp::StateMachine::State::STOP);
}

TEST(GoAfterStopLatch, DisabledPolicyAndNewEncounterDoNotInheritGo)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(false, 0.0, true, 0.0, 0.0, 1.0));
  observeStop(latch);
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
  EXPECT_FALSE(latch.update(false, 0.0, false, 0.0, 0.0, 2.2));
  EXPECT_FALSE(latch.update(true, 1.0, false, 0.0, 0.0, 2.3));
  bvp::GoAfterStopLatch next_roundabout;
  EXPECT_FALSE(next_roundabout.update(true, 1.0, true, 0.0, 0.0, 2.4));
}

TEST(GoAfterStopLatch, TeleportRequiresAnotherFullStopAndSafeGo)
{
  bvp::GoAfterStopLatch latch;
  observeStop(latch);
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
  EXPECT_FALSE(latch.update(true, 1.0, true, 8.0, 0.0, 2.2));
  EXPECT_FALSE(latch.stoppedObserved());
  observeStop(latch, 3.0, 8.0);
  EXPECT_TRUE(latch.update(true, 0.0, true, 8.0, 0.0, 4.1));
}

TEST(GoAfterStopLatch, ClockRewindResetsGoAndHold)
{
  bvp::GoAfterStopLatch latch;
  observeStop(latch);
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
  EXPECT_FALSE(latch.update(true, 1.0, false, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 0.0);
}

TEST(GoAfterStopLatch, NonfiniteMeasurementCannotRetainGo)
{
  bvp::GoAfterStopLatch latch;
  observeStop(latch);
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 2.1));
  EXPECT_FALSE(latch.update(true, std::numeric_limits<double>::quiet_NaN(),
    true, 0.0, 0.0, 2.2));
  EXPECT_FALSE(latch.stoppedObserved());
}

TEST(GoAfterStopLatch, ObservationGapDoesNotCountAsContinuousStop)
{
  bvp::GoAfterStopLatch latch;
  latch.update(true, 0.0, false, 0.0, 0.0, 1.0);
  latch.update(true, 0.0, false, 0.0, 0.0, 1.2);
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 2.0));
  EXPECT_DOUBLE_EQ(latch.stopDurationSec(), 0.0);
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 2.4));
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 2.8));
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 3.0));
}

TEST(GoAfterStopLatch, StopDurationIsConfigurable)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0, 0.5));
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.49, 0.5));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 1.5, 0.5));
}

TEST(GoAfterStopLatch, InvalidDurationCannotArm)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0, -1.0));
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 2.0,
    std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(latch.stoppedObserved());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
