// Copyright 2026 AjouNice2026
// SPDX-License-Identifier: Apache-2.0

#include "go_after_stop_latch.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <gtest/gtest.h>

#include <limits>

namespace bvp = autoware::behavior_velocity_planner;

TEST(GoAfterStopLatch, MovingGoDoesNotCommit)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 5.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.update(true, 0.02, true, 0.1, 0.0, 1.1));
  EXPECT_FALSE(latch.stoppedObserved());
}

TEST(GoAfterStopLatch, StopArmsButDoesNotOverrideCollisionBeforeFirstGo)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0));
  EXPECT_TRUE(latch.stoppedObserved());
  EXPECT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.1));
  EXPECT_FALSE(latch.isLatched());
}

TEST(GoAfterStopLatch, FirstGoAfterStopCommitsEvenIfEgoAlreadyStartedMoving)
{
  bvp::GoAfterStopLatch latch;
  ASSERT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.0));
  EXPECT_TRUE(latch.update(true, 0.2, true, 0.01, 0.0, 1.1));
  EXPECT_TRUE(latch.update(true, 0.5, false, 0.04, 0.0, 1.2));
  EXPECT_TRUE(latch.update(true, 0.0, false, 0.05, 0.0, 1.3));
}

TEST(GoAfterStopLatch, InitiallyStationarySafeGoDoesNotCommitBeforeRoundaboutStop)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, 0.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_FALSE(latch.update(true, 1.0, false, 0.1, 0.0, 1.1));
}

TEST(GoAfterStopLatch, NumericalZeroUsesAbsoluteMeasuredVelocity)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(true, -0.1, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.update(true, -0.01, false, 0.0, 0.0, 1.1));
  EXPECT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 1.2));
}

TEST(GoAfterStopLatch, WaitsForExistingStopToGoHoldBeforeCommitting)
{
  bvp::GoAfterStopLatch latch;
  bvp::StateMachine collision;
  collision.setMarginTime(0.1);
  collision.setState(bvp::StateMachine::State::STOP);
  const auto update = [&](double time, bool raw_collision) {
    collision.setStateWithMarginTime(
      raw_collision ? bvp::StateMachine::State::STOP : bvp::StateMachine::State::GO,
      ros::Time(time));
    return latch.update(
      true, 0.0, !raw_collision && collision.getState() == bvp::StateMachine::State::GO,
      0.0, 0.0, time);
  };
  EXPECT_FALSE(update(1.0, true));
  EXPECT_FALSE(update(1.05, false));
  EXPECT_FALSE(update(1.10, false));
  EXPECT_TRUE(update(1.16, false));
  // A real STOP request still reaches the detector, while the experiment retains GO.
  EXPECT_TRUE(update(1.20, true));
  EXPECT_EQ(collision.getState(), bvp::StateMachine::State::STOP);
}

TEST(GoAfterStopLatch, DisabledPolicyAndNewEncounterDoNotInheritGo)
{
  bvp::GoAfterStopLatch latch;
  EXPECT_FALSE(latch.update(false, 0.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.stoppedObserved());
  ASSERT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 1.05));
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 1.1));
  EXPECT_FALSE(latch.update(false, 0.0, false, 0.0, 0.0, 1.2));
  EXPECT_FALSE(latch.update(true, 1.0, false, 0.0, 0.0, 1.3));
  bvp::GoAfterStopLatch next_roundabout;
  EXPECT_FALSE(next_roundabout.update(true, 1.0, true, 0.0, 0.0, 1.4));
}

TEST(GoAfterStopLatch, TeleportResetsGoAndRequiresAnotherStopAndSafeGo)
{
  bvp::GoAfterStopLatch latch;
  ASSERT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 0.9));
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.update(true, 1.0, true, 8.0, 0.0, 1.1));
  EXPECT_FALSE(latch.stoppedObserved());
  EXPECT_FALSE(latch.update(true, 0.0, false, 8.0, 0.0, 1.2));
  EXPECT_TRUE(latch.update(true, 0.0, true, 8.0, 0.0, 1.3));
}

TEST(GoAfterStopLatch, ClockRewindResetsGo)
{
  bvp::GoAfterStopLatch latch;
  ASSERT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 9.9));
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 10.0));
  EXPECT_FALSE(latch.update(true, 1.0, false, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.stoppedObserved());
}

TEST(GoAfterStopLatch, NonfiniteMeasurementCannotArmOrRetainGo)
{
  bvp::GoAfterStopLatch latch;
  ASSERT_FALSE(latch.update(true, 0.0, false, 0.0, 0.0, 0.9));
  ASSERT_TRUE(latch.update(true, 0.0, true, 0.0, 0.0, 1.0));
  EXPECT_FALSE(latch.update(
    true, std::numeric_limits<double>::quiet_NaN(), true, 0.0, 0.0, 1.1));
  EXPECT_FALSE(latch.stoppedObserved());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
