#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>

#include <gtest/gtest.h>

using StateMachine = autoware::behavior_velocity_planner::StateMachine;
using State = StateMachine::State;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_state_machine");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(StateMachine, InitializedGo)
{
  EXPECT_EQ(State::GO, StateMachine().getState());
}

TEST(StateMachine, SetStop)
{
  StateMachine state_machine;
  state_machine.setState(State::STOP);
  EXPECT_EQ(State::STOP, state_machine.getState());
}

TEST(StateMachine, StopIsImmediateWithMargin)
{
  StateMachine state_machine;
  state_machine.setMarginTime(1.0);
  state_machine.setStateWithMarginTime(State::STOP, ros::Time(100.0));
  EXPECT_EQ(State::STOP, state_machine.getState());
}

TEST(StateMachine, GoRequiresStrictlyMoreThanMargin)
{
  StateMachine state_machine;
  state_machine.setMarginTime(0.2);
  state_machine.setState(State::STOP);

  state_machine.setStateWithMarginTime(State::GO, ros::Time(10.0));
  EXPECT_EQ(State::STOP, state_machine.getState());
  EXPECT_DOUBLE_EQ(0.0, state_machine.getDuration());

  state_machine.setStateWithMarginTime(State::GO, ros::Time(10.2));
  EXPECT_EQ(State::STOP, state_machine.getState());
  EXPECT_DOUBLE_EQ(0.2, state_machine.getDuration());

  state_machine.setStateWithMarginTime(State::GO, ros::Time(10.201));
  EXPECT_EQ(State::GO, state_machine.getState());
  EXPECT_DOUBLE_EQ(0.201, state_machine.getDuration());
}
