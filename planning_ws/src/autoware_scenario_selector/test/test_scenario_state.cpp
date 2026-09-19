#include <autoware/scenario_selector/scenario_state.hpp>
#include <gtest/gtest.h>
using autoware::scenario_selector::Inputs;
using autoware::scenario_selector::Scenario;
using autoware::scenario_selector::ScenarioState;
TEST(ScenarioState, InitialPositionSelectionFollowsSourcePriority) {
  ScenarioState state;
  EXPECT_EQ(state.update(Inputs{true, true}), Scenario::LaneDriving);
  state.reset(); Inputs parking; parking.in_parking_lot = true;
  EXPECT_EQ(state.update(parking), Scenario::Parking);
  state.reset(); EXPECT_EQ(state.update(Inputs{}), Scenario::LaneDriving);
}
TEST(ScenarioState, LaneDrivingModeSwitchRequiresContinuousFiveSecondStop) {
  ScenarioState state; EXPECT_EQ(state.update(Inputs{true, true}), Scenario::LaneDriving);
  Inputs in; in.in_parking_lot = true; in.stopped = true; in.autonomous = true; in.enable_mode_switching = true;
  in.now_s = 10.; EXPECT_EQ(state.update(in), Scenario::LaneDriving);
  in.now_s = 15.; EXPECT_EQ(state.update(in), Scenario::LaneDriving);
  in.now_s = 15.01; EXPECT_EQ(state.update(in), Scenario::Parking);
}
TEST(ScenarioState, ParkingReturnsAfterContinuousThreeSecondEmptyTrajectory) {
  ScenarioState state; Inputs initial; initial.in_parking_lot = true; EXPECT_EQ(state.update(initial), Scenario::Parking);
  Inputs in; in.enable_mode_switching = true; in.parking_trajectory_empty = true; in.along_lane = true;
  in.now_s = 1.; EXPECT_EQ(state.update(in), Scenario::Parking);
  in.now_s = 4.; EXPECT_EQ(state.update(in), Scenario::Parking);
  in.now_s = 4.01; EXPECT_EQ(state.update(in), Scenario::LaneDriving);
}
int main(int argc, char ** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
