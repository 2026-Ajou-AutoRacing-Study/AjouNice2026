#include <autoware/scenario_selector/scenario_state.hpp>
namespace autoware::scenario_selector {
Scenario ScenarioState::select_by_position(const Inputs & in) const {
  if (current_ == Scenario::Empty) {
    if (in.in_lane && in.goal_in_lane) return Scenario::LaneDriving;
    if (in.in_parking_lot) return Scenario::Parking;
    return Scenario::LaneDriving;
  }
  if (current_ == Scenario::LaneDriving && in.in_parking_lot && !in.goal_in_lane) return Scenario::Parking;
  if (current_ == Scenario::Parking && in.parking_completed && in.in_lane) return Scenario::LaneDriving;
  return current_;
}
Scenario ScenarioState::update(const Inputs & in) {
  const auto previous = current_;
  if (current_ == Scenario::Empty || (in.near_trajectory_end && in.stopped)) current_ = select_by_position(in);
  if (in.enable_mode_switching && current_ == Scenario::LaneDriving) {
    if (!in.stopped || !in.autonomous || !in.in_parking_lot || in.goal_in_lane) lane_stop_started_s_.reset();
    else if (!lane_stop_started_s_) lane_stop_started_s_ = in.now_s;
    else if (in.now_s - *lane_stop_started_s_ > 5.0) current_ = Scenario::Parking;
  } else if (in.enable_mode_switching && current_ == Scenario::Parking) {
    if (!in.parking_trajectory_empty || !in.along_lane) empty_parking_started_s_.reset();
    else if (!empty_parking_started_s_) empty_parking_started_s_ = in.now_s;
    else if (in.now_s - *empty_parking_started_s_ > 3.0) current_ = Scenario::LaneDriving;
  }
  if (current_ != previous) { lane_stop_started_s_.reset(); empty_parking_started_s_.reset(); }
  return current_;
}
}
