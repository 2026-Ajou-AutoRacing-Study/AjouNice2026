#pragma once
#include <optional>

namespace autoware::scenario_selector {
enum class Scenario { Empty, LaneDriving, Parking };
struct Inputs {
  bool in_lane{false};
  bool goal_in_lane{false};
  bool in_parking_lot{false};
  bool parking_completed{false};
  bool near_trajectory_end{false};
  bool stopped{false};
  bool autonomous{false};
  bool parking_trajectory_empty{false};
  bool along_lane{false};
  bool enable_mode_switching{false};
  double now_s{0.0};
};
class ScenarioState {
public:
  Scenario current() const { return current_; }
  void reset() { current_ = Scenario::Empty; lane_stop_started_s_.reset(); empty_parking_started_s_.reset(); }
  Scenario update(const Inputs &);
private:
  Scenario select_by_position(const Inputs &) const;
  Scenario current_{Scenario::Empty};
  std::optional<double> lane_stop_started_s_;
  std::optional<double> empty_parking_started_s_;
};
}
