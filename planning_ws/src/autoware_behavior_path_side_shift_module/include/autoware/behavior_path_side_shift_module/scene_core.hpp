#pragma once

#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
struct LegacySideShiftParameters
{
  double time_to_start_shifting{0.0};
  double min_distance_to_start_shifting{0.0};
  double shifting_lateral_jerk{1.0};
  double min_shifting_distance{0.0};
  double min_shifting_speed{0.0};
  double shift_request_time_limit{0.0};
  double drivable_area_left_bound_offset{0.0};
  double drivable_area_right_bound_offset{0.0};
  std::vector<std::string> drivable_area_types_to_skip{};
};

class SideShiftSceneCore
{
public:
  explicit SideShiftSceneCore(LegacySideShiftParameters parameters = {});
  void setReferencePath(const PathWithLaneId & path);
  void setCurrentLanelets(const lanelet::ConstLanelets & lanelets);
  void requestLateralOffset(double offset);
  BehaviorModuleOutput plan(size_t ego_index, double ego_speed);
  ShiftLineArray shiftLines() const { return path_shifter_.getShiftLines(); }

private:
  ShiftLine calcShiftLine(size_t ego_index, double ego_speed) const;
  LegacySideShiftParameters parameters_;
  PathWithLaneId reference_path_;
  lanelet::ConstLanelets current_lanelets_;
  double requested_lateral_offset_{0.0};
  PathShifter path_shifter_;
};
}  // namespace autoware::behavior_path_planner
