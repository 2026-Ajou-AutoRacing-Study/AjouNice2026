#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>
#include <autoware/behavior_path_side_shift_module/scene_core.hpp>
#include <ros/time.h>

#include <memory>
#include <optional>

namespace autoware::behavior_path_planner
{
class SideShiftSceneModule : public SceneModuleInterface
{
public:
  explicit SideShiftSceneModule(LegacySideShiftParameters parameters = {});
  SideShiftSceneModule(
    LegacySideShiftParameters parameters, RTCInterfaceMap rtc_interfaces,
    MarkerInterfaceMap marker_interfaces,
    std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface);
  void setLateralOffset(double offset);
  std::optional<ShiftLine> latestShiftLine() const;

protected:
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override { return true; }
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }
  void updateData() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput plan() override;

private:
  bool isShifting(const size_t ego_index) const;
  LegacySideShiftParameters parameters_;
  SideShiftSceneCore scene_;
  double requested_lateral_offset_{0.0};
  bool lateral_offset_change_request_{false};
  std::optional<ros::Time> latest_lateral_offset_stamp_{};
  std::optional<ShiftLine> latest_shift_line_{};
  BehaviorModuleOutput previous_shifted_output_{};
};
}  // namespace autoware::behavior_path_planner
