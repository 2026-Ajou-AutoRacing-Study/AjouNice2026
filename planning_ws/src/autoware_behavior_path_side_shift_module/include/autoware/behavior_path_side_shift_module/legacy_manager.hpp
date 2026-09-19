#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>
#include <autoware/behavior_path_side_shift_module/scene_module.hpp>

namespace autoware::behavior_path_planner
{
class SideShiftSceneManager : public SceneModuleManagerInterface
{
public:
  explicit SideShiftSceneManager(LegacySideShiftParameters parameters = {})
  : SideShiftSceneManager(nullptr, std::move(parameters)) {}
  SideShiftSceneManager(ros::NodeHandle * node, LegacySideShiftParameters parameters = {})
  : SceneModuleManagerInterface("side_shift", node), parameters_(std::move(parameters)) {}
  void setLateralOffset(double offset) { lateral_offset_ = offset; }
  LegacySideShiftParameters parameters() const { return parameters_; }
  void setParameters(LegacySideShiftParameters parameters) { parameters_ = std::move(parameters); }

protected:
  std::unique_ptr<SceneModuleCore> createNewSceneModuleInstance() override
  {
    auto module = std::make_unique<SideShiftSceneModule>(
      parameters_, rtc_interfaces_, marker_interfaces_, planning_factor_interface_);
    module->setLateralOffset(lateral_offset_);
    return module;
  }

private:
  LegacySideShiftParameters parameters_;
  double lateral_offset_{0.0};
};

// BPP owns the source-name manager lifecycle.  Keep the scene-core manager
// reusable while exposing the ROS1 source contract expected by the BPP node.
class LegacySideShiftModuleManager : public SideShiftSceneManager
{
public:
  using SideShiftSceneManager::SideShiftSceneManager;
  bool init(ros::NodeHandle * node)
  {
    // Use the same candidate-local ROS1 provider path exercised by
    // SideShiftSceneManager tests.  Crossing the separately rebuilt common
    // façade here left the BPP host with an uninitialized manager state.
    if (!node) return false;
    node_ = node;
    // The source manager owns the scene parameter surface.  Read it before
    // the slot can create its idle module; otherwise a valid ROS1 launch
    // parameter is silently replaced by a zero-initialized candidate default.
    auto configured = parameters();
    node_->param("side_shift/time_to_start_shifting", configured.time_to_start_shifting,
                 configured.time_to_start_shifting);
    node_->param("side_shift/min_distance_to_start_shifting",
                 configured.min_distance_to_start_shifting,
                 configured.min_distance_to_start_shifting);
    node_->param("side_shift/shifting_lateral_jerk", configured.shifting_lateral_jerk,
                 configured.shifting_lateral_jerk);
    node_->param("side_shift/min_shifting_distance", configured.min_shifting_distance,
                 configured.min_shifting_distance);
    node_->param("side_shift/min_shifting_speed", configured.min_shifting_speed,
                 configured.min_shifting_speed);
    node_->param("side_shift/shift_request_time_limit", configured.shift_request_time_limit,
                 configured.shift_request_time_limit);
    node_->param("side_shift/drivable_area_left_bound_offset",
                 configured.drivable_area_left_bound_offset,
                 configured.drivable_area_left_bound_offset);
    node_->param("side_shift/drivable_area_right_bound_offset",
                 configured.drivable_area_right_bound_offset,
                 configured.drivable_area_right_bound_offset);
    setParameters(std::move(configured));
    return initProviders({"side_shift"});
  }
};
}  // namespace autoware::behavior_path_planner
