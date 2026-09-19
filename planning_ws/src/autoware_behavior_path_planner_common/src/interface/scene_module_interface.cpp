#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>

namespace autoware::behavior_path_planner
{
SceneModuleInterface::SceneModuleInterface(
  std::string name, RTCInterfaceMap rtc_interfaces, MarkerInterfaceMap marker_interfaces,
  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface,
  const ModuleStatus initial)
: SceneModuleCore(std::move(name), initial),
  rtc_interfaces_(std::move(rtc_interfaces)),
  marker_interfaces_(std::move(marker_interfaces)),
  planning_factor_interface_(std::move(planning_factor_interface)),
  time_keeper_(std::make_shared<autoware_utils::TimeKeeper>()),
  rtc_interface_ptr_map_(rtc_interfaces_),
  objects_of_interest_marker_interface_ptr_map_(marker_interfaces_),
  uuid_map_(rtc_uuids_),
  clock_(std::make_shared<Ros1SceneModuleClock>())
{
  for (const auto & [module_name, interface] : rtc_interfaces_) {
    rtc_uuids_.emplace(module_name, autoware_utils::generate_uuid());
  }
}
}  // namespace autoware::behavior_path_planner
