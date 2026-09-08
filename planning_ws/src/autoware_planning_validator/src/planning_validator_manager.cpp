#include "autoware/planning_validator/planning_validator_manager.hpp"

#include <pluginlib/exceptions.hpp>

#include <algorithm>

namespace autoware::planning_validator
{
PlanningValidatorManager::PlanningValidatorManager()
: loader_("autoware_planning_validator", "autoware::planning_validator::PluginInterface")
{
}

bool PlanningValidatorManager::load_plugin(ros::NodeHandle & private_nh, const std::string & class_name)
{
  const auto existing = std::find_if(plugins_.begin(), plugins_.end(), [&](const auto & plugin) {
    return plugin->module_name() == class_name;
  });
  if (existing != plugins_.end()) return true;
  try {
    const auto plugin = loader_.createInstance(class_name);
    plugin->initialize(private_nh, class_name);
    plugins_.push_back(plugin);
    return true;
  } catch (const pluginlib::PluginlibException & exception) {
    ROS_ERROR_STREAM("planning validator plugin load failed for '" << class_name << "': " << exception.what());
    return false;
  }
}

bool PlanningValidatorManager::unload_plugin(const std::string & class_name)
{
  const auto first_removed = std::remove_if(plugins_.begin(), plugins_.end(), [&](const auto & plugin) {
    return plugin->module_name() == class_name;
  });
  if (first_removed == plugins_.end()) return false;
  plugins_.erase(first_removed, plugins_.end());
  return true;
}

autoware_planning_validator::PlanningValidatorStatus PlanningValidatorManager::validate(
  const TrajectoryCheckerInput & input)
{
  // R002 has no result for an uninstalled checker. Default false therefore
  // prevents a partial plugin set from masquerading as a full validation pass.
  autoware_planning_validator::PlanningValidatorStatus result;
  for (const auto & plugin : plugins_) plugin->validate(input, result);
  return result;
}

size_t PlanningValidatorManager::loaded_plugin_count() const { return plugins_.size(); }

bool PlanningValidatorManager::diagnostic_override_all_errors() const
{
  // Source grants this behavior only to the trajectory checker. Calling the
  // appended virtual on the still-independent non-trajectory candidate
  // plugins would require an unrelated ABI rebuild, so do not do it.
  return std::any_of(plugins_.begin(), plugins_.end(), [](const auto & plugin) {
    return plugin->module_name() == "autoware::planning_validator::TrajectoryCheckerRos1" &&
      plugin->diagnostic_override_all_errors();
  });
}

int PlanningValidatorManager::requested_invalid_trajectory_handling() const
{
  int requested = -1;
  for (const auto & plugin : plugins_) {
    if (plugin->module_name() != "autoware::planning_validator::TrajectoryCheckerRos1") continue;
    const int value = plugin->requested_invalid_trajectory_handling();
    if (value == -2) return -2;
    requested = std::max(requested, value);
  }
  return requested;
}
}  // namespace autoware::planning_validator
