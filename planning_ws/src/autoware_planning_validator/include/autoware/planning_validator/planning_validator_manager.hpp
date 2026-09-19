#ifndef AUTOWARE_PLANNING_VALIDATOR_MANAGER_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_MANAGER_HPP_

#include "autoware/planning_validator/plugin_interface.hpp"

#include <pluginlib/class_loader.h>

#include <boost/shared_ptr.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
class PlanningValidatorManager
{
public:
  PlanningValidatorManager();
  bool load_plugin(ros::NodeHandle & private_nh, const std::string & class_name);
  bool unload_plugin(const std::string & class_name);
  autoware_planning_validator::PlanningValidatorStatus validate(
    const TrajectoryCheckerInput & input);
  size_t loaded_plugin_count() const;
  bool diagnostic_override_all_errors() const;
  int requested_invalid_trajectory_handling() const;

private:
  pluginlib::ClassLoader<PluginInterface> loader_;
  std::vector<boost::shared_ptr<PluginInterface>> plugins_;
};
}  // namespace autoware::planning_validator

#endif
