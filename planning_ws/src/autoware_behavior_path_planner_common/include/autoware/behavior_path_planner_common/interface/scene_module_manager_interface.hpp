#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_manager_ros1.hpp>

#include <string>
#include <vector>

namespace rclcpp { class Parameter; }

namespace autoware::behavior_path_planner
{
// Source-name ROS1 manager façade.  It preserves the upstream interface's
// explicit provider initialization and publication sequence while delegating
// lifecycle ownership to the executable ROS1 manager core.
class SceneModuleManagerInterface : public SceneModuleManagerRos1
{
public:
  explicit SceneModuleManagerInterface(std::string name, ros::NodeHandle * node = nullptr);

  bool init(ros::NodeHandle * node, const std::vector<std::string> & rtc_types, bool enable_rtc = false);
  // Dynamic BPP launchers only know a manager through this base type.  Keep a
  // virtual lifecycle hook so a provider can load its own ROS1 parameters
  // instead of requiring host-side downcasts or a synthetic parameter ABI.
  virtual bool initialize(ros::NodeHandle * node)
  {
    return init(node, {""});
  }
  // Legacy test managers remain constructible, but unsupported live updates
  // must be reported rather than silently discarded. Original real providers
  // override this hook with their source updateModuleParams implementation.
  virtual void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
  {
    if (!parameters.empty()) throw std::logic_error("BPP manager has no parameter update implementation: " + name());
  }
  void publishMarker() const { publishMarkers(); }
  void publishVirtualWall() const;
  std::vector<std::weak_ptr<SceneModuleInterface>> getSceneModuleObservers() const
  {
    std::vector<std::weak_ptr<SceneModuleInterface>> result;
    for (const auto & weak : observerSnapshot()) {
      const auto module = std::dynamic_pointer_cast<SceneModuleInterface>(weak.lock());
      result.emplace_back(module);
    }
    return result;
  }
  void publishRTCStatus() { SceneModuleManagerRos1::publishRTCStatus(); }
  void publishPlanningFactors() { SceneModuleManagerRos1::publish_planning_factors(); }
  void publishObjectsOfInterest() { SceneModuleManagerRos1::publishObjectsOfInterest(); }
  virtual bool isSimultaneousExecutableAsApprovedModule() const
  {
    bool enabled = false;
    if (node_) node_->getParam(name() + "/enable_simultaneous_execution_as_approved_module", enabled);
    return enabled;
  }
  virtual bool isSimultaneousExecutableAsCandidateModule() const
  {
    bool enabled = false;
    if (node_) node_->getParam(name() + "/enable_simultaneous_execution_as_candidate_module", enabled);
    return enabled;
  }
private:
  ros::Publisher virtual_wall_publisher_;
};
}  // namespace autoware::behavior_path_planner
