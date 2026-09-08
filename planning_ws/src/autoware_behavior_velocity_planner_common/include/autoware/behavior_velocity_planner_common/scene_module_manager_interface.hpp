#pragma once

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_internal_debug_msgs/Float64Stamped.h>

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <type_traits>

namespace autoware::behavior_velocity_planner
{
namespace detail
{
template <class TStamp>
void assign_ros1_debug_stamp(TStamp & output, const ros::Time & input)
{
  if constexpr (std::is_assignable<TStamp &, ros::Time>::value) {
    output = input;
  } else {
    output.sec = input.sec;
    output.nanosec = input.nsec;
  }
}
}  // namespace detail

template <class T = SceneModuleInterface>
class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(ros::NodeHandle & node, const char * module_name)
  : node_(std::make_shared<ros::NodeHandle>(node)), module_name_(module_name)
  {
    pub_debug_ = node_->advertise<visualization_msgs::MarkerArray>("debug/" + module_name_, 1);
    pub_virtual_wall_ = node_->advertise<visualization_msgs::MarkerArray>("virtual_wall/" + module_name_, 5);
    node_->param("is_publish_debug_path", publish_debug_path_, false);
    if (publish_debug_path_) {
      pub_debug_path_ = node_->advertise<autoware_internal_planning_msgs::PathWithLaneId>(
        "debug/path_with_lane_id/" + module_name_, 1);
    }
    bool console_output = false;
    int throttle_duration_ms = 1000;
    // ROS2 parameter namespaces represented this hierarchy with dots.  Dots
    // are illegal in ROS1 graph resource names, so retain the hierarchy with
    // ROS1 namespace separators.
    node_->param("planning_factor_console_output/enable", console_output, false);
    node_->param("planning_factor_console_output/duration", throttle_duration_ms, 1000);
    planning_factor_interface_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      node_.get(), module_name_, console_output, throttle_duration_ms);
    debug_context_ = autoware_utils_debug::ros1::CallerNodeContext::borrow(node_);
    processing_time_publisher_ = std::make_shared<autoware_utils_debug::DebugPublisher>(debug_context_, "~/debug");
    pub_processing_time_detail_ = node_->advertise<autoware_utils_debug::ProcessingTimeDetail>(
      "debug/processing_time_detail_ms/" + module_name_, 1);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);
  }
  virtual ~SceneModuleManagerInterface() = default;
  virtual const char * getModuleName() = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() const = 0;

  void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> & planner_data,
    const autoware_internal_planning_msgs::PathWithLaneId & path)
  {
    planner_data_ = planner_data;
    launchNewModules(path);
    deleteExpiredModules(path);
  }
  virtual void plan(autoware_internal_planning_msgs::PathWithLaneId * path)
  {
    modifyPathVelocity(path);
  }

protected:
  virtual void modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId * path)
  {
    autoware_utils_debug::ScopedTimeTrack track("SceneModuleManagerInterface::modifyPathVelocity", *time_keeper_);
    const ros::WallTime started = ros::WallTime::now();
    visualization_msgs::MarkerArray markers;
    for (const auto & module : scene_modules_) {
      module->setPlannerData(planner_data_);
      module->modifyPathVelocity(path);
      auto next = module->createDebugMarkerArray();
      markers.markers.insert(markers.markers.end(), next.markers.begin(), next.markers.end());
      virtual_wall_marker_creator_.add_virtual_walls(module->createVirtualWalls());
    }
    planning_factor_interface_->publish();
    pub_debug_.publish(markers);
    if (publish_debug_path_) {
      autoware_internal_planning_msgs::PathWithLaneId debug_path;
      debug_path.header = path->header;
      debug_path.points = path->points;
      pub_debug_path_.publish(debug_path);
    }
    pub_virtual_wall_.publish(virtual_wall_marker_creator_.create_markers(ros::Time::now()));
    // The staged debug message retains a ROS2-compatible builtin time field;
    // construct it explicitly instead of assigning ros::Time through the
    // generic scalar helper.
    autoware_internal_debug_msgs::Float64Stamped processing_time;
    const ros::Time stamp = ros::Time::now();
    detail::assign_ros1_debug_stamp(processing_time.stamp, stamp);
    processing_time.data = (ros::WallTime::now() - started).toSec() * 1000.0;
    processing_time_publisher_->publish(
      std::string(getModuleName()) + "/processing_time_ms", processing_time);
  }

protected:
  virtual void launchNewModules(const autoware_internal_planning_msgs::PathWithLaneId & path) = 0;
  virtual std::function<bool(const std::shared_ptr<T> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::PathWithLaneId & path) = 0;
  bool isModuleRegistered(int64_t id) const { return registered_module_id_set_.count(id) != 0; }
  void registerModule(const std::shared_ptr<T> & module)
  {
    registered_module_id_set_.insert(module->getModuleId());
    scene_modules_.insert(module);
  }
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  std::shared_ptr<const PlannerData> planner_data_;
  // RTC-backed managers must publish a terminal RTC status before removing an
  // expired module.  Keep the default lifecycle exactly as before while
  // exposing the same extension point as the upstream manager contract.
  virtual void deleteExpiredModules(const autoware_internal_planning_msgs::PathWithLaneId & path)
  {
    const auto expired = getModuleExpiredFunction(path);
    for (auto it = scene_modules_.begin(); it != scene_modules_.end();) {
      if (expired(*it)) {
        registered_module_id_set_.erase((*it)->getModuleId());
        it = scene_modules_.erase(it);
      } else {
        ++it;
      }
    }
  }
  std::set<std::shared_ptr<T>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

protected:
  std::string module_name_;
  motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator_;
  bool publish_debug_path_{false};
  ros::Publisher pub_debug_;
  ros::Publisher pub_virtual_wall_;
  ros::Publisher pub_debug_path_;
  ros::Publisher pub_processing_time_detail_;
  autoware_utils_debug::ros1::CallerNodeContext::Ptr debug_context_;
  std::shared_ptr<autoware_utils_debug::DebugPublisher> processing_time_publisher_;
};
}  // namespace autoware::behavior_velocity_planner
