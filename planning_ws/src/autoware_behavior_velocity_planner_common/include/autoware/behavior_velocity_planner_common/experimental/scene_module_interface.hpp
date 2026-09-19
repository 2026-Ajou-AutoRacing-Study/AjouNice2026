#pragma once

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/objects_of_interest_marker_interface/marker_data.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <functional>
#include <cstdarg>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
struct ObjectOfInterest
{
  geometry_msgs::Pose pose;
  autoware_perception_msgs::Shape shape;
  objects_of_interest_marker_interface::ColorName color;
};

class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    int64_t module_id, const ros::NodeHandle & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
  : module_id_(module_id), node_(node), time_keeper_(std::move(time_keeper)),
    planning_factor_interface_(std::move(planning_factor_interface))
  {
  }
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data) = 0;
  virtual visualization_msgs::MarkerArray createDebugMarkerArray() = 0;
  virtual motion_utils::VirtualWalls createVirtualWalls() = 0;
  int64_t getModuleId() const { return module_id_; }
  std::vector<ObjectOfInterest> getObjectsOfInterestData() const { return objects_of_interest_; }
  void clearObjectsOfInterestData() { objects_of_interest_.clear(); }

protected:
  const int64_t module_id_;
  ros::NodeHandle node_;
  std::vector<ObjectOfInterest> objects_of_interest_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;

  void setObjectsOfInterestData(
    const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
    objects_of_interest_marker_interface::ColorName color)
  {
    objects_of_interest_.push_back({pose, shape, color});
  }
  size_t findEgoSegmentIndex(
    const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points,
    const PlannerData & planner_data) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, planner_data.current_odometry->pose, planner_data.ego_nearest_dist_threshold,
      planner_data.ego_nearest_yaw_threshold);
  }
  virtual std::vector<int64_t> getRegulatoryElementIds() const { return {}; }
  virtual std::vector<int64_t> getLaneletIds() const { return {}; }
  virtual std::vector<int64_t> getLineIds() const { return {}; }
  void logInfo(const char * format, ...) const;
  void logWarn(const char * format, ...) const;
  void logDebug(const char * format, ...) const;
  void logInfoThrottle(int duration_ms, const char * format, ...) const;
  void logWarnThrottle(int duration_ms, const char * format, ...) const;
  void logDebugThrottle(int duration_ms, const char * format, ...) const;

private:
  std::string formatLogMessage(const char * format, va_list args) const;
};

template <class T = SceneModuleInterface>
class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(ros::NodeHandle & node, const char * module_name)
  : node_(node), module_name_(module_name)
  {
    pub_debug_ = node_.advertise<visualization_msgs::MarkerArray>("debug/" + module_name_, 1);
    pub_virtual_wall_ = node_.advertise<visualization_msgs::MarkerArray>("virtual_wall/" + module_name_, 5);
    bool console_output = false;
    int throttle_duration_ms = 1000;
    node_.param("planning_factor_console_output/enable", console_output, false);
    node_.param("planning_factor_console_output/duration", throttle_duration_ms, 1000);
    planning_factor_interface_ = std::make_shared<planning_factor_interface::PlanningFactorInterface>(
      &node_, module_name_, console_output, throttle_duration_ms);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
  }
  virtual ~SceneModuleManagerInterface() = default;
  virtual const char * getModuleName() = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() const = 0;

  void updateSceneModuleInstances(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data)
  {
    launchNewModules(path, stamp, planner_data);
    deleteExpiredModules(path, planner_data);
  }
  virtual void plan(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data)
  {
    modifyPathVelocity(path, header, left_bound, right_bound, planner_data);
  }

protected:
  virtual void modifyPathVelocity(
    Trajectory & path, const std_msgs::Header & header,
    const std::vector<geometry_msgs::Point> & left_bound,
    const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data)
  {
    visualization_msgs::MarkerArray all_markers;
    for (const auto & module : scene_modules_) {
      module->modifyPathVelocity(path, left_bound, right_bound, planner_data);
      const auto markers = module->createDebugMarkerArray();
      all_markers.markers.insert(all_markers.markers.end(), markers.markers.begin(), markers.markers.end());
      virtual_wall_marker_creator_.add_virtual_walls(module->createVirtualWalls());
    }
    planning_factor_interface_->publish();
    pub_debug_.publish(all_markers);
    pub_virtual_wall_.publish(virtual_wall_marker_creator_.create_markers(header.stamp));
  }

protected:
  virtual void launchNewModules(
    const Trajectory & path, const ros::Time & stamp, const PlannerData & planner_data) = 0;
  virtual std::function<bool(const std::shared_ptr<T> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) = 0;
  bool isModuleRegistered(int64_t module_id) const { return registered_module_id_set_.count(module_id) != 0; }
  void registerModule(const std::shared_ptr<T> & module, const PlannerData &)
  {
    registered_module_id_set_.insert(module->getModuleId());
    scene_modules_.insert(module);
  }
  ros::NodeHandle node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  std::set<std::shared_ptr<T>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  virtual void deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data)
  {
    const auto expired = getModuleExpiredFunction(path, planner_data);
    for (auto it = scene_modules_.begin(); it != scene_modules_.end();) {
      if (expired(*it)) {
        registered_module_id_set_.erase((*it)->getModuleId());
        it = scene_modules_.erase(it);
      } else {
        ++it;
      }
    }
  }

protected:
  std::string module_name_;
  motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_virtual_wall_;
};
}  // namespace autoware::behavior_velocity_planner::experimental
