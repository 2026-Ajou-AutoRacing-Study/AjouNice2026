#pragma once

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/objects_of_interest_marker_interface/marker_data.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdarg>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_factor_interface
{
class PlanningFactorInterface;
}

namespace autoware_utils_debug
{
class TimeKeeper;
}

namespace autoware::behavior_velocity_planner
{
struct ObjectOfInterest
{
  geometry_msgs::Pose pose;
  autoware_perception_msgs::Shape shape;
  objects_of_interest_marker_interface::ColorName color;
};

// ROS1 scene-module base boundary.  It intentionally owns no publisher: the
// manager layer must use a provider adapter that cannot alter this common
// package's ROS1 message-header resolution.
class SceneModuleInterface
{
public:
  SceneModuleInterface(
    int64_t module_id, const ros::NodeHandle & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface);
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(autoware_internal_planning_msgs::PathWithLaneId * path) = 0;
  virtual visualization_msgs::MarkerArray createDebugMarkerArray() = 0;
  virtual motion_utils::VirtualWalls createVirtualWalls() = 0;

  int64_t getModuleId() const { return module_id_; }
  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }
  std::vector<ObjectOfInterest> getObjectsOfInterestData() const { return objects_of_interest_; }
  void clearObjectsOfInterestData() { objects_of_interest_.clear(); }

protected:
  const int64_t module_id_;
  ros::NodeHandle node_;
  std::shared_ptr<const PlannerData> planner_data_;
  std::vector<ObjectOfInterest> objects_of_interest_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;

  void setObjectsOfInterestData(
    const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
    objects_of_interest_marker_interface::ColorName color);
  size_t findEgoSegmentIndex(
    const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points) const;
  void logInfo(const char * format, ...) const;
  void logWarn(const char * format, ...) const;
  void logDebug(const char * format, ...) const;
  void logInfoThrottle(int duration_ms, const char * format, ...) const;
  void logWarnThrottle(int duration_ms, const char * format, ...) const;
  void logDebugThrottle(int duration_ms, const char * format, ...) const;
  virtual std::vector<int64_t> getRegulatoryElementIds() const { return {}; }
  virtual std::vector<int64_t> getLaneletIds() const { return {}; }
  virtual std::vector<int64_t> getLineIds() const { return {}; }

private:
  std::string formatLogMessage(const char * format, va_list args) const;
};
}  // namespace autoware::behavior_velocity_planner
