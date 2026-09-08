#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cstdarg>
#include <cstdio>
#include <sstream>

namespace autoware::behavior_velocity_planner
{
namespace
{
std::string formatIds(const std::vector<int64_t> & ids, const char * prefix)
{
  if (ids.empty()) return "";
  std::ostringstream stream;
  stream << "[" << prefix << ": ";
  for (size_t i = 0; i < ids.size(); ++i) stream << (i == 0 ? "" : ", ") << ids.at(i);
  return stream.str() + "]";
}
}

SceneModuleInterface::SceneModuleInterface(
  const int64_t module_id, const ros::NodeHandle & node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface)
: module_id_(module_id), node_(node), time_keeper_(std::move(time_keeper)),
  planning_factor_interface_(std::move(planning_factor_interface))
{
}

void SceneModuleInterface::setObjectsOfInterestData(
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::Shape & shape,
  const objects_of_interest_marker_interface::ColorName color)
{
  objects_of_interest_.push_back(ObjectOfInterest{pose, shape, color});
}

size_t SceneModuleInterface::findEgoSegmentIndex(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> & points) const
{
  const auto & data = planner_data_;
  return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, data->current_odometry->pose, data->ego_nearest_dist_threshold,
    data->ego_nearest_yaw_threshold);
}

std::string SceneModuleInterface::formatLogMessage(const char * format, va_list args) const
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, args);
  return "[Module ID: " + std::to_string(module_id_) + "]" +
         formatIds(getRegulatoryElementIds(), "Reg") + formatIds(getLaneletIds(), "Lane") +
         formatIds(getLineIds(), "Line") + " " + buffer;
}

void SceneModuleInterface::logInfo(const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_INFO_STREAM_NAMED(node_.getNamespace(), message);
}

void SceneModuleInterface::logWarn(const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_WARN_STREAM_NAMED(node_.getNamespace(), message);
}

void SceneModuleInterface::logDebug(const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_DEBUG_STREAM_NAMED(node_.getNamespace(), message);
}

void SceneModuleInterface::logInfoThrottle(const int duration_ms, const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_INFO_THROTTLE_NAMED(static_cast<double>(duration_ms) / 1000.0, node_.getNamespace(), "%s", message.c_str());
}

void SceneModuleInterface::logWarnThrottle(const int duration_ms, const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_WARN_THROTTLE_NAMED(static_cast<double>(duration_ms) / 1000.0, node_.getNamespace(), "%s", message.c_str());
}

void SceneModuleInterface::logDebugThrottle(const int duration_ms, const char * format, ...) const
{
  va_list args; va_start(args, format);
  const auto message = formatLogMessage(format, args); va_end(args);
  ROS_DEBUG_THROTTLE_NAMED(static_cast<double>(duration_ms) / 1000.0, node_.getNamespace(), "%s", message.c_str());
}
}  // namespace autoware::behavior_velocity_planner
