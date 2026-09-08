#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>

#include <cstdio>
#include <sstream>

namespace autoware::behavior_velocity_planner::experimental
{
namespace
{
std::string format_ids(const std::vector<int64_t> & regulatory_element_ids,
  const std::vector<int64_t> & lanelet_ids, const std::vector<int64_t> & line_ids)
{
  const auto format_group = [](const std::vector<int64_t> & ids, const char * prefix) {
    if (ids.empty()) return std::string{};
    std::ostringstream stream;
    stream << "[" << prefix << ": ";
    for (size_t index = 0; index < ids.size(); ++index) stream << (index ? ", " : "") << ids.at(index);
    return stream.str() + "]";
  };
  return format_group(regulatory_element_ids, "Reg") + format_group(lanelet_ids, "Lane") +
    format_group(line_ids, "Line");
}
}

std::string SceneModuleInterface::formatLogMessage(const char * format, va_list args) const
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, args);
  return "[Module ID: " + std::to_string(module_id_) + "]" +
    format_ids(getRegulatoryElementIds(), getLaneletIds(), getLineIds()) + " " + buffer;
}

#define AW149_DEFINE_EXPERIMENTAL_LOG(function_name, ros_macro) \
  void SceneModuleInterface::function_name(const char * format, ...) const \
  { \
    va_list args; va_start(args, format); \
    const auto message = formatLogMessage(format, args); va_end(args); \
    ros_macro##_STREAM_NAMED(node_.getNamespace(), message); \
  }
AW149_DEFINE_EXPERIMENTAL_LOG(logInfo, ROS_INFO)
AW149_DEFINE_EXPERIMENTAL_LOG(logWarn, ROS_WARN)
AW149_DEFINE_EXPERIMENTAL_LOG(logDebug, ROS_DEBUG)
#undef AW149_DEFINE_EXPERIMENTAL_LOG

#define AW149_DEFINE_EXPERIMENTAL_THROTTLE_LOG(function_name, ros_macro) \
  void SceneModuleInterface::function_name(const int duration_ms, const char * format, ...) const \
  { \
    va_list args; va_start(args, format); \
    const auto message = formatLogMessage(format, args); va_end(args); \
    ros_macro##_THROTTLE_NAMED(static_cast<double>(duration_ms) / 1000.0, node_.getNamespace(), "%s", message.c_str()); \
  }
AW149_DEFINE_EXPERIMENTAL_THROTTLE_LOG(logInfoThrottle, ROS_INFO)
AW149_DEFINE_EXPERIMENTAL_THROTTLE_LOG(logWarnThrottle, ROS_WARN)
AW149_DEFINE_EXPERIMENTAL_THROTTLE_LOG(logDebugThrottle, ROS_DEBUG)
#undef AW149_DEFINE_EXPERIMENTAL_THROTTLE_LOG
}  // namespace autoware::behavior_velocity_planner::experimental
