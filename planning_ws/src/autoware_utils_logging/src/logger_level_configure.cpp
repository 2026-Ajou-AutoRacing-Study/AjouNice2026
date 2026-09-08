#include "autoware_utils_logging/logger_level_configure.hpp"

#include <ros/console.h>

#include <array>
#include <string>

namespace autoware_utils_logging
{
namespace
{
bool parse_level(const std::string & text, ros::console::levels::Level * level)
{
  static const std::array<std::pair<const char *, ros::console::levels::Level>, 5> levels{{
    {"DEBUG", ros::console::levels::Debug},
    {"INFO", ros::console::levels::Info},
    {"WARN", ros::console::levels::Warn},
    {"ERROR", ros::console::levels::Error},
    {"FATAL", ros::console::levels::Fatal},
  }};
  for (const auto & candidate : levels) {
    if (text == candidate.first) {
      *level = candidate.second;
      return true;
    }
  }
  return false;
}
}  // namespace

LoggerLevelConfigure::LoggerLevelConfigure(ros::NodeHandle & private_node_handle)
{
  srv_config_logger_ = private_node_handle.advertiseService(
    "config_logger", &LoggerLevelConfigure::on_logger_config_service, this);
}

bool LoggerLevelConfigure::on_logger_config_service(
  logging_demo::ConfigLogger::Request & request,
  logging_demo::ConfigLogger::Response & response)
{
  ros::console::levels::Level level;
  if (!parse_level(request.level, &level)) {
    response.success = false;
    ROS_WARN_STREAM("Failed to change logger level for " << request.logger_name
                    << " due to an invalid logging severity: " << request.level);
    return true;
  }

  response.success = ros::console::set_logger_level(request.logger_name, level);
  if (!response.success) {
    ROS_WARN_STREAM("Failed to set logger level for " << request.logger_name);
    return true;
  }

  ROS_INFO_STREAM("Logger level [" << request.level << "] is set for " << request.logger_name);
  return true;
}
}  // namespace autoware_utils_logging
