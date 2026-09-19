#ifndef AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
#define AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_

#include <logging_demo/ConfigLogger.h>
#include <ros/ros.h>

namespace autoware_utils_logging
{
class LoggerLevelConfigure
{
public:
  explicit LoggerLevelConfigure(ros::NodeHandle & private_node_handle);

private:
  bool on_logger_config_service(
    logging_demo::ConfigLogger::Request & request,
    logging_demo::ConfigLogger::Response & response);

  ros::ServiceServer srv_config_logger_;
};
}  // namespace autoware_utils_logging

#endif  // AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
