#pragma once

#include <ros/ros.h>

#include <string>

namespace rclcpp
{
class Logger
{
public:
  explicit Logger(std::string name = "aw149") : name_(std::move(name)) {}
  Logger get_child(const std::string & child) const { return Logger(name_ + "." + child); }
  const std::string & name() const { return name_; }

private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) { return Logger(name); }
}  // namespace rclcpp

#undef RCLCPP_DEBUG
#undef RCLCPP_WARN
#undef RCLCPP_ERROR
#undef RCLCPP_INFO
#undef RCLCPP_WARN_THROTTLE
#undef RCLCPP_INFO_THROTTLE
#undef RCLCPP_DEBUG_THROTTLE
#undef RCLCPP_DEBUG_STREAM
#undef RCLCPP_INFO_STREAM
#undef RCLCPP_WARN_STREAM
#undef RCLCPP_ERROR_STREAM
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, period_ms, ...) ROS_WARN_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...) ROS_INFO_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...) ROS_DEBUG_THROTTLE((period_ms) / 1000.0, __VA_ARGS__)
#define RCLCPP_DEBUG_STREAM(logger, stream_args) ROS_DEBUG_STREAM(stream_args)
#define RCLCPP_INFO_STREAM(logger, stream_args) ROS_INFO_STREAM(stream_args)
#define RCLCPP_WARN_STREAM(logger, stream_args) ROS_WARN_STREAM(stream_args)
#define RCLCPP_ERROR_STREAM(logger, stream_args) ROS_ERROR_STREAM(stream_args)

#undef RCLCPP_FATAL_STREAM
#undef RCLCPP_ERROR_THROTTLE
#define RCLCPP_FATAL_STREAM(logger,args) ROS_FATAL_STREAM(args)
#define RCLCPP_ERROR_THROTTLE(logger,clock,ms,...) ROS_ERROR_THROTTLE((ms)/1000.0,__VA_ARGS__)
