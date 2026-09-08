#pragma once

#include <rclcpp/logger.hpp>
#include <autoware_utils_rclcpp/parameter_event.hpp>

#include <ros/ros.h>

#include <chrono>
#include <string>
#include <vector>

#ifndef RCL_ROS_TIME
#define RCL_ROS_TIME 1
#endif

namespace rclcpp
{
using Time = ros::Time;

class Clock
{
public:
  explicit Clock(int = RCL_ROS_TIME) {}
  Time now() const { return ros::Time::now(); }
};

class Duration
{
public:
  explicit Duration(double seconds) : seconds_(seconds) {}
  explicit Duration(const ros::Duration & duration) : seconds_(duration.toSec()) {}
  double seconds() const { return seconds_; }

private:
  double seconds_{};
};



inline void init(int, char **) {}
inline void shutdown() {}
inline bool ok() { return ros::ok(); }
inline void sleep_for(const Duration & duration) { ros::Duration(duration.seconds()).sleep(); }
template <class Rep, class Period>
inline void sleep_for(const std::chrono::duration<Rep, Period> & duration)
{
  ros::Duration(std::chrono::duration<double>(duration).count()).sleep();
}
}  // namespace rclcpp

