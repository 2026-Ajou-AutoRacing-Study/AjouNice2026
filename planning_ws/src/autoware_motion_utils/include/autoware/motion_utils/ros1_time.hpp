#ifndef AUTOWARE__MOTION_UTILS__ROS1_TIME_HPP_
#define AUTOWARE__MOTION_UTILS__ROS1_TIME_HPP_

#include <builtin_interfaces/Duration.h>
#include <ros/duration.h>

namespace autoware::motion_utils
{
inline double duration_to_seconds(const builtin_interfaces::Duration & duration)
{
  return ros::Duration(duration.sec, duration.nanosec).toSec();
}

inline builtin_interfaces::Duration duration_from_seconds(const double seconds)
{
  const ros::Duration duration(seconds);
  builtin_interfaces::Duration output;
  output.sec = duration.sec;
  output.nanosec = duration.nsec;
  return output;
}
}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__ROS1_TIME_HPP_
