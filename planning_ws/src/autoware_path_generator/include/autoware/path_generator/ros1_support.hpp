#pragma once
#include <ros/ros.h>
namespace autoware::path_generator_ros1 {
template<class T> void stamp(T & out,const ros::Time & now) { out.sec=now.sec; out.nanosec=now.nsec; }
inline void stamp(ros::Time & out,const ros::Time & now) { out=now; }
}
#ifndef RCLCPP_ERROR
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#endif
#ifndef RCLCPP_ERROR_THROTTLE
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) ROS_ERROR_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
#ifndef RCLCPP_INFO_SKIPFIRST_THROTTLE
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger, clock, ms, ...) ROS_INFO_DELAYED_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
