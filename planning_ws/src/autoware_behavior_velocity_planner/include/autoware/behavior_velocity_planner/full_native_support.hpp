// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_debug/ros1_adapters.hpp>
#include <ros/ros.h>
#include <builtin_interfaces/Time.h>
#include <limits>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#ifndef RCLCPP_ERROR
#define RCLCPP_ERROR(logger,...) ROS_ERROR(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(logger,...) ROS_WARN(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN_THROTTLE
#define RCLCPP_WARN_THROTTLE(logger,clock,ms,...) ROS_WARN_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
#ifndef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(logger,clock,ms,...) ROS_INFO_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG_STREAM
#define RCLCPP_DEBUG_STREAM(logger,args) ROS_DEBUG_STREAM(args)
#endif
#ifndef RCLCPP_INFO
#define RCLCPP_INFO(logger,...) ROS_INFO(__VA_ARGS__)
#endif
#ifndef RCLCPP_ERROR_THROTTLE
#define RCLCPP_ERROR_THROTTLE(logger,clock,ms,...) ROS_ERROR_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
#ifndef RCLCPP_ERROR_STREAM
#define RCLCPP_ERROR_STREAM(logger,args) ROS_ERROR_STREAM(args)
#endif
#ifndef RCLCPP_INFO_SKIPFIRST_THROTTLE
#define RCLCPP_INFO_SKIPFIRST_THROTTLE(logger,clock,ms,...) ROS_INFO_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
namespace autoware::behavior_velocity_planner::bvp_full_native {
struct Clock {ros::Time now() const {return ros::Time::now();}};
inline builtin_interfaces::Time stamp(const ros::Time & time) {
  if(time.sec>static_cast<uint32_t>(std::numeric_limits<int32_t>::max()))throw std::out_of_range("BPP signed timestamp overflow");
  builtin_interfaces::Time result;result.sec=time.sec;result.nanosec=time.nsec;return result;
}
inline ros::Time time(const builtin_interfaces::Time & stamp) {
  if(stamp.sec<0||stamp.nanosec>=1000000000U)throw std::invalid_argument("BPP invalid timestamp");
  return ros::Time(stamp.sec,stamp.nanosec);
}
template<class T> class Publisher {
  using Identified=autoware_utils_debug::ros1::IdentifiedPublisher<T>;
  std::shared_ptr<Identified> publisher_;
public:
  Publisher()=default;
  explicit Publisher(Identified publisher):publisher_(std::make_shared<Identified>(std::move(publisher))) {}
  const Publisher * operator->() const {return this;}
  void publish(const T & message) const {identity().publish(message);}
  const Identified & identity() const {
    if(!publisher_)throw std::logic_error("BPP publisher not initialized");
    return *publisher_;
  }
  size_t get_subscription_count() const {return identity().getNumSubscribers();}
  size_t get_intra_process_subscription_count() const {return 0;}
};
class Node:public ros::NodeHandle {
  std::shared_ptr<Clock> clock_=std::make_shared<Clock>();
  std::shared_ptr<ros::NodeHandle> debug_owner_;
public:
  autoware_utils_debug::ros1::CallerNodeContext::Ptr debug_context_;
  explicit Node(ros::NodeHandle node):ros::NodeHandle(std::move(node)),
    debug_owner_(std::make_shared<ros::NodeHandle>(*this)),
    debug_context_(autoware_utils_debug::ros1::CallerNodeContext::borrow(debug_owner_)) {}
  template<class T> T declare_parameter(const std::string & key) {
    return autoware_utils_rclcpp::detail::strict_get<T>(*this,autoware_utils_rclcpp::to_ros1_parameter_name(key));
  }
  template<class T> T declare_parameter(const std::string & key,const T & fallback) {
    if(!hasParam(autoware_utils_rclcpp::to_ros1_parameter_name(key)))return fallback;
    return declare_parameter<T>(key);
  }
  template<class T> Publisher<T> create_publisher(const std::string & topic,unsigned depth,bool latch=false) {
    autoware_utils_debug::ros1::PublisherFactory factory;
    return Publisher<T>(factory.advertise<T>(*this,topic,depth,latch));
  }
  ros::Time now() const {return clock_->now();}
  std::shared_ptr<Clock> get_clock() const {return clock_;}
  std::string get_logger() const {return getNamespace();}
};
template<class D, class S> void assign_message(D & destination, const S & source) { destination=source; }
template<class T> void assign_message(std::shared_ptr<const T> & destination, const boost::shared_ptr<const T> & source) {
  destination=source ? std::make_shared<const T>(*source) : nullptr;
}
}
