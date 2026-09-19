// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <ros/ros.h>
#include <builtin_interfaces/Time.h>
#include <boost/make_shared.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef RCLCPP_INFO_THROTTLE
#define RCLCPP_INFO_THROTTLE(logger,clock,ms,...) ROS_INFO_THROTTLE((ms)/1000.0,__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(logger,...) ROS_WARN(__VA_ARGS__)
#endif
namespace autoware::surround_obstacle_checker::full_native {
struct Clock { ros::Time now() const { return ros::Time::now(); } };
class Node : public ros::NodeHandle {
  std::shared_ptr<Clock> clock_=std::make_shared<Clock>();
public:
  explicit Node(ros::NodeHandle node):ros::NodeHandle(std::move(node)) {}
  std::shared_ptr<Clock> get_clock() const { return clock_; }
  ros::Time now() const { return clock_->now(); }
  std::string get_logger() const { return getNamespace(); }
};
inline builtin_interfaces::Time stamp(const ros::Time & time) {
  if(time.sec>static_cast<uint32_t>(std::numeric_limits<int32_t>::max()))
    throw std::out_of_range("surround timestamp exceeds original signed seconds domain");
  builtin_interfaces::Time result;result.sec=time.sec;result.nanosec=time.nsec;return result;
}
}
namespace surround_obstacle_checker_node {
struct Enabled { bool enable_check; };
struct Distances { double surround_check_front_distance, surround_check_side_distance, surround_check_back_distance; };
struct Params {
  Enabled pointcloud;
  std::vector<std::string> object_types,obstacle_types;
  std::map<std::string,Enabled> object_types_map;
  std::map<std::string,Distances> obstacle_types_map;
  double surround_check_hysteresis_distance,state_clear_time,stop_state_ego_speed;
  bool publish_debug_footprints;
  std::string debug_footprint_label;
};
class ParamListener {
  ros::NodeHandle node_;
  mutable Params last_;
  template<class T> T require(const std::string & key) const {
    T value{};
    if(!node_.getParam(key,value))throw std::runtime_error("required surround parameter missing: "+key);
    return value;
  }
  double nonnegative(const std::string & key) const {
    double value=require<double>(key);
    if(!std::isfinite(value)||value<0.0)throw std::invalid_argument("surround parameter must be nonnegative: "+key);
    return value;
  }
  Params read() const {
    Params p;
    p.object_types={"unknown","car","truck","bus","trailer","motorcycle","bicycle","pedestrian"};
    p.obstacle_types=p.object_types;p.obstacle_types.insert(p.obstacle_types.begin(),"pointcloud");
    const auto valid_labels=p.obstacle_types;
    if(node_.hasParam("object_types"))p.object_types=require<std::vector<std::string>>("object_types");
    if(node_.hasParam("obstacle_types"))p.obstacle_types=require<std::vector<std::string>>("obstacle_types");
    p.pointcloud={require<bool>("pointcloud/enable_check")};
    for(const auto & label:p.object_types)p.object_types_map[label]={require<bool>(label+"/enable_check")};
    for(const auto & label:p.obstacle_types)p.obstacle_types_map[label]={
      nonnegative(label+"/surround_check_front_distance"),nonnegative(label+"/surround_check_side_distance"),
      nonnegative(label+"/surround_check_back_distance")};
    p.surround_check_hysteresis_distance=nonnegative("surround_check_hysteresis_distance");
    p.state_clear_time=nonnegative("state_clear_time");p.stop_state_ego_speed=nonnegative("stop_state_ego_speed");
    p.publish_debug_footprints=require<bool>("publish_debug_footprints");
    p.debug_footprint_label=require<std::string>("debug_footprint_label");
    if(std::find(valid_labels.begin(),valid_labels.end(),p.debug_footprint_label)==valid_labels.end())
      throw std::invalid_argument("invalid surround debug_footprint_label");
    return p;
  }
public:
  explicit ParamListener(ros::NodeHandle node):node_(std::move(node)),last_(read()) {}
  Params get_params() const {
    try {last_=read();}catch(const std::exception & e){ROS_ERROR_THROTTLE(1.0,"Surround parameter update rejected: %s",e.what());}
    return last_;
  }
};
}
