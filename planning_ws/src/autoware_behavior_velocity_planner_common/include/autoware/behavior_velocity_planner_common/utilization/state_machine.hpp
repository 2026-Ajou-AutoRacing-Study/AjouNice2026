// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_STATE_MACHINE_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_PLANNER_COMMON_UTILIZATION_STATE_MACHINE_HPP_

#include <ros/ros.h>

#include <string>

namespace autoware::behavior_velocity_planner
{
/** Manage STOP/GO transitions, holding STOP for a configured margin before GO.
 *
 * ROS2's rclcpp::Clock/logger arguments are represented by an explicit ROS1
 * timestamp.  This makes the state transition deterministic for both ROS time
 * and unit tests; callers choose ros::Time::now() in runtime code.
 */
class StateMachine
{
public:
  enum State { STOP = 0, GO };

  static std::string toString(const State state)
  {
    if (state == STOP) return "STOP";
    if (state == GO) return "GO";
    return "";
  }

  StateMachine() = default;

  void setStateWithMarginTime(const State requested, const ros::Time & now)
  {
    if (state_ == requested) {
      start_time_ = ros::Time();
      waiting_for_go_ = false;
      return;
    }

    if (requested == STOP) {
      state_ = STOP;
      start_time_ = ros::Time();
      waiting_for_go_ = false;
      duration_ = 0.0;
      return;
    }

    if (requested == GO) {
      if (!waiting_for_go_) {
        start_time_ = now;
        waiting_for_go_ = true;
        duration_ = 0.0;
        return;
      }
      duration_ = (now - start_time_).toSec();
      if (duration_ > margin_time_) {
        state_ = GO;
        start_time_ = ros::Time();
        waiting_for_go_ = false;
      }
      return;
    }

    ROS_ERROR("Unsuitable state. ignore request.");
  }

  void setMarginTime(const double seconds) { margin_time_ = seconds; }
  void setState(const State state)
  {
    state_ = state;
    start_time_ = ros::Time();
    waiting_for_go_ = false;
    duration_ = 0.0;
  }
  State getState() const { return state_; }
  double getDuration() const { return duration_; }

private:
  State state_{GO};
  double margin_time_{0.0};
  double duration_{0.0};
  ros::Time start_time_{};
  bool waiting_for_go_{false};
};
}  // namespace autoware::behavior_velocity_planner

#endif
