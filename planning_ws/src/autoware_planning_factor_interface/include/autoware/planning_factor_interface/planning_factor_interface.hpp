// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__PLANNING_FACTOR_INTERFACE__PLANNING_FACTOR_INTERFACE_HPP_
#define AUTOWARE__PLANNING_FACTOR_INTERFACE__PLANNING_FACTOR_INTERFACE_HPP_

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_internal_planning_msgs/ControlPoint.h>
#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_internal_planning_msgs/PlanningFactor.h>
#include <autoware_internal_planning_msgs/PlanningFactorArray.h>
#include <autoware_internal_planning_msgs/SafetyFactorArray.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace autoware
{
namespace planning_factor_interface
{

using autoware_internal_planning_msgs::ControlPoint;
using autoware_internal_planning_msgs::PlanningFactor;
using autoware_internal_planning_msgs::PlanningFactorArray;
using autoware_internal_planning_msgs::SafetyFactorArray;
using geometry_msgs::Pose;

class PlanningFactorInterface
{
public:
  PlanningFactorInterface(
    ros::NodeHandle * node, const std::string & name, bool enable_console_output = false,
    int throttle_duration_ms = 1000)
  : name_(name),
    pub_factors_(node->advertise<PlanningFactorArray>("/planning/planning_factors/" + name, 1)),
    enable_console_output_(enable_console_output),
    throttle_duration_ms_(throttle_duration_ms)
  {
  }

  template <class PointType>
  void add(
    const std::vector<PointType> & points, const Pose & ego_pose, const Pose & control_point_pose,
    const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double velocity = 0.0,
    const double shift_length = 0.0, const std::string & detail = "")
  {
    const float distance = static_cast<float>(autoware::motion_utils::calcSignedArcLength(
      points, ego_pose.position, control_point_pose.position));
    add(distance, control_point_pose, behavior, safety_factors, is_driving_forward, velocity,
      shift_length, detail);
  }

  template <class PointType>
  void add(
    const std::vector<PointType> & points, const Pose & ego_pose, const Pose & start_pose,
    const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double start_velocity = 0.0,
    const double end_velocity = 0.0, const double start_shift_length = 0.0,
    const double end_shift_length = 0.0, const std::string & detail = "")
  {
    const float start_distance = static_cast<float>(
      autoware::motion_utils::calcSignedArcLength(points, ego_pose.position, start_pose.position));
    const float end_distance = static_cast<float>(
      autoware::motion_utils::calcSignedArcLength(points, ego_pose.position, end_pose.position));
    add(start_distance, end_distance, start_pose, end_pose, behavior, safety_factors,
      is_driving_forward, start_velocity, end_velocity, start_shift_length, end_shift_length,
      detail);
  }

  void add(
    const double distance, const Pose & control_point_pose, const uint16_t behavior,
    const SafetyFactorArray & safety_factors, const bool is_driving_forward = true,
    const double velocity = 0.0, const double shift_length = 0.0, const std::string & detail = "")
  {
    ControlPoint control_point;
    control_point.pose = control_point_pose;
    control_point.velocity = static_cast<float>(velocity);
    control_point.shift_length = static_cast<float>(shift_length);
    control_point.distance = static_cast<float>(distance);

    PlanningFactor factor;
    factor.module = name_;
    factor.is_driving_forward = is_driving_forward;
    factor.control_points.push_back(control_point);
    factor.behavior = behavior;
    factor.detail = detail;
    factor.safety_factors = safety_factors;
    factors_.push_back(factor);
  }

  void add(
    const double start_distance, const double end_distance, const Pose & start_pose,
    const Pose & end_pose, const uint16_t behavior, const SafetyFactorArray & safety_factors,
    const bool is_driving_forward = true, const double start_velocity = 0.0,
    const double end_velocity = 0.0, const double start_shift_length = 0.0,
    const double end_shift_length = 0.0, const std::string & detail = "")
  {
    ControlPoint start_point;
    start_point.pose = start_pose;
    start_point.velocity = static_cast<float>(start_velocity);
    start_point.shift_length = static_cast<float>(start_shift_length);
    start_point.distance = static_cast<float>(start_distance);

    ControlPoint end_point;
    end_point.pose = end_pose;
    end_point.velocity = static_cast<float>(end_velocity);
    end_point.shift_length = static_cast<float>(end_shift_length);
    end_point.distance = static_cast<float>(end_distance);

    PlanningFactor factor;
    factor.module = name_;
    factor.is_driving_forward = is_driving_forward;
    factor.control_points.push_back(start_point);
    factor.control_points.push_back(end_point);
    factor.behavior = behavior;
    factor.detail = detail;
    factor.safety_factors = safety_factors;
    factors_.push_back(factor);
  }

  void publish()
  {
    PlanningFactorArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.factors = factors_;
    pub_factors_.publish(msg);
    if (enable_console_output_ && !factors_.empty()) {
      print_factors_to_console(msg);
    }
    factors_.clear();
  }

  std::vector<PlanningFactor> get_factors() const { return factors_; }

private:
  void print_factors_to_console(const PlanningFactorArray & msg)
  {
    std::ostringstream output;
    output << "Planning factor: count=" << msg.factors.size();
    for (std::vector<PlanningFactor>::const_iterator factor = msg.factors.begin();
         factor != msg.factors.end(); ++factor) {
      output << " module=" << factor->module << " behavior=" << factor->behavior
             << " control_points=" << factor->control_points.size();
    }
    const double period = static_cast<double>(throttle_duration_ms_) / 1000.0;
    if (throttle_duration_ms_ > 0) {
      ROS_INFO_STREAM_THROTTLE_NAMED(period, name_, output.str());
    } else {
      ROS_INFO_STREAM_NAMED(name_, output.str());
    }
  }

  std::string name_;
  ros::Publisher pub_factors_;
  std::vector<PlanningFactor> factors_;
  bool enable_console_output_{false};
  int throttle_duration_ms_{0};
};

extern template void PlanningFactorInterface::add<autoware_internal_planning_msgs::PathPointWithLaneId>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &, const Pose &,
  const Pose &, const uint16_t, const SafetyFactorArray &, const bool, const double,
  const double, const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::PathPoint>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const std::string &);
extern template void PlanningFactorInterface::add<autoware_internal_planning_msgs::PathPointWithLaneId>(
  const std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> &, const Pose &,
  const Pose &, const Pose &, const uint16_t, const SafetyFactorArray &, const bool,
  const double, const double, const double, const double, const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::PathPoint>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const Pose &, const Pose &, const Pose &,
  const uint16_t, const SafetyFactorArray &, const bool, const double, const double,
  const double, const double, const std::string &);
extern template void PlanningFactorInterface::add<autoware_planning_msgs::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const Pose &, const Pose &,
  const Pose &, const uint16_t, const SafetyFactorArray &, const bool, const double,
  const double, const double, const double, const std::string &);

}  // namespace planning_factor_interface
}  // namespace autoware

#endif
