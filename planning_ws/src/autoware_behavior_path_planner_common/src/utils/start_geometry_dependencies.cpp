// Copyright 2022 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
// Function bodies restored from frozen source-closure-001; see inventory receipt.
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <boost/geometry.hpp>
#include <cmath>
#include <stdexcept>
namespace autoware::behavior_path_planner::utils {
using Vector3 = geometry_msgs::Vector3;
using Pose = geometry_msgs::Pose;
using PredictedObjects = autoware_perception_msgs::PredictedObjects;
using PathPointWithLaneId = autoware_internal_planning_msgs::PathPointWithLaneId;
std::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.leftBound().id() == current_lane.rightBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

std::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.rightBound().id() == current_lane.leftBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

PathPointWithLaneId insertStopPoint(const double length, PathWithLaneId & path)
{
  const size_t original_size = path.points.size();

  // insert stop point
  const auto insert_idx = autoware::motion_utils::insertStopPoint(length, path.points);
  if (!insert_idx) {
    return PathPointWithLaneId();
  }

  // check if a stop point is inserted
  if (path.points.size() == original_size) {
    return path.points.at(*insert_idx);
  }

  if (*insert_idx == 0 || *insert_idx == original_size - 1) {
    return path.points.at(*insert_idx);
  }

  // check lane ids of the inserted stop point
  path.points.at(*insert_idx).lane_ids = {};
  const auto & prev_lane_ids = path.points.at(*insert_idx - 1).lane_ids;
  const auto & next_lane_ids = path.points.at(*insert_idx + 1).lane_ids;

  for (const auto target_lane_id : prev_lane_ids) {
    if (
      std::find(next_lane_ids.begin(), next_lane_ids.end(), target_lane_id) !=
      next_lane_ids.end()) {
      path.points.at(*insert_idx).lane_ids.push_back(target_lane_id);
    }
  }

  // If there is no lane ids, we are going to insert prev lane ids
  if (path.points.at(*insert_idx).lane_ids.empty()) {
    path.points.at(*insert_idx).lane_ids = prev_lane_ids;
  }

  return path.points.at(*insert_idx);
}

std::optional<double> calc_feasible_decel_distance(
  const std::shared_ptr<const PlannerData> & planner_data, const double acc_lim,
  const double jerk_lim, const double target_velocity)
{
  const auto v_now = planner_data->self_odometry->twist.twist.linear.x;

  if (acc_lim >= 0.0) {
    throw std::invalid_argument("Maximum deceleration value must be negative.");
  }

  if (v_now < target_velocity) {
    return std::nullopt;
  }

  const auto a_now = planner_data->self_acceleration->accel.accel.linear.x;
  auto min_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    v_now, target_velocity, a_now, acc_lim, jerk_lim, -1.0 * jerk_lim);

  if (min_stop_distance) {
    return std::max(*min_stop_distance, 0.0);
  }

  return std::nullopt;
}

PoseWithDetailOpt insert_feasible_stop_point(
  PathWithLaneId & current_path, const std::shared_ptr<const PlannerData> & planner_data,
  const double maximum_deceleration, const double maximum_jerk, const std::string & stop_reason)
{
  if (current_path.points.empty()) {
    return std::nullopt;
  }

  constexpr double target_velocity = 0.0;
  const auto min_stop_distance =
    calc_feasible_decel_distance(planner_data, maximum_deceleration, maximum_jerk, target_velocity);

  if (!min_stop_distance) {
    return std::nullopt;
  }

  const auto stop_idx = autoware::motion_utils::insertStopPoint(
    planner_data->self_odometry->pose.pose, *min_stop_distance, current_path.points);

  if (!stop_idx) {
    return std::nullopt;
  }

  return PoseWithDetail(current_path.points.at(*stop_idx).point.pose, stop_reason);
}

std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & shoulder_lanes)
{
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & current_lane : current_lanes) {
    DrivableLanes drivable_lane;

    const auto right_lane = utils::getRightLanelet(current_lane, shoulder_lanes);
    const auto left_lane = utils::getLeftLanelet(current_lane, shoulder_lanes);

    if (right_lane && left_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = *left_lane;
      drivable_lane.middle_lanes.push_back(current_lane);
    } else if (right_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = current_lane;
    } else if (left_lane) {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = *left_lane;
    } else {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = current_lane;
    }

    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

std::vector<DrivableLanes> getNonOverlappingExpandedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const DrivableAreaExpansionParameters & parameters)
{
  const auto shorten_lanes = cutOverlappedLanes(path, lanes);
  return utils::expandLanelets(
    shorten_lanes, parameters.drivable_area_left_bound_offset,
    parameters.drivable_area_right_bound_offset, parameters.drivable_area_types_to_skip);
}
namespace parking_departure {
std::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, const double acc_lim, const double jerk_lim,
  const double target_velocity)
{
  const auto v_now = planner_data->self_odometry->twist.twist.linear.x;

  if (v_now < target_velocity) {
    return 0.0;
  }

  return calc_feasible_decel_distance(planner_data, -acc_lim, jerk_lim, target_velocity);
}
}
}
