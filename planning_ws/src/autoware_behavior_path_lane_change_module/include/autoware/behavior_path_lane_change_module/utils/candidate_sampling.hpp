// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/structs/path.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
namespace autoware::behavior_path_planner::utils::lane_change
{
// Explicit preparation times are absolute alternatives from the current state.
// Do not shift the grid with readiness time or add an unrequested ninth sample.
inline std::vector<double> preparation_duration_samples(
  const autoware::behavior_path_planner::lane_change::TrajectoryParameters & params,
  const double earliest)
{
  std::vector<double> result;
  if (!std::isfinite(earliest) || earliest < 0.0 || earliest > params.max_prepare_duration) {
    return result;
  }
  for (const double value : params.preparation_duration_samples) {
    if (!std::isfinite(value) || value < earliest - 1e-6 ||
        value > params.max_prepare_duration + 1e-6) continue;
    if (std::none_of(result.begin(), result.end(), [&](double x) {
        return std::abs(x - value) < 1e-6; })) result.push_back(value);
  }
  std::sort(result.begin(), result.end());
  return result;
}

// A Frenet path's initial lateral acceleration can be almost zero even for
// an abrupt maneuver. Rank the whole maneuver's peak, not its first sample.
inline double candidate_lateral_acceleration(const LaneChangePath & candidate)
{
  if (candidate.type != autoware::behavior_path_planner::lane_change::PathType::FrenetPlanner ||
      candidate.frenet_path.lane_changing.lateral_accelerations.empty()) {
    return std::abs(candidate.info.lateral_acceleration);
  }
  double peak = 0.0;
  for (double acceleration : candidate.frenet_path.lane_changing.lateral_accelerations) {
    if (!std::isfinite(acceleration)) return std::numeric_limits<double>::infinity();
    peak = std::max(peak, std::abs(acceleration));
  }
  return peak;
}

inline bool frenet_lateral_envelope_satisfied(
  const autoware::frenet_planner::Trajectory & trajectory,
  const autoware::behavior_path_planner::lane_change::TrajectoryParameters & parameters)
{
  if (trajectory.lateral_accelerations.size() != trajectory.longitudinal_velocities.size() ||
      trajectory.lateral_accelerations.empty() || parameters.lat_acc_map.base_vel.empty()) return false;
  for (size_t i = 0; i < trajectory.lateral_accelerations.size(); ++i) {
    const double a = trajectory.lateral_accelerations[i];
    const double v = trajectory.longitudinal_velocities[i];
    if (!std::isfinite(a) || !std::isfinite(v) ||
        std::abs(a) > parameters.lat_acc_map.find(std::abs(v)).second + 1e-3) return false;
  }
  return true;
}

// Preserve earliest preparation and strongest acceleration preference, while
// sampling constant-speed and mild braking before exhausting positive samples.
inline std::vector<LaneChangePhaseMetrics> diversify_prepare_metrics(
  const std::vector<LaneChangePhaseMetrics> & input)
{
  std::vector<LaneChangePhaseMetrics> output;
  for (size_t begin=0; begin<input.size();) {
    size_t end=begin;
    while(end<input.size() && std::abs(input[end].duration-input[begin].duration)<1e-6) ++end;
    std::vector<LaneChangePhaseMetrics> positive,zero,negative;
    for(size_t i=begin;i<end;++i) {
      if(input[i].sampled_lon_accel>1e-6) positive.push_back(input[i]);
      else if(input[i].sampled_lon_accel < -1e-6) negative.push_back(input[i]);
      else zero.push_back(input[i]);
    }
    for(size_t i=0;i<std::max({positive.size(),zero.size(),negative.size()});++i) {
      if(i<positive.size()) output.push_back(positive[i]);
      if(i<zero.size()) output.push_back(zero[i]);
      if(i<negative.size()) output.push_back(negative[i]);
    }
    begin=end;
  }
  return output;
}
// Explicit levels are limits, not requested achieved accelerations. Respect the
// velocity-dependent physical map and collapse levels clipped to the same limit.
inline std::vector<double> lateral_acceleration_samples(
  const autoware::behavior_path_planner::lane_change::TrajectoryParameters & params, const double velocity)
{
  const auto [lo, hi] = params.lat_acc_map.find(velocity);
  std::vector<double> result;
  if (!params.lateral_acceleration_samples.empty()) {
    for (const double value : params.lateral_acceleration_samples) {
      const double clipped = std::clamp(value, lo, hi);
      if (std::none_of(result.begin(), result.end(), [&](double x) {
          return std::abs(x - clipped) < 1e-6; })) result.push_back(clipped);
    }
  } else {
    const int intervals = std::max(1, params.lat_acc_sampling_num);
    for (int i=0; i<=intervals; ++i) {
      if (i>0 && std::abs(hi-lo)<1e-6) break;
      result.push_back(lo+(hi-lo)*i/intervals);
    }
  }
  std::sort(result.begin(),result.end());
  return result;
}

// Keep physically feasible preparation kinematics and one intent per strategy.
// A road cap can force coasting during preparation without forbidding acceleration
// on the execution road. sampled_lon_accel carries that intent; acceleration and
// velocity remain the feasible preparation values. No independent Cartesian grid.
inline std::vector<LaneChangePhaseMetrics> compact_prepare_metrics(
  const std::vector<LaneChangePhaseMetrics> & input,
  const double front_acceleration, const double rear_acceleration)
{
  std::vector<LaneChangePhaseMetrics> result;
  for (size_t begin=0; begin<input.size();) {
    size_t end=begin+1;
    while(end<input.size() && std::abs(input[end].duration-input[begin].duration)<1e-6) ++end;
    for (const double desired : {front_acceleration, 0.0, rear_acceleration}) {
      const auto first=input.begin()+begin, last=input.begin()+end;
      const auto best=std::min_element(first,last,[&](const auto & a,const auto & b) {
        return std::abs(a.sampled_lon_accel-desired)<std::abs(b.sampled_lon_accel-desired);
      });
      auto selected = *best;
      // Never let a zero sampling hint erase the accelerating alternative. At
      // a preparation speed cap the execution phase checks its own road caps.
      if (desired > 1e-6 && selected.sampled_lon_accel <= 1e-6) {
        selected.sampled_lon_accel = desired;
      }
      if (std::none_of(result.begin(),result.end(),[&](const auto & metric) {
        return std::abs(metric.duration-selected.duration)<1e-6 &&
          std::abs(metric.sampled_lon_accel-selected.sampled_lon_accel)<1e-6;
      })) result.push_back(selected);
    }
    begin=end;
  }
  return result;
}

}
