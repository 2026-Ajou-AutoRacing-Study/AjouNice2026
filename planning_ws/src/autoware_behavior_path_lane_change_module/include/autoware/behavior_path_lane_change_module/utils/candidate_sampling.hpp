// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <tuple>
namespace autoware::behavior_path_planner::utils::lane_change
{
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

// Use existing physically feasible metrics, rather than inventing a second
// velocity profile for the same strategy. No Cartesian prep/execution accel product.
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
      if (std::none_of(result.begin(),result.end(),[&](const auto & metric) {
        return std::abs(metric.duration-best->duration)<1e-6 &&
          std::abs(metric.sampled_lon_accel-best->sampled_lon_accel)<1e-6;
      })) result.push_back(*best);
    }
    begin=end;
  }
  return result;
}

inline int acceleration_priority(double acceleration)
{
  constexpr double epsilon = 1e-3;
  return acceleration > epsilon ? 0 : (acceleration < -epsilon ? 2 : 1);
}

inline std::vector<LaneChangePhaseMetrics> independent_execution_metrics(
  const std::vector<LaneChangePhaseMetrics> & preparation,
  const std::vector<double> & execution_accelerations)
{
  std::vector<LaneChangePhaseMetrics> result;
  for (const auto & prep : preparation) {
    for (const double acceleration : execution_accelerations) {
      auto metric = prep;
      metric.sampled_lon_accel = acceleration;
      if (std::none_of(result.begin(), result.end(), [&](const auto & other) {
            return std::abs(other.duration - metric.duration) < 1e-6 &&
              std::abs(other.actual_lon_accel - metric.actual_lon_accel) < 1e-6 &&
              std::abs(other.sampled_lon_accel - acceleration) < 1e-6;
          })) result.push_back(metric);
    }
  }
  std::stable_sort(result.begin(), result.end(), [](const auto & a, const auto & b) {
    return std::make_tuple(a.duration, acceleration_priority(a.actual_lon_accel),
                          acceleration_priority(a.sampled_lon_accel)) <
           std::make_tuple(b.duration, acceleration_priority(b.actual_lon_accel),
                          acceleration_priority(b.sampled_lon_accel));
  });
  return result;
}

}
