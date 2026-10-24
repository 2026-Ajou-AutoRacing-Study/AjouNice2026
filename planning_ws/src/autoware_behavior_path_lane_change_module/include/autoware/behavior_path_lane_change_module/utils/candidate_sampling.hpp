// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#pragma once
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
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
}
