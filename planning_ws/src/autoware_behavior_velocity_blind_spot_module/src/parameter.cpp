// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_velocity_blind_spot_module/parameter.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <string>

namespace autoware::behavior_velocity_planner
{

BlindSpotPlannerParam BlindSpotPlannerParam::init(ros::NodeHandle & node, const std::string & ns)
{
  using blind_spot_ros1::require_parameter;
  BlindSpotPlannerParam param;
  param.backward_attention_length =
    require_parameter<double>(node, ns + ".backward_attention_length");
  param.ttc_start_margin = require_parameter<double>(node, ns + ".ttc_start_margin");
  param.ttc_end_margin = require_parameter<double>(node, ns + ".ttc_end_margin");
  param.minimum_default_velocity =
    require_parameter<double>(node, ns + ".minimum_default_velocity");
  param.collision_judge_debounce =
    require_parameter<double>(node, ns + ".collision_judge_debounce");
  param.critical_stopline_margin =
    require_parameter<double>(node, ns + ".critical_stopline_margin");
  param.detect_non_vru.car = require_parameter<bool>(node, ns + ".detect_non_vru.car");
  param.detect_non_vru.truck = require_parameter<bool>(node, ns + ".detect_non_vru.truck");
  param.detect_non_vru.bus = require_parameter<bool>(node, ns + ".detect_non_vru.bus");
  param.detect_non_vru.trailer =
    require_parameter<bool>(node, ns + ".detect_non_vru.trailer");
  param.brake.critical.deceleration =
    require_parameter<double>(node, ns + ".brake.critical.deceleration");
  param.brake.critical.jerk = require_parameter<double>(node, ns + ".brake.critical.jerk");
  param.brake.semi_critical.deceleration =
    require_parameter<double>(node, ns + ".brake.semi_critical.deceleration");
  param.brake.semi_critical.jerk =
    require_parameter<double>(node, ns + ".brake.semi_critical.jerk");
  param.brake_for_ttc.critical_threshold_ub =
    require_parameter<double>(node, ns + ".brake_for_ttc.critical_threshold_ub");
  param.brake_for_ttc.semi_critical_threshold_lb =
    require_parameter<double>(node, ns + ".brake_for_ttc.semi_critical_threshold_lb");
  return param;
}
}  // namespace autoware::behavior_velocity_planner
