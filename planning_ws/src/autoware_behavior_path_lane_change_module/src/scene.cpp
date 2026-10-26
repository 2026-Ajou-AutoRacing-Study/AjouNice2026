// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/scene.hpp"

#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_lane_change_module/utils/safety_check.hpp"
#include "autoware/behavior_path_lane_change_module/utils/overtake.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_frenet_planner/structures.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <range/v3/action/insert.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view.hpp>

#include <boost/geometry/algorithms/buffer.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <std_msgs/String.h>
#include <chrono>
#include "autoware/behavior_path_lane_change_module/utils/candidate_sampling.hpp"

namespace autoware::behavior_path_planner
{
using autoware_adapi_v1_msgs::OperationModeState;
using autoware::motion_utils::calcSignedArcLength;
using utils::lane_change::create_lanes_polygon;
namespace calculation = utils::lane_change::calculation;
using utils::path_safety_checker::filter::velocity_filter;

NormalLaneChange::NormalLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: LaneChangeBase(parameters, type, direction)
{
  stop_watch_.tic(getModuleTypeStr());
  stop_watch_.tic("stop_time");
  if (type == LaneChangeModuleType::NORMAL) {
    ros::NodeHandle node("~");
    safety_diagnostics_pub_ = node.advertise<std_msgs::String>(
      std::string("debug/lane_change_") +
      (direction == Direction::LEFT ? "left" : "right") + "/safety_checks", 1);
    // The ROS1 planning launch publishes the final (post-smoother) trajectory
    // on /planning/trajectory. Keep this input configurable for other launches.
    const auto final_trajectory_topic = node.param<std::string>(
      "lane_change/overtake/final_trajectory_topic", "/planning/trajectory");
    final_trajectory_sub_ = node.subscribe(
      final_trajectory_topic, 1,
      &NormalLaneChange::receiveFinalTrajectory, this);
  }
}

void NormalLaneChange::update_lanes(const bool is_approved)
{
  if (is_approved) {
    return;
  }

  const auto invalidate_pending_lanes = [&](const char * reason) {
    // A pending candidate must not keep geometry from a previous route section
    // when current/target discovery fails. Approved maneuvers return above.
    *common_data_ptr_->lanes_ptr = lane_change::Lanes();
    *common_data_ptr_->lanes_polygon_ptr = lane_change::LanesPolygon{};
    common_data_ptr_->current_lanes_path = PathWithLaneId{};
    common_data_ptr_->target_lanes_path = PathWithLaneId{};
    common_data_ptr_->no_lane_change_lines.clear();
    common_data_ptr_->transient_data = lane_change::TransientData{};
    terminal_lane_change_path_.reset();
    overtake_ = OvertakeState{};
    overtake_prepare_path_.reset();
    status_ = LaneChangeStatus{};
    lane_change_debug_.reset();
    RCLCPP_DEBUG(logger_, "LC lane refresh invalid side=%s reason=%s",
      direction_ == Direction::LEFT ? "left" : "right", reason);
  };

  auto current_lanes = utils::getCurrentLanesFromPath(prev_module_output_.path, planner_data_);
  utils::lane_change::trim_preferred_after_alternative(
    current_lanes, common_data_ptr_->route_handler_ptr->getPreferredLanelets());

  if (current_lanes.empty()) {
    invalidate_pending_lanes("current_sequence_empty");
    return;
  }

  const auto target_lanes = get_lane_change_lanes(current_lanes);
  if (target_lanes.empty()) {
    invalidate_pending_lanes("target_sequence_empty");
    return;
  }

  lanelet::ConstLanelet current_lane;
  if (!common_data_ptr_->route_handler_ptr->getClosestLaneletWithinRoute(
        common_data_ptr_->get_ego_pose(), &current_lane)) {
    invalidate_pending_lanes("closest_route_lane_unavailable");
    return;
  }

  common_data_ptr_->lanes_ptr->ego_lane = current_lane;
  const auto is_same_lanes_with_prev_iteration =
    utils::lane_change::is_same_lane_with_prev_iteration(
      common_data_ptr_, current_lanes, target_lanes);

  if (is_same_lanes_with_prev_iteration) {
    return;
  }

  // A cached terminal maneuver belongs to the old lane sequence, not to the
  // next left/right request or a different successor at a merge.
  terminal_lane_change_path_.reset();
  const auto shares_lane = [](const auto & before, const auto & after) {
    return std::any_of(before.begin(), before.end(), [&](const auto & a) {
      return std::any_of(after.begin(), after.end(), [&](const auto & b) { return a.id() == b.id(); });
    });
  };
  if (!shares_lane(common_data_ptr_->lanes_ptr->current, current_lanes) ||
      !shares_lane(common_data_ptr_->lanes_ptr->target, target_lanes)) {
    overtake_ = OvertakeState{};
    overtake_prepare_path_.reset();
  }

  common_data_ptr_->lanes_ptr->current = current_lanes;
  common_data_ptr_->lanes_ptr->target = target_lanes;

  const auto & route_handler_ptr = common_data_ptr_->route_handler_ptr;
  common_data_ptr_->current_lanes_path =
    route_handler_ptr->getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());

  common_data_ptr_->target_lanes_path =
    route_handler_ptr->getCenterLinePath(target_lanes, 0.0, std::numeric_limits<double>::max());

  common_data_ptr_->lanes_ptr->target_neighbor = utils::lane_change::get_target_neighbor_lanes(
    *route_handler_ptr, current_lanes, common_data_ptr_->lc_type);

  common_data_ptr_->current_lanes_path =
    route_handler_ptr->getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());
  common_data_ptr_->lanes_ptr->current_lane_in_goal_section =
    route_handler_ptr->isInGoalRouteSection(current_lanes.back());
  common_data_ptr_->lanes_ptr->target_lane_in_goal_section =
    route_handler_ptr->isInGoalRouteSection(target_lanes.back());

  common_data_ptr_->lanes_ptr->preceding_target =
    utils::lane_change::get_preceding_lanes(common_data_ptr_);

  lane_change_debug_.current_lanes = common_data_ptr_->lanes_ptr->current;
  lane_change_debug_.target_lanes = common_data_ptr_->lanes_ptr->target;

  lane_change_debug_.target_backward_lanes.clear();
  ranges::for_each(
    common_data_ptr_->lanes_ptr->preceding_target,
    [&](const lanelet::ConstLanelets & preceding_lanes) {
      ranges::insert(
        lane_change_debug_.target_backward_lanes, lane_change_debug_.target_backward_lanes.end(),
        preceding_lanes);
    });

  *common_data_ptr_->lanes_polygon_ptr = create_lanes_polygon(common_data_ptr_);
  common_data_ptr_->no_lane_change_lines =
    utils::lane_change::get_no_lane_change_lines(common_data_ptr_->lanes_ptr->current, direction_);
}

void NormalLaneChange::update_transient_data(const bool is_approved)
{
  if (
    !common_data_ptr_ || !common_data_ptr_->is_data_available() ||
    !common_data_ptr_->is_lanes_available()) {
    return;
  }

  auto & transient_data = common_data_ptr_->transient_data;

  const auto & p = *common_data_ptr_->bpp_param_ptr;
  const auto nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prev_module_output_.path.points, common_data_ptr_->get_ego_pose(),
      p.ego_nearest_dist_threshold, p.ego_nearest_yaw_threshold);
  transient_data.current_path_velocity =
    prev_module_output_.path.points.at(nearest_seg_idx).point.longitudinal_velocity_mps;
  transient_data.current_path_seg_idx = nearest_seg_idx;

  const auto active_signal_duration =
    signal_activation_time_ ? (clock_.now() - signal_activation_time_.value()).toSec() : 0.0;
  transient_data.lane_change_prepare_duration =
    is_approved ? status_.lane_change_path.info.duration.prepare
                : calculation::calc_actual_prepare_duration(
                    common_data_ptr_, common_data_ptr_->get_ego_speed(), active_signal_duration);

  const auto lc_distance =
    calculation::calc_lc_length_and_dist_buffer(common_data_ptr_, get_current_lanes());
  transient_data.has_valid_lc_distance = lc_distance.has_value();
  if (lc_distance) {
    std::tie(transient_data.lane_changing_length, transient_data.current_dist_buffer) = *lc_distance;
  } else {
    transient_data.lane_changing_length = lane_change::MinMaxValue{0.0, 0.0};
    transient_data.current_dist_buffer = lane_change::MinMaxValue{0.0, 0.0};
    RCLCPP_DEBUG(
      logger_, "No contiguous lateral connection from the ego lane. Skip LC distance update.");
  }

  transient_data.next_dist_buffer.min =
    std::max(0.0, transient_data.current_dist_buffer.min - transient_data.lane_changing_length.min -
    common_data_ptr_->lc_param_ptr->lane_change_finish_judge_buffer);

  transient_data.dist_to_terminal_end = calculation::calc_dist_from_pose_to_terminal_end(
    common_data_ptr_, common_data_ptr_->lanes_ptr->current, common_data_ptr_->get_ego_pose());
  transient_data.dist_to_terminal_start =
    transient_data.dist_to_terminal_end - transient_data.current_dist_buffer.min;

  transient_data.dist_to_target_end = calculation::calc_dist_from_pose_to_terminal_end(
    common_data_ptr_, common_data_ptr_->lanes_ptr->target, common_data_ptr_->get_ego_pose());

  transient_data.max_prepare_length = calculation::calc_maximum_prepare_length(common_data_ptr_);

  transient_data.target_lane_length =
    lanelet::geometry::length3d(lanelet::LaneletSequence(common_data_ptr_->lanes_ptr->target));

  transient_data.current_lanes_ego_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(
      common_data_ptr_->lanes_ptr->current, common_data_ptr_->get_ego_pose());

  transient_data.target_lanes_ego_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    common_data_ptr_->lanes_ptr->target, common_data_ptr_->get_ego_pose());

  transient_data.is_ego_near_current_terminal_start =
    transient_data.dist_to_terminal_start < transient_data.max_prepare_length;

  transient_data.current_footprint = utils::lane_change::get_ego_footprint(
    common_data_ptr_->get_ego_pose(), common_data_ptr_->bpp_param_ptr->vehicle_info);

  transient_data.ego_to_terminal_end_proximity =
    utils::lane_change::calc_polygon_dist_range_from_terminal_end(
      common_data_ptr_->current_lanes_path, transient_data.current_footprint);

  const auto & ego_lane = common_data_ptr_->lanes_ptr->ego_lane;
  const auto & route_handler_ptr = common_data_ptr_->route_handler_ptr;
  transient_data.in_intersection = utils::lane_change::is_within_intersection(
    route_handler_ptr, ego_lane, transient_data.current_footprint);
  transient_data.in_turn_direction_lane =
    utils::lane_change::is_within_turn_direction_lanes(ego_lane, transient_data.current_footprint);

  transient_data.interval_dist_no_lane_change_lines =
    utils::lane_change::get_interval_dist_no_lane_change_lines(
      common_data_ptr_->no_lane_change_lines, common_data_ptr_->current_lanes_path,
      common_data_ptr_->get_ego_pose());

  update_dist_from_intersection();

  updateStopTime();
  transient_data.is_ego_stuck = is_ego_stuck();

  RCLCPP_DEBUG(
    logger_, "lane_changing_length - min: %.4f, max: %.4f", transient_data.lane_changing_length.min,
    transient_data.lane_changing_length.max);
  RCLCPP_DEBUG(
    logger_, "current_dist_buffer - min: %.4f, max: %.4f", transient_data.current_dist_buffer.min,
    transient_data.current_dist_buffer.max);
  RCLCPP_DEBUG(
    logger_, "next_dist_buffer - min: %.4f, max: %.4f", transient_data.next_dist_buffer.min,
    transient_data.next_dist_buffer.max);
  RCLCPP_DEBUG(logger_, "dist_to_terminal_start: %.4f", transient_data.dist_to_terminal_start);
  RCLCPP_DEBUG(logger_, "dist_to_terminal_end: %.4f", transient_data.dist_to_terminal_end);
  RCLCPP_DEBUG(logger_, "max_prepare_length: %.4f", transient_data.max_prepare_length);
  updateOvertake(is_approved);
  RCLCPP_DEBUG(
    logger_, "is_ego_near_current_terminal_start: %s",
    (transient_data.is_ego_near_current_terminal_start ? "true" : "false"));
}

void NormalLaneChange::update_filtered_objects()
{
  filtered_objects_ = filter_objects();
}

void NormalLaneChange::updateLaneChangeStatus()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  safety_diagnostic_samples_.clear();
  safety_rejected_pairs_ = 0;
  search_prepare_metrics_ = search_generated_ = search_checked_ = search_safe_ = 0;
  search_budget_exhausted_ = false;
  ahead_candidates_checked_ = 0;
  other_candidates_checked_ = 0;
  unavailable_slot_candidates_ = 0;
  required_slot_prediction_time_ = 0.0;
  selected_merge_slot_ = "none";
  overtake_feedback_reason_ = "not_checked_this_cycle";
  feedback_candidate_frame_.clear();
  feedback_trajectory_frame_.clear();
  const auto search_start = std::chrono::steady_clock::now();
  const auto [found_valid_path, found_safe_path] = getSafePath(status_.lane_change_path);
  const double search_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_start).count();

  // Update status
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  if (!found_valid_path) {
    status_.lane_change_path = LaneChangePath{};
  }
  status_.lane_change_path.path.header = getRouteHeader();
  RCLCPP_DEBUG(
    logger_, "LC decision side=%s valid=%d safe=%d points=%zu prepare=%.2f shift=%.2f end_v=%.2f "
    "ego_lane=%ld current=%zu target=%zu neighbor=%zu distance_valid=%d "
    "prepare_t=%.2f lc_t=%.2f prepare_a=%.3f lc_a=%.3f",
    direction_ == Direction::LEFT ? "left" : "right", found_valid_path, found_safe_path,
    status_.lane_change_path.path.points.size(), status_.lane_change_path.info.length.prepare,
    status_.lane_change_path.info.length.lane_changing,
    status_.lane_change_path.info.terminal_lane_changing_velocity,
    common_data_ptr_->lanes_ptr->ego_lane.id(), get_current_lanes().size(),
    get_target_lanes().size(), common_data_ptr_->lanes_ptr->target_neighbor.size(),
    common_data_ptr_->transient_data.has_valid_lc_distance,
    status_.lane_change_path.info.duration.prepare,
    status_.lane_change_path.info.duration.lane_changing,
    status_.lane_change_path.info.longitudinal_acceleration.prepare,
    status_.lane_change_path.info.longitudinal_acceleration.lane_changing);
  publishSafetyDiagnostics(search_ms);
}

// Preparation is a receding-horizon decision, not a timer locking an old start pose.
// A safe replacement is committed atomically; a failed search leaves the approved
// maneuver and its RTC identity intact.
void NormalLaneChange::refreshApprovedPlan()
{
  if (!status_.is_valid_path || isAbortState()) return;
  const bool preparing = isEgoOnPreparePhase();
  const auto approved_safety = isApprovedPathSafe();
  const bool unsafe = !approved_safety.is_safe;
  if (!unsafe && !lane_change_parameters_->always_update_approved) {
    rolling_plan_reason_ = "keep_safe_approved_maneuver";
    return;
  }
  const auto previous = status_;
  common_data_ptr_->transient_data.lane_change_prepare_duration =
    calculation::calc_actual_prepare_duration(common_data_ptr_,getEgoVelocity(),
      signal_activation_time_ ? (clock_.now()-*signal_activation_time_).toSec() : 0.0);
  const double remaining_prepare = std::max(0.0, calcSignedArcLength(
    previous.lane_change_path.path.points, getEgoPosition(),
    previous.lane_change_path.info.shift_line.start.position)) / std::max(1.0,getEgoVelocity());
  updateLaneChangeStatus();
  const auto replacement = status_;
  status_ = previous;
  if (!replacement.is_valid_path || !replacement.is_safe) {
    rolling_plan_reason_ = "keep_approved_no_safe_replacement";
    return;
  }
  if (!replacementIsContinuous(replacement.lane_change_path)) {
    rolling_plan_reason_ = "keep_approved_replacement_discontinuous";
    return;
  }
  const bool immediate = replacement.lane_change_path.info.duration.prepare <= 1e-6;
  // Even in the experimental always-update mode a safe start cannot slide
  // into the future at each replan. The unsafe/recovery branches may reschedule.
  if (!unsafe && !execution_replan_requested_ && preparing &&
      replacement.lane_change_path.info.duration.prepare > remaining_prepare + 0.1) {
    rolling_plan_reason_ = "keep_approved_start_deadline";
    return;
  }
  status_ = replacement;
  execution_replan_requested_ = false;
  feedback_recovery_ = {}; ego_recovery_ = {};
  unsafe_hysteresis_count_ = 0;
  rolling_plan_reason_ = immediate ? "replace_with_immediate_safe_lc" : "replace_with_updated_safe_lc";
}

bool NormalLaneChange::replacementIsContinuous(const LaneChangePath & candidate) const
{
  if (candidate.path.points.size()<3) return false;
  const auto & points = candidate.path.points;
  const auto idx = autoware::motion_utils::findNearestIndex(points,getEgoPosition());
  if (idx+1>=points.size()) return false;
  const auto & pose = points[idx].point.pose;
  // This gate constrains a newly joined path, not deviation from stale approval.
  if (autoware_utils::calc_distance2d(pose,getEgoPose())>0.5) return false;
  const double yaw = autoware_utils::normalize_radian(
    tf2::getYaw(pose.orientation)-tf2::getYaw(getEgoPose().orientation));
  if (std::abs(yaw)>autoware_utils::deg2rad(5.0)) return false;
  return std::abs(points[idx].point.longitudinal_velocity_mps-getEgoVelocity())<=1.0;
}

std::tuple<int,double,int,int,double,double> NormalLaneChange::candidateRank(
  const LaneChangePath & candidate) const
{
  const auto & info = candidate.info;
  int slot = 1;
  const ExtendedPredictedObject * nearest = nullptr;
  for (const auto * group : {&filtered_objects_.target_lane_leading.moving,
       &filtered_objects_.target_lane_trailing}) {
    for (const auto & object : *group) {
      if (!nearest || std::abs(object.dist_from_ego)<std::abs(nearest->dist_from_ego)) nearest=&object;
    }
  }
  if (nearest) {
    const auto value = utils::lane_change::predicted_merge_slot(
      common_data_ptr_->current_lanes_path, info.lane_changing_end,
      info.duration.prepare+info.duration.lane_changing, *nearest,
      planner_data_->parameters.vehicle_info);
    slot = value == utils::lane_change::MergeSlot::Ahead ? 0 : 2;
  }
  const double lateral = std::abs(info.lateral_acceleration);
  // Gap first: a feasible delayed front slot beats an immediate rear slot.
  // With no neighboring vehicle all slots are equal, so immediate LC wins.
  const int prepare = info.duration.prepare <= 1e-6 ? 1 :
    utils::lane_change::acceleration_priority(info.longitudinal_acceleration.prepare);
  const int execution = utils::lane_change::acceleration_priority(
    info.longitudinal_acceleration.lane_changing);
  return {slot, info.duration.prepare, prepare, execution, lateral, -info.terminal_lane_changing_velocity};
}

std::pair<bool, bool> NormalLaneChange::getSafePath(LaneChangePath & safe_path) const
{
  const auto & current_lanes = get_current_lanes();
  const auto & target_lanes = get_target_lanes();

  if (current_lanes.empty() || target_lanes.empty()) {
    return {false, false};
  }

  LaneChangePaths valid_paths{};
  bool found_safe_path = get_lane_change_paths(valid_paths);
  // if no safe path is found and ego is stuck, try to find a path with a small margin

  if (valid_paths.empty() && terminal_lane_change_path_) {
    valid_paths.push_back(terminal_lane_change_path_.value());
  }

  lane_change_debug_.valid_paths = valid_paths;

  if (valid_paths.empty()) {
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  if (!utils::lane_change::contains_lane_change_maneuver(safe_path)) {
    RCLCPP_WARN(logger_, "Reject LC candidate: published path does not contain the full maneuver.");
    return {false, false};
  }

  if (found_safe_path) {
    // Validate the same extended, overlap-processed geometry before approval,
    // rather than discovering a missing maneuver after switching to RUNNING.
    if (const auto extension = extendPath(safe_path.path)) {
      safe_path.path = utils::combinePath(safe_path.path, *extension);
    }
    const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
      *getRouteHandler(), get_current_lanes(), get_target_lanes());
    utils::cutOverlappedLanes(safe_path.path, drivable_lanes);
    if (!utils::lane_change::contains_lane_change_maneuver(safe_path)) {
      RCLCPP_ERROR(logger_, "Reject LC approval: overlap processing removed the maneuver.");
      return {false, false};
    }
    const double remaining = calcSignedArcLength(
      safe_path.path.points, getEgoPosition(), safe_path.path.points.back().point.pose.position);
    const double available = calculation::calc_dist_from_pose_to_terminal_end(
      common_data_ptr_, get_target_lanes(), getEgoPose());
    const double required = std::min(getCommonParam().forward_path_length, std::max(0.0, available));
    if (remaining + 1.0 < required) {
      RCLCPP_ERROR(logger_, "Reject LC approval: output %.2f m shorter than available %.2f m.",
        remaining, required);
      return {false, false};
    }
    if (safe_path.cooperative_overtake_target &&
        !check_candidate_path_safety(safe_path, allOvertakeObjects())) {
      return {true, false};
    }
  }

  return {true, found_safe_path};
}

std::optional<std::string> NormalLaneChange::isLaneChangeRequired()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!common_data_ptr_) return {"LC unavailable: common data missing."};
  if (!common_data_ptr_->is_data_available()) {
    return {"LC unavailable: planner data pointers missing."};
  }
  if (common_data_ptr_->lanes_ptr->current.empty()) {
    return {"LC unavailable: current lane sequence empty."};
  }
  if (common_data_ptr_->lanes_ptr->target.empty()) {
    return {"LC unavailable: target lane sequence empty."};
  }
  if (common_data_ptr_->lanes_ptr->target_neighbor.empty()) {
    return {"LC unavailable: target-neighbor lane sequence empty."};
  }

  const auto & current_lanes = common_data_ptr_->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr_->lanes_ptr->target;

  const auto ego_dist_to_target_start =
    calculation::calc_ego_dist_to_lanes_start(common_data_ptr_, current_lanes, target_lanes);
  const auto max_prepare_length = calculation::calc_maximum_prepare_length(common_data_ptr_);

  if (ego_dist_to_target_start > max_prepare_length) {
    return {"Ego is far from target lane start."};
  }

  if (is_near_regulatory_element()) {
    return {"Ego is close to regulatory element."};
  }

  if (
    is_near_terminal_end() && planner_data_ && planner_data_->operation_mode &&
    planner_data_->operation_mode->mode != OperationModeState::AUTONOMOUS) {
    return {"Ego is in MANUAL Mode and near terminal end, don't run LC module."};
  }

  return std::nullopt;
}

bool NormalLaneChange::is_near_regulatory_element() const
{
  if (!common_data_ptr_ || !common_data_ptr_->is_data_available()) return false;

  if (common_data_ptr_->transient_data.is_ego_near_current_terminal_start) return false;

  const bool only_tl = getStopTime() >= lane_change_parameters_->th_stop_time;

  if (only_tl) {
    RCLCPP_DEBUG(logger_, "Stop time is over threshold. Ignore crosswalk and intersection checks.");
  }

  return common_data_ptr_->transient_data.max_prepare_length >
         utils::lane_change::get_distance_to_next_regulatory_element(
           common_data_ptr_, only_tl, only_tl);
}

bool NormalLaneChange::isStoppedAtRedTrafficLight() const
{
  return utils::traffic_light::isStoppedAtRedTrafficLightWithinDistance(
    get_current_lanes(), status_.lane_change_path.path, planner_data_,
    status_.lane_change_path.info.length.sum());
}

TurnSignalInfo NormalLaneChange::get_current_turn_signal_info() const
{
  const auto original_turn_signal_info = prev_module_output_.turn_signal_info;

  if (getModuleType() != LaneChangeModuleType::NORMAL || get_current_lanes().empty()) {
    return original_turn_signal_info;
  }

  if (direction_ != Direction::LEFT && direction_ != Direction::RIGHT) {
    return original_turn_signal_info;
  }

  const auto & prev_path = prev_module_output_.path.points;
  if (prev_path.empty()) {
    return original_turn_signal_info;
  }

  const auto & original_command = original_turn_signal_info.turn_signal.command;
  if (
    original_command != TurnIndicatorsCommand::DISABLE &&
    original_command != TurnIndicatorsCommand::NO_COMMAND) {
    return get_terminal_turn_signal_info();
  }

  if (!status_.is_valid_path) {
    // The request already publishes the LC indicator even without a feasible
    // candidate. Count that preparation time instead of waiting for a candidate
    // whose preparation distance may itself be the reason generation fails.
    set_signal_activation_time();
    return get_turn_signal(getEgoPose(), prev_path.back().point.pose);
  }

  set_signal_activation_time();

  return get_turn_signal(getEgoPose(), getLaneChangePath().info.lane_changing_end);
}

TurnSignalInfo NormalLaneChange::get_terminal_turn_signal_info() const
{
  const auto & lane_change_param = getLaneChangeParam();
  const auto & common_param = getCommonParam();
  const auto & current_pose = getEgoPose();
  const auto & path = prev_module_output_.path;

  const auto original_turn_signal_info = prev_module_output_.turn_signal_info;

  const auto buffer = common_data_ptr_->transient_data.current_dist_buffer.min +
                      lane_change_param.min_length_for_turn_signal_activation +
                      common_param.base_link2front;
  const double path_length = autoware::motion_utils::calcArcLength(path.points);
  const auto start_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    path.points, 0, std::max(path_length - buffer, 0.0));

  if (!start_pose) return original_turn_signal_info;

  const auto terminal_turn_signal_info =
    get_turn_signal(*start_pose, getLaneChangePath().info.lane_changing_end);

  const auto nearest_dist_threshold = common_param.ego_nearest_dist_threshold;
  const auto nearest_yaw_threshold = common_param.ego_nearest_yaw_threshold;
  const auto current_nearest_seg_idx = common_data_ptr_->transient_data.current_path_seg_idx;

  const auto turn_signal_info = getTurnSignalDecider().overwrite_turn_signal(
    path, current_pose, current_nearest_seg_idx, original_turn_signal_info,
    terminal_turn_signal_info, nearest_dist_threshold, nearest_yaw_threshold);

  set_signal_activation_time(
    turn_signal_info.turn_signal.command != terminal_turn_signal_info.turn_signal.command);

  return turn_signal_info;
}

LaneChangePath NormalLaneChange::getLaneChangePath() const
{
  return status_.lane_change_path;
}

BehaviorModuleOutput NormalLaneChange::getTerminalLaneChangePath() const
{
  if (overtake_prepare_path_) {
    auto output = prev_module_output_;
    output.path = *overtake_prepare_path_;
    output.turn_signal_info = get_current_turn_signal_info();
    return output;
  }
  if (
    !lane_change_parameters_->terminal_path.enable ||
    !common_data_ptr_->transient_data.is_ego_near_current_terminal_start) {
    return prev_module_output_;
  }

  const auto is_near_goal = lane_change_parameters_->terminal_path.disable_near_goal &&
                            common_data_ptr_->lanes_ptr->target_lane_in_goal_section &&
                            common_data_ptr_->transient_data.dist_to_target_end <
                              common_data_ptr_->transient_data.lane_changing_length.max;
  if (is_near_goal) {
    return prev_module_output_;
  }

  const auto & current_lanes = get_current_lanes();
  if (current_lanes.empty()) {
    RCLCPP_DEBUG(logger_, "Current lanes not found. Returning previous module's path as output.");
    return prev_module_output_;
  }

  const auto terminal_lc_path = compute_terminal_lane_change_path();

  if (!terminal_lc_path) {
    RCLCPP_DEBUG(logger_, "Terminal path not found. Returning previous module's path as output.");
    return prev_module_output_;
  }

  auto output = prev_module_output_;

  output.path = *terminal_lc_path;
  if (const auto extension = extendPath(output.path)) {
    output.path = utils::combinePath(output.path, *extension);
  }
  output.turn_signal_info = get_current_turn_signal_info();

  extendOutputDrivableArea(output);

  return output;
}

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!status_.is_valid_path) {
    RCLCPP_DEBUG(logger_, "No valid path found. Returning previous module's path as output.");
    insert_stop_point(get_current_lanes(), prev_module_output_.path);
    return prev_module_output_;
  }

  auto output = prev_module_output_;
  if (isAbortState() && abort_path_) {
    output.path = abort_path_->path;
    insert_stop_point(get_current_lanes(), output.path);
  } else {
    bool rejected_shortened_lc_output = false;
    output.path = status_.lane_change_path.path;

    const auto found_extended_path = extendPath();
    if (found_extended_path) {
      output.path = utils::combinePath(output.path, *found_extended_path);
    }

    const auto remaining_length = [&](const PathWithLaneId & candidate) {
      if (candidate.points.empty()) return 0.0;
      return std::max(
        0.0, calcSignedArcLength(
               candidate.points, getEgoPosition(), candidate.points.back().point.pose.position));
    };
    const auto output_remaining = remaining_length(output.path);
    const auto available_remaining = calculation::calc_dist_from_pose_to_terminal_end(
      common_data_ptr_, get_target_lanes(), getEgoPose());
    const auto required_remaining = std::min(
      getCommonParam().forward_path_length, std::max(0.0, available_remaining));
    constexpr double path_length_tolerance = 1.0;
    if (output_remaining + path_length_tolerance < required_remaining) {
      RCLCPP_ERROR(
        logger_,
        "Reject shortened LC output: remaining %.2f m, required %.2f m. Retaining the previous "
        "module path.",
        output_remaining, required_remaining);
      output.path = prev_module_output_.path;
      rejected_shortened_lc_output = true;
    }
    output.reference_path = rejected_shortened_lc_output ? prev_module_output_.reference_path
                                                         : getReferencePath();

    if (isStopState()) {
      const auto current_velocity = getEgoVelocity();
      const auto current_dist = calcSignedArcLength(
        output.path.points, output.path.points.front().point.pose.position, getEgoPosition());
      const auto stop_dist =
        -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));
      set_stop_pose(stop_dist + current_dist, output.path, "incoming rear object");
    } else if (!rejected_shortened_lc_output) {
      insert_stop_point(get_target_lanes(), output.path);
    }
  }

  applyPredictionCoverageStop(output.path);
  extendOutputDrivableArea(output);

  const auto turn_signal_info =
    get_turn_signal(getEgoPose(), status_.lane_change_path.info.lane_changing_end);
  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path.points);
  output.turn_signal_info = getTurnSignalDecider().overwrite_turn_signal(
    output.path, getEgoPose(), current_seg_idx, prev_module_output_.turn_signal_info,
    turn_signal_info, planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  set_signal_activation_time(
    output.turn_signal_info.turn_signal.command != turn_signal_info.turn_signal.command);

  return output;
}

void NormalLaneChange::applyPredictionCoverageStop(PathWithLaneId & path)
{
  if (path.points.empty() || !status_.lane_change_path.prediction_coverage_deferred || isAbortState()) return;
  const double start_arc = calcSignedArcLength(path.points,
    path.points.front().point.pose.position,
    status_.lane_change_path.info.lane_changing_start.position);
  set_stop_pose(std::max(0.0, start_arc - planner_data_->parameters.base_link2front),
    path, "prediction coverage pending before lateral entry");
}

void NormalLaneChange::extendOutputDrivableArea(BehaviorModuleOutput & output) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), get_current_lanes(), get_target_lanes());
  const auto length_before_drivable_area = autoware::motion_utils::calcArcLength(output.path.points);
  const auto shorten_lanes = utils::cutOverlappedLanes(output.path, drivable_lanes);
  const auto length_after_drivable_area = autoware::motion_utils::calcArcLength(output.path.points);
  if (length_after_drivable_area + 1.0 < length_before_drivable_area) {
    std::ostringstream groups;
    for (const auto & lanes : drivable_lanes) {
      groups << '[' << lanes.left_lane.id() << ',' << lanes.right_lane.id();
      for (const auto & lane : lanes.middle_lanes) groups << ',' << lane.id();
      groups << ']';
    }
    RCLCPP_ERROR(logger_, "LC drivable-area path truncation: %.2f -> %.2f m; groups=%s",
      length_before_drivable_area, length_after_drivable_area, groups.str().c_str());
  }
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // for new architecture
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, prev_module_output_.drivable_area_info);
}

void NormalLaneChange::insert_stop_point(
  const lanelet::ConstLanelets & lanelets, PathWithLaneId & path, const bool is_waiting_approval)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (lanelets.empty()) {
    return;
  }

  const auto & route_handler = getRouteHandler();

  if (route_handler->getNumLaneToPreferredLane(lanelets.back()) == 0) {
    return;
  }

  const auto & current_lanes = get_current_lanes();
  const auto is_current_lane = lanelets.front().id() == current_lanes.front().id() &&
                               lanelets.back().id() == current_lanes.back().id();

  // if input is not current lane, we can just insert the points at terminal.
  if (!is_current_lane) {
    if (common_data_ptr_->transient_data.next_dist_buffer.min < calculation::eps) {
      return;
    }
    const auto arc_length_to_stop_pose = motion_utils::calcArcLength(path.points) -
                                         common_data_ptr_->transient_data.next_dist_buffer.min;
    set_stop_pose(arc_length_to_stop_pose, path, "next lc terminal");
    return;
  }

  insert_stop_point_on_current_lanes(path, is_waiting_approval);
}

void NormalLaneChange::insert_stop_point_on_current_lanes(
  PathWithLaneId & path, const bool is_waiting_approval)
{
  const auto & path_front_pose = path.points.front().point.pose;
  const auto & ego_pose = common_data_ptr_->get_ego_pose();
  const auto dist_from_path_front =
    motion_utils::calcSignedArcLength(path.points, path_front_pose.position, ego_pose.position);

  const auto & transient_data = common_data_ptr_->transient_data;
  const auto & lanes_ptr = common_data_ptr_->lanes_ptr;
  const auto & lc_param_ptr = common_data_ptr_->lc_param_ptr;

  const auto dist_to_terminal_start =
    transient_data.dist_to_terminal_start - calculation::calc_stopping_distance(lc_param_ptr);

  const auto & bpp_param_ptr = common_data_ptr_->bpp_param_ptr;
  const auto dist_to_terminal_stop = std::invoke([&]() -> double {
    const auto & curr_lanes_poly = common_data_ptr_->lanes_polygon_ptr->current;
    if (!utils::isEgoWithinOriginalLane(curr_lanes_poly, getEgoPose(), *bpp_param_ptr)) {
      return dist_from_path_front + dist_to_terminal_start;
    }

    if (
      terminal_lane_change_path_ && is_waiting_approval &&
      lc_param_ptr->terminal_path.stop_at_boundary) {
      return calculation::calc_dist_to_last_fit_width(common_data_ptr_, path);
    }

    const auto dist_to_last_fit_width = calculation::calc_dist_to_last_fit_width(
      lanes_ptr->current, path.points.front().point.pose, *bpp_param_ptr);

    return std::min(dist_from_path_front + dist_to_terminal_start, dist_to_last_fit_width);
  });

  const auto terminal_stop_reason = status_.is_valid_path ? "no safe path" : "no valid path";
  if (
    filtered_objects_.current_lane.empty() ||
    !lane_change_parameters_->enable_stopped_vehicle_buffer) {
    set_stop_pose(dist_to_terminal_stop, path, terminal_stop_reason);
    return;
  }

  const auto & center_line = common_data_ptr_->current_lanes_path.points;
  const auto dist_to_target_lane_start = std::invoke([&]() -> double {
    const auto & front_lane = lanes_ptr->target_neighbor.front();
    const auto target_front =
      utils::to_geom_msg_pose(front_lane.centerline2d().front(), front_lane);
    return motion_utils::calcSignedArcLength(
      center_line, path_front_pose.position, target_front.position);
  });

  const auto arc_length_to_current_obj = utils::lane_change::get_min_dist_to_current_lanes_obj(
    common_data_ptr_, filtered_objects_, dist_to_target_lane_start, path);

  // margin with leading vehicle
  // consider rss distance when the LC need to avoid obstacles
  const auto rss_dist = calcRssDistance(
    0.0, lc_param_ptr->trajectory.min_lane_changing_velocity,
    lc_param_ptr->safety.rss_params_for_parked);

  const auto stop_margin = transient_data.lane_changing_length.min +
                           lc_param_ptr->backward_length_buffer_for_blocking_object + rss_dist +
                           bpp_param_ptr->base_link2front;
  const auto stop_arc_length_behind_obj = arc_length_to_current_obj - stop_margin;

  if (stop_arc_length_behind_obj < dist_to_target_lane_start) {
    set_stop_pose(dist_to_target_lane_start, path, "maintain distance to front object");
    return;
  }

  if (stop_arc_length_behind_obj > dist_to_terminal_stop) {
    set_stop_pose(dist_to_terminal_stop, path, terminal_stop_reason);
    return;
  }

  //  If the target lane in the lane change section is blocked by a stationary obstacle, there
  //  is no reason for stopping with a lane change margin. Instead, stop right behind the
  //  obstacle.
  //  ----------------------------------------------------------
  //                            [obj]>
  //  ----------------------------------------------------------
  //    [ego]>          | <--- stop margin --->  [obj]>
  //  ----------------------------------------------------------
  const auto has_blocking_target_lane_obj = utils::lane_change::has_blocking_target_object(
    filtered_objects_.target_lane_leading, stop_arc_length_behind_obj, path);

  if (has_blocking_target_lane_obj || stop_arc_length_behind_obj <= 0.0) {
    set_stop_pose(dist_to_terminal_stop, path, terminal_stop_reason);
    return;
  }

  set_stop_pose(stop_arc_length_behind_obj, path, "maintain distance to front object");
}

PathWithLaneId NormalLaneChange::getReferencePath() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto closest_lanelet_opt =
    autoware::experimental::lanelet2_utils::get_closest_lanelet(get_target_lanes(), getEgoPose());
  if (!closest_lanelet_opt) {
    return prev_module_output_.reference_path;
  }
  const auto & closest_lanelet = closest_lanelet_opt.value();
  auto reference_path = utils::getCenterLinePathFromLanelet(closest_lanelet, planner_data_);
  if (reference_path.points.empty()) {
    return prev_module_output_.reference_path;
  }
  return reference_path;
}

std::optional<PathWithLaneId> NormalLaneChange::extendPath()
{
  return extendPath(status_.lane_change_path.path);
}

std::optional<PathWithLaneId> NormalLaneChange::extendPath(const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (path.points.empty()) {
    return std::nullopt;
  }

  auto & target_lanes = common_data_ptr_->lanes_ptr->target;
  if (target_lanes.empty()) {
    return std::nullopt;
  }

  const auto forward_path_length = getCommonParam().forward_path_length;
  const auto remaining_path_length = std::max(
    0.0, calcSignedArcLength(
           path.points, getEgoPosition(), path.points.back().point.pose.position));
  const auto required_extension = forward_path_length - remaining_path_length;
  if (required_extension <= calculation::eps) {
    return std::nullopt;
  }

  const auto dist_to_end_of_path = autoware::experimental::lanelet2_utils::get_arc_coordinates(
                                     target_lanes, path.points.back().point.pose)
                                     .length;

  auto terminal_arc = [&]() {
    if (getRouteHandler()->isInGoalRouteSection(target_lanes.back())) {
      return autoware::experimental::lanelet2_utils::get_arc_coordinates(
               target_lanes, getRouteHandler()->getGoalPose())
        .length;
    }
    return lanelet::geometry::length3d(lanelet::LaneletSequence(target_lanes));
  };

  auto dist_to_terminal = terminal_arc();
  std::unordered_set<lanelet::Id> visited_lane_ids;
  ranges::for_each(target_lanes, [&](const auto & lane) { visited_lane_ids.insert(lane.id()); });
  while (dist_to_terminal - dist_to_end_of_path + calculation::eps < required_extension) {
    if (getRouteHandler()->isInGoalRouteSection(target_lanes.back())) {
      break;
    }

    lanelet::ConstLanelets successors;
    if (!getRouteHandler()->getNextLaneletsWithinRoute(target_lanes.back(), &successors)) {
      break;
    }
    // Only choose among real longitudinal successors, never jump laterally
    // to a preferred lane. Keep route preference deterministic at a split.
    const auto preferred = getRouteHandler()->getPreferredLanelets();
    const auto preferred_next = std::find_if(successors.begin(), successors.end(), [&](const auto & s) {
      return std::any_of(preferred.begin(), preferred.end(), [&](const auto & p) {
        return p.id() == s.id();
      });
    });
    const auto next_lane = preferred_next == successors.end() ? successors.front() : *preferred_next;
    if (!visited_lane_ids.insert(next_lane.id()).second) {
      RCLCPP_ERROR(logger_, "Detected a route loop while extending an LC output path.");
      break;
    }
    target_lanes.push_back(next_lane);
    dist_to_terminal = terminal_arc();
  }

  const auto extension_end =
    std::min(dist_to_end_of_path + required_extension, dist_to_terminal);
  if (extension_end <= dist_to_end_of_path + calculation::eps) {
    return std::nullopt;
  }

  return getRouteHandler()->getCenterLinePath(target_lanes, dist_to_end_of_path, extension_end);
}

void NormalLaneChange::resetParameters()
{
  overtake_ = OvertakeState{};
  overtake_prepare_path_.reset();
  feedback_recovery_ = {}; ego_recovery_ = {};
  recovery_maneuver_start_.reset(); recovery_maneuver_end_.reset();
  terminal_lane_change_path_.reset();
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;
  status_ = LaneChangeStatus();
  unsafe_hysteresis_count_ = 0;
  lane_change_debug_.reset();
  set_signal_activation_time(true);

  RCLCPP_DEBUG(logger_, "reset all flags and debug information.");
}

TurnSignalInfo NormalLaneChange::updateOutputTurnSignal() const
{
  // The ROS1 provider already owns deterministic arbitration.  The upstream
  // shifted-path discovery method is not yet part of PlannerData; retain the
  // lane-change's own start/end signal instead of fabricating a neutral signal.
  return get_turn_signal(getEgoPose(), status_.lane_change_path.info.lane_changing_end);
}

lanelet::ConstLanelets NormalLaneChange::get_lane_change_lanes(
  const lanelet::ConstLanelets & current_lanes) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  const auto & route_handler = getRouteHandler();

  const auto lane_change_lane =
    utils::lane_change::get_lane_change_target_lane(common_data_ptr_, current_lanes);

  if (!lane_change_lane) {
    return {};
  }

  const auto forward_length = std::invoke([&]() {
    const auto front_pose = utils::to_geom_msg_pose(
      lane_change_lane->centerline().front().basicPoint(), *lane_change_lane);
    const auto signed_distance = utils::getSignedDistance(front_pose, getEgoPose(), current_lanes);
    const auto forward_path_length = planner_data_->parameters.forward_path_length;
    return forward_path_length + std::max(signed_distance, 0.0);
  });

  return route_handler->getLaneletSequence(
    lane_change_lane.value(), getEgoPose(), 0.0, forward_length);
}

bool NormalLaneChange::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_end = status_.lane_change_path.info.shift_line.end;
  const auto & target_lanes = get_target_lanes();
  const double dist_to_lane_change_end =
    utils::getSignedDistance(current_pose, lane_change_end, target_lanes);

  const auto finish_judge_buffer = std::invoke([&]() {
    const double ego_velocity = getEgoVelocity();
    // If ego velocity is low, relax finish judge buffer
    if (std::abs(ego_velocity) < 1.0) {
      return 0.0;
    }
    return lane_change_parameters_->lane_change_finish_judge_buffer;
  });

  const auto has_passed_end_pose = dist_to_lane_change_end + finish_judge_buffer < 0.0;

  lane_change_debug_.distance_to_lane_change_finished =
    dist_to_lane_change_end + finish_judge_buffer;

  if (has_passed_end_pose) {
    const auto & lanes_polygon = common_data_ptr_->lanes_polygon_ptr->target;
    return !boost::geometry::disjoint(
      lanes_polygon,
      lanelet::utils::to2D(experimental::lanelet2_utils::from_ros(current_pose.position)));
  }

  const auto yaw_deviation_to_centerline =
    utils::lane_change::calc_angle_to_lanelet_segment(target_lanes, current_pose);

  if (yaw_deviation_to_centerline > lane_change_parameters_->th_finish_judge_yaw_diff) {
    return false;
  }

  const auto & arc_length = common_data_ptr_->transient_data.target_lanes_ego_arc;
  const auto reach_target_lane =
    std::abs(arc_length.distance) < lane_change_parameters_->th_finish_judge_lateral_diff;

  lane_change_debug_.distance_to_lane_change_finished = arc_length.distance;

  return reach_target_lane;
}

bool NormalLaneChange::isAbleToReturnCurrentLane() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    lane_change_debug_.is_able_to_return_to_current_lane = false;
    return false;
  }

  const auto & curr_lanes_poly = common_data_ptr_->lanes_polygon_ptr->current;
  if (!utils::isEgoWithinOriginalLane(
        curr_lanes_poly, getEgoPose(), planner_data_->parameters,
        lane_change_parameters_->cancel.overhang_tolerance)) {
    lane_change_debug_.is_able_to_return_to_current_lane = false;
    return false;
  }

  // The old gate projected one second along the OUTBOUND LC and treated its
  // departure as proof that a RETURN was impossible. Test the current body
  // here; calcAbortPath validates the actual return geometry/dynamics/objects.
  lane_change_debug_.is_able_to_return_to_current_lane = true;
  return true;
}

bool NormalLaneChange::is_near_terminal_end() const
{
  if (!common_data_ptr_ || !common_data_ptr_->is_data_available()) {
    return true;
  }

  // TODO(Azu) fully change to transient data
  const auto & lc_param_ptr = common_data_ptr_->lc_param_ptr;
  const auto backward_buffer = calculation::calc_stopping_distance(lc_param_ptr);

  const auto current_min_dist_buffer = common_data_ptr_->transient_data.current_dist_buffer.min;
  const auto min_lc_dist_with_buffer = backward_buffer + current_min_dist_buffer;

  return common_data_ptr_->transient_data.dist_to_terminal_end < min_lc_dist_with_buffer;
}

bool NormalLaneChange::isEgoOnPreparePhase() const
{
  const auto & start_position = status_.lane_change_path.info.shift_line.start.position;
  const auto & path_points = status_.lane_change_path.path.points;
  return calcSignedArcLength(path_points, start_position, getEgoPosition()) < 0.0;
}

bool NormalLaneChange::isAbleToStopSafely() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const auto current_velocity = getEgoVelocity();
  const auto stop_dist =
    -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));

  const auto & curr_lanes_poly = common_data_ptr_->lanes_polygon_ptr->current;
  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > stop_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::isEgoWithinOriginalLane(
        curr_lanes_poly, estimated_pose, planner_data_->parameters);
    }
  }
  return true;
}

bool NormalLaneChange::hasFinishedAbort() const
{
  if (!abort_path_) {
    return true;
  }

  const auto distance_to_finish = calcSignedArcLength(
    abort_path_->path.points, getEgoPosition(), abort_path_->info.shift_line.end.position);
  lane_change_debug_.distance_to_abort_finished = distance_to_finish;

  const auto has_finished_abort = distance_to_finish < 0.0;

  return has_finished_abort;
}

bool NormalLaneChange::isAbortState() const
{
  if (!lane_change_parameters_->cancel.enable_on_lane_changing_phase) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  return abort_path_ != nullptr;
}

lane_change::TargetObjects NormalLaneChange::get_target_objects(
  const FilteredLanesObjects & filtered_objects,
  [[maybe_unused]] const lanelet::ConstLanelets & current_lanes) const
{
  if (lane_change_parameters_->overtake.enable && (overtake_.target || isAbortState())) {
    return allOvertakeObjects();
  }
  ExtendedPredictedObjects leading_objects = filtered_objects.target_lane_leading.moving;
  auto insert_leading_objects = [&](const auto & objects) {
    ranges::actions::insert(leading_objects, leading_objects.end(), objects);
  };

  insert_leading_objects(filtered_objects.target_lane_leading.stopped);
  insert_leading_objects(filtered_objects.target_lane_leading.stopped_at_bound);
  // LC owns longitudinal planning after approval: the original lane's
  // vehicles must be checked before Cruise can relinquish following control.
  const bool chk_obj_in_curr_lanes = true;
  if (chk_obj_in_curr_lanes || common_data_ptr_->transient_data.is_ego_stuck) {
    insert_leading_objects(filtered_objects.current_lane);
  }

  const auto chk_obj_in_other_lanes =
    lane_change_parameters_->safety.collision_check.check_other_lanes;
  if (chk_obj_in_other_lanes) {
    insert_leading_objects(filtered_objects.others);
  }

  return {leading_objects, filtered_objects.target_lane_trailing};
}

FilteredLanesObjects NormalLaneChange::filter_objects() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  auto objects = *planner_data_->dynamic_object;
  utils::path_safety_checker::filterObjectsByClass(
    objects, lane_change_parameters_->safety.target_object_types);

  if (objects.objects.empty()) {
    return {};
  }

  filterOncomingObjects(objects);

  if (objects.objects.empty()) {
    return {};
  }

  if (!common_data_ptr_->is_lanes_available()) {
    return {};
  }

  const auto & current_pose = common_data_ptr_->get_ego_pose();
  const auto & current_lanes = common_data_ptr_->lanes_ptr->current;

  const auto & current_lanes_ref_path = common_data_ptr_->current_lanes_path;

  const auto & lanes_polygon = *common_data_ptr_->lanes_polygon_ptr;
  const auto dist_ego_to_current_lanes_center =
    autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
      current_lanes, current_pose);

  FilteredLanesObjects filtered_objects;
  const auto reserve_size = objects.objects.size();
  filtered_objects.current_lane.reserve(reserve_size);
  auto & target_lane_leading = filtered_objects.target_lane_leading;
  target_lane_leading.stopped.reserve(reserve_size);
  target_lane_leading.moving.reserve(reserve_size);
  target_lane_leading.stopped_at_bound.reserve(reserve_size);
  filtered_objects.target_lane_trailing.reserve(reserve_size);
  filtered_objects.others.reserve(reserve_size);

  for (const auto & object : objects.objects) {
    auto ext_object = utils::lane_change::transform(object, *lane_change_parameters_);
    const auto & ext_obj_pose = ext_object.initial_pose;
    ext_object.dist_from_ego = autoware::motion_utils::calcSignedArcLength(
      current_lanes_ref_path.points, current_pose.position, ext_obj_pose.position);

    const auto is_before_terminal =
      utils::lane_change::is_before_terminal(common_data_ptr_, current_lanes_ref_path, ext_object);

    const auto ego_object_proximity = utils::lane_change::calc_ego_object_proximity(
      common_data_ptr_, current_lanes_ref_path, ext_object);

    const auto ahead_of_ego = ego_object_proximity.is_ahead_of_ego;

    if (utils::lane_change::filter_target_lane_objects(
          common_data_ptr_, ext_object, dist_ego_to_current_lanes_center, ego_object_proximity,
          is_before_terminal, target_lane_leading, filtered_objects.target_lane_trailing)) {
      continue;
    }

    if (
      ahead_of_ego && is_before_terminal &&
      !boost::geometry::disjoint(ext_object.initial_polygon, lanes_polygon.current)) {
      // check only the objects that are in front of the ego vehicle
      filtered_objects.current_lane.push_back(ext_object);
      continue;
    }

    filtered_objects.others.push_back(ext_object);
  }

  const auto dist_comparator = [](const auto & obj1, const auto & obj2) {
    return obj1.dist_from_ego < obj2.dist_from_ego;
  };

  // There are no use cases for other lane objects yet, so to save some computation time, we dont
  // have to sort them.
  ranges::sort(filtered_objects.current_lane, dist_comparator);
  ranges::sort(target_lane_leading.stopped_at_bound, dist_comparator);
  ranges::sort(target_lane_leading.stopped, dist_comparator);
  ranges::sort(target_lane_leading.moving, dist_comparator);
  ranges::sort(filtered_objects.target_lane_trailing, [](const auto & obj1, const auto & obj2) {
    return obj2.dist_from_ego < obj1.dist_from_ego;
  });

  lane_change_debug_.filtered_objects = filtered_objects;

  return filtered_objects;
}

void NormalLaneChange::filterOncomingObjects(PredictedObjects & objects) const
{
  const auto & current_pose = getEgoPose();

  const auto is_same_direction = [&](const PredictedObject & object) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    return !utils::path_safety_checker::isTargetObjectOncoming(
      current_pose, object_pose,
      common_data_ptr_->lc_param_ptr->safety.collision_check.th_incoming_object_yaw);
  };

  //  Perception noise could make stationary objects seem opposite the ego vehicle; check the
  //  velocity to prevent this.
  const auto is_stopped_object = [](const auto & object) -> bool {
    constexpr double min_vel_th = -0.5;
    constexpr double max_vel_th = 0.5;
    return velocity_filter(
      object.kinematics.initial_twist_with_covariance.twist, min_vel_th, max_vel_th);
  };

  utils::path_safety_checker::filterObjects(objects, [&](const PredictedObject & object) {
    const auto same_direction = is_same_direction(object);
    if (same_direction) {
      return true;
    }

    return is_stopped_object(object);
  });
}

std::vector<LaneChangePhaseMetrics> NormalLaneChange::get_prepare_metrics() const
{
  const auto & current_lanes = common_data_ptr_->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr_->lanes_ptr->target;
  const auto current_velocity = getEgoVelocity();
  // set speed limit to be current path velocity;
  const auto max_path_velocity = common_data_ptr_->transient_data.current_path_velocity;

  const auto dist_to_target_start =
    calculation::calc_ego_dist_to_lanes_start(common_data_ptr_, current_lanes, target_lanes);
  return calculation::calc_prepare_phase_metrics(
    common_data_ptr_, current_velocity, max_path_velocity, dist_to_target_start,
    common_data_ptr_->transient_data.dist_to_terminal_start);
}

std::vector<LaneChangePhaseMetrics> NormalLaneChange::get_lane_changing_metrics(
  const PathWithLaneId & prep_segment, const LaneChangePhaseMetrics & prep_metric,
  const double shift_length, const double dist_to_reg_element,
  lane_change::MetricsDebug & debug_metrics) const
{
  const auto & transient_data = common_data_ptr_->transient_data;
  const auto dist_lc_start_to_end_of_lanes = calculation::calc_dist_from_pose_to_terminal_end(
    common_data_ptr_, common_data_ptr_->lanes_ptr->target_neighbor,
    prep_segment.points.back().point.pose);

  const auto max_lane_changing_length = std::invoke([&]() {
    double max_length =
      transient_data.is_ego_near_current_terminal_start
        ? transient_data.dist_to_terminal_end - prep_metric.length
        : std::min(transient_data.dist_to_terminal_end, dist_to_reg_element) - prep_metric.length;
    max_length =
      std::min(max_length, dist_lc_start_to_end_of_lanes - transient_data.next_dist_buffer.min);
    return max_length;
  });

  debug_metrics.max_lane_changing_length = max_lane_changing_length;
  // A preparation endpoint is not a speed limit for the whole maneuver. In
  // immediate-LC mode construct_candidate_path applies the spatial road limits
  // and acceleration envelope, then predicts collisions using that same profile.
  const auto max_path_velocity =
    common_data_ptr_->lc_param_ptr->trajectory.min_prepare_duration <= 1e-6
      ? common_data_ptr_->bpp_param_ptr->max_vel
      : prep_segment.points.back().point.longitudinal_velocity_mps;
  if (common_data_ptr_->lc_param_ptr->trajectory.min_prepare_duration <= 1e-6 &&
      prep_metric.duration > 1e-6 &&
      !common_data_ptr_->lc_param_ptr->trajectory.compact_sampling) {
    // Preparation can be speed-limited while the execution road permits
    // acceleration. Do not reuse the preparation's (often zero) acceleration
    // as the only execution sample.
    std::vector<LaneChangePhaseMetrics> metrics;
    for (const double acceleration : calculation::calc_lon_acceleration_samples(
           common_data_ptr_, max_path_velocity, 0.0)) {
      const auto samples = calculation::calc_shift_phase_metrics(
        common_data_ptr_, shift_length, prep_metric.velocity, max_path_velocity,
        acceleration, max_lane_changing_length);
      metrics.insert(metrics.end(), samples.begin(), samples.end());
    }
    return metrics;
  }
  return calculation::calc_shift_phase_metrics(
    common_data_ptr_, shift_length, prep_metric.velocity, max_path_velocity,
    prep_metric.sampled_lon_accel, max_lane_changing_length);
}

bool NormalLaneChange::get_lane_change_paths(LaneChangePaths & candidate_paths) const
{
  lane_change_debug_.collision_check_objects.clear();
  lane_change_debug_.lane_change_metrics.clear();

  if (!common_data_ptr_->is_lanes_available()) {
    RCLCPP_WARN(logger_, "lanes are not available. Not expected.");
    return false;
  }

  if (!common_data_ptr_->transient_data.has_valid_lc_distance) {
    RCLCPP_DEBUG(logger_, "LC distance is unavailable. Skip candidate generation.");
    return false;
  }

  if (common_data_ptr_->lanes_polygon_ptr->target_neighbor.empty()) {
    RCLCPP_WARN(logger_, "target_lane_neighbors_polygon_2d is empty. Not expected.");
    return false;
  }

  const auto & current_lanes = get_current_lanes();

  const auto target_objects = get_target_objects(filtered_objects_, current_lanes);

  auto prepare_phase_metrics = utils::lane_change::diversify_prepare_metrics(get_prepare_metrics());
  if (lane_change_parameters_->trajectory.compact_sampling) {
    double front = lane_change_parameters_->trajectory.max_longitudinal_acc;
    double rear = std::max(-2.0, lane_change_parameters_->trajectory.min_longitudinal_acc);
    // Relative motion is only a sampling hint. Every relevant object and every
    // prediction is still checked by the unchanged hard safety checker.
    const ExtendedPredictedObject * nearest = nullptr;
    for (const auto * group : {&filtered_objects_.target_lane_leading.moving,
         &filtered_objects_.target_lane_trailing}) {
      for (const auto & object : *group) {
        if (!nearest || std::abs(object.dist_from_ego)<std::abs(nearest->dist_from_ego)) nearest=&object;
      }
    }
    if (nearest) {
      const double horizon = std::max(1.0, 0.5 * lane_change_parameters_->trajectory.max_prepare_duration + 3.0);
      const double drift = nearest->dist_from_ego +
        (nearest->initial_twist.linear.x-getEgoVelocity())*horizon;
      const double gap = planner_data_->parameters.vehicle_length +
        lane_change_parameters_->safety.rss_params.longitudinal_distance_min_threshold;
      front = std::clamp(2.0*(drift+gap)/(horizon*horizon), 0.0, front);
      rear = std::clamp(2.0*(drift-gap)/(horizon*horizon),
        lane_change_parameters_->trajectory.min_longitudinal_acc, 0.0);
    }
    prepare_phase_metrics = utils::lane_change::compact_prepare_metrics(prepare_phase_metrics,front,rear);
    const auto execution_grid = calculation::calc_lon_acceleration_samples(
      common_data_ptr_, planner_data_->parameters.max_vel, 0.0);
    std::vector<LaneChangePhaseMetrics> execution_metrics;
    for (double a : execution_grid) execution_metrics.emplace_back(0, 0, 0, a, 0, 0);
    const auto representatives = utils::lane_change::compact_prepare_metrics(execution_metrics,
      lane_change_parameters_->trajectory.max_longitudinal_acc,
      std::max(-2.0, lane_change_parameters_->trajectory.min_longitudinal_acc));
    std::vector<double> execution_samples;
    for (const auto & metric : representatives) execution_samples.push_back(metric.sampled_lon_accel);
    prepare_phase_metrics = utils::lane_change::independent_execution_metrics(
      prepare_phase_metrics, execution_samples);
  }
  search_prepare_metrics_ = prepare_phase_metrics.size();

  const auto sorted_lane_ids = utils::lane_change::get_sorted_lane_ids(common_data_ptr_);
  const auto search = [&](const CandidateSearchMode mode, const double budget_ms) {
    if (common_data_ptr_->lc_param_ptr->frenet.enable &&
        common_data_ptr_->transient_data.is_ego_near_current_terminal_start) {
      return get_path_using_frenet(
        prepare_phase_metrics, target_objects, sorted_lane_ids, candidate_paths, mode, budget_ms);
    }
    return get_path_using_path_shifter(
      prepare_phase_metrics, target_objects, sorted_lane_ids, candidate_paths, mode, budget_ms);
  };
  const double budget_ms = lane_change_parameters_->time_limit;
  const bool has_neighbor = !filtered_objects_.target_lane_leading.moving.empty() ||
    !filtered_objects_.target_lane_trailing.empty();
  if (!findOvertakeTarget() && !has_neighbor) {
    const bool safe = search(CandidateSearchMode::Any, budget_ms);
    if (safe) selected_merge_slot_ = "ordinary";
    return safe;
  }
  // Prefer a safe front slot, but reserve half of the existing cycle budget for
  // the rear/ordinary alternatives. A difficult front search must not starve them.
  const auto started = std::chrono::steady_clock::now();
  if (search(CandidateSearchMode::Ahead, budget_ms * 0.5)) {
    selected_merge_slot_ = "ahead";
    return true;
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  if (search(CandidateSearchMode::Other, std::max(0.0, budget_ms - elapsed_ms))) {
    selected_merge_slot_ = "rear_or_ordinary";
    return true;
  }
  return false;
}

bool NormalLaneChange::get_path_using_frenet(
  const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
  const lane_change::TargetObjects & target_objects,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids,
  LaneChangePaths & candidate_paths, const CandidateSearchMode mode,
  const double time_limit_ms) const
{
  stop_watch_.tic(__func__);
  std::optional<LaneChangePath> best_safe;
  lane_change_debug_.frenet_states.clear();
  size_t generated_count = 0;
  // Do not generate hundreds of late-start alternatives before checking the
  // immediate ones. Bound each batch and stop as soon as a safe path is found.
  std::vector<std::vector<LaneChangePhaseMetrics>> batches;
  if (lane_change_parameters_->trajectory.min_prepare_duration <= 1e-6) {
    for (const auto & metric : prepare_metrics) batches.push_back({metric});
  } else {
    batches.push_back(prepare_metrics);
  }
  for (const auto & batch : batches) {
    if (stop_watch_.toc(__func__) >= time_limit_ms) { search_budget_exhausted_=true; break; }
    const auto frenet_candidates = utils::lane_change::generate_frenet_candidates(
      common_data_ptr_, prev_module_output_.path, batch);
    generated_count += frenet_candidates.size();
    search_generated_ += frenet_candidates.size();
    RCLCPP_DEBUG(
      logger_, "Generated %lu candidate paths in %2.2f[ms]", frenet_candidates.size(),
      stop_watch_.toc(__func__));

    candidate_paths.reserve(frenet_candidates.size());
    lane_change_debug_.frenet_states.reserve(generated_count);
    for (const auto & frenet_candidate : frenet_candidates) {
      if (stop_watch_.toc(__func__) >= time_limit_ms) {
        search_budget_exhausted_=true;
        break;
      }

      lane_change_debug_.frenet_states.emplace_back(
        frenet_candidate.prepare_metric, frenet_candidate.lane_changing.sampling_parameter,
        frenet_candidate.lc_average_curvature, frenet_candidate.max_lane_changing_length);

      std::optional<LaneChangePath> candidate_path_opt;
      try {
        candidate_path_opt =
          utils::lane_change::get_candidate_path(frenet_candidate, common_data_ptr_, sorted_lane_ids);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(logger_, "%s", e.what());
      }

      if (!candidate_path_opt) {
        continue;
      }

      if (!prepareCandidateForSearch(*candidate_path_opt, mode)) continue;

      try {
        ++search_checked_;
        if (check_candidate_path_safety(*candidate_path_opt, target_objects)) {
          ++search_safe_;
          RCLCPP_DEBUG(
            logger_, "Found safe path after %lu candidate(s). Total time: %2.2f[ms]",
            frenet_candidates.size(), stop_watch_.toc(__func__));
          utils::lane_change::append_target_ref_to_candidate(
            *candidate_path_opt, common_data_ptr_->lc_param_ptr->frenet.th_curvature_smoothing);
          if (!best_safe || candidateRank(*candidate_path_opt)<candidateRank(*best_safe)) {
            best_safe = *candidate_path_opt;
          }
          continue;
        }
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(logger_, "%s", e.what());
      }

      // appending all paths affect performance
      if (candidate_paths.empty()) {
        candidate_paths.push_back(*candidate_path_opt);
      }
    }
  }

  RCLCPP_DEBUG(
    logger_, "No safe path after %lu candidate(s). Total time: %2.2f[ms]", generated_count,
    stop_watch_.toc(__func__));
  if (best_safe) { candidate_paths.push_back(*best_safe); return true; }
  return false;
}

bool NormalLaneChange::get_path_using_path_shifter(
  const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
  const lane_change::TargetObjects & target_objects,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids,
  LaneChangePaths & candidate_paths, const CandidateSearchMode mode,
  const double time_limit_ms) const
{
  stop_watch_.tic(__func__);
  std::optional<LaneChangePath> best_safe;
  const auto & target_lanes = get_target_lanes();
  candidate_paths.reserve(
    prepare_metrics.size() * lane_change_parameters_->trajectory.lat_acc_sampling_num);

  const bool only_tl = getStopTime() >= lane_change_parameters_->th_stop_time;
  const auto dist_to_next_regulatory_element =
    utils::lane_change::get_distance_to_next_regulatory_element(common_data_ptr_, only_tl, only_tl);

  auto check_length_diff =
    [&](const double prep_length, const double lc_length, const bool check_lc) {
      if (candidate_paths.empty()) return true;

      const auto prep_diff = std::abs(candidate_paths.back().info.length.prepare - prep_length);
      if (prep_diff > lane_change_parameters_->trajectory.th_prepare_length_diff) return true;

      if (!check_lc) return false;

      const auto lc_diff = std::abs(candidate_paths.back().info.length.lane_changing - lc_length);
      return lc_diff > lane_change_parameters_->trajectory.th_lane_changing_length_diff;
    };

  for (const auto & prep_metric : prepare_metrics) {
    const auto debug_print = [&](const std::string & s) {
      RCLCPP_DEBUG(
        logger_, "%s | prep_time: %.5f | lon_acc: %.5f | prep_len: %.5f", s.c_str(),
        prep_metric.duration, prep_metric.actual_lon_accel, prep_metric.length);
    };

    // Equal preparation lengths (especially zero) do not imply equal LC
    // maneuvers: longitudinal samples can change the execution phase. Apply
    // the full prepare+LC length check below instead of dropping them here.

    PathWithLaneId prepare_segment;
    try {
      if (!utils::lane_change::get_prepare_segment(
            common_data_ptr_, prev_module_output_.path, prep_metric.length, prepare_segment)) {
        debug_print("Reject: failed to get valid prepare segment!");
        continue;
      }
    } catch (const std::exception & e) {
      debug_print(e.what());
      break;
    }

    debug_print("Prepare path satisfy constraints");

    const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;

    const auto shift_length =
      autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
        target_lanes, lane_changing_start_pose);

    lane_change_debug_.lane_change_metrics.emplace_back();
    auto & debug_metrics = lane_change_debug_.lane_change_metrics.back();
    debug_metrics.prep_metric = prep_metric;
    debug_metrics.max_prepare_length = common_data_ptr_->transient_data.dist_to_terminal_start;
    const auto lane_changing_metrics = get_lane_changing_metrics(
      prepare_segment, prep_metric, shift_length, dist_to_next_regulatory_element, debug_metrics);

    // Keep preparation speeds independent from the execution speed samples.
    utils::lane_change::set_prepare_velocity(
      prepare_segment, common_data_ptr_->get_ego_speed(), prep_metric.velocity);

    std::vector<std::pair<double,double>> tested_dynamics;
    for (const auto & lc_metric : lane_changing_metrics) {
      // Larger lateral limits can produce the very same jerk-limited maneuver.
      // Do not spend the cycle regenerating identical rejected candidates.
      if (std::any_of(tested_dynamics.begin(),tested_dynamics.end(),[&](const auto & tested) {
            return std::abs(tested.first-lc_metric.duration)<1e-3 &&
              std::abs(tested.second-lc_metric.actual_lon_accel)<1e-6;
          })) continue;
      tested_dynamics.emplace_back(lc_metric.duration,lc_metric.actual_lon_accel);
      if (stop_watch_.toc(__func__) >= time_limit_ms) {
        RCLCPP_DEBUG(logger_, "Time limit reached and no safe path was found.");
        search_budget_exhausted_=true;
        if (best_safe) { candidate_paths.push_back(*best_safe); return true; }
        return false;
      }

      debug_metrics.lc_metrics.emplace_back(lc_metric, -1);

      const auto debug_print_lat = [&](const std::string & s) {
        RCLCPP_DEBUG(
          logger_, "%s | lc_time: %.5f | lon_acc: %.5f | lat_acc: %.5f | lc_len: %.5f", s.c_str(),
          lc_metric.duration, lc_metric.actual_lon_accel, lc_metric.lat_accel, lc_metric.length);
      };

      const bool different_dynamics = !candidate_paths.empty() &&
        lane_change_parameters_->trajectory.min_prepare_duration <= 1e-6 &&
        (std::abs(candidate_paths.back().info.longitudinal_acceleration.lane_changing -
                  lc_metric.actual_lon_accel) > 1e-6 ||
         std::abs(candidate_paths.back().info.duration.lane_changing - lc_metric.duration) > 1e-6);
      if (!different_dynamics && !check_length_diff(prep_metric.length, lc_metric.length, true)) {
        RCLCPP_DEBUG(logger_, "Skip: Change in lane changing length is less than threshold.");
        continue;
      }

      LaneChangePath candidate_path;
      try {
        candidate_path = utils::lane_change::get_candidate_path(
          common_data_ptr_, prep_metric, lc_metric, prepare_segment, sorted_lane_ids, shift_length);
      } catch (const std::exception & e) {
        debug_print_lat(std::string("Reject: ") + e.what());
        continue;
      }

      ++search_generated_;
      if (!prepareCandidateForSearch(candidate_path, mode)) continue;
      candidate_paths.push_back(candidate_path);
      debug_metrics.lc_metrics.back().second = static_cast<int>(candidate_paths.size()) - 1;

      try {
        ++search_checked_;
        if (check_candidate_path_safety(candidate_path, target_objects)) {
          ++search_safe_;
          debug_print_lat("ACCEPT!!!: it is valid and safe!");
          if (!best_safe || candidateRank(candidate_path)<candidateRank(*best_safe)) {
            best_safe = candidate_path;
          }
          continue;
        }
      } catch (const std::exception & e) {
        debug_print_lat(std::string("Reject: ") + e.what());
        // A footprint/boundary rejection belongs to this sample, not to the
        // entire search. Tighter lateral or different longitudinal samples can
        // still be feasible at the current speed (as in the Frenet branch).
        continue;
      }

      debug_print_lat("Reject: sampled path is not safe.");
    }
  }

  RCLCPP_DEBUG(logger_, "No safety path found.");
  if (best_safe) { candidate_paths.push_back(*best_safe); return true; }
  return false;
}

bool NormalLaneChange::check_candidate_path_safety(
  const LaneChangePath & candidate_path, const lane_change::TargetObjects & target_objects) const
{
  const auto is_stuck = common_data_ptr_->transient_data.is_ego_stuck;
  if (utils::lane_change::has_overtaking_turn_lane_object(
        common_data_ptr_, filtered_objects_.target_lane_trailing)) {
    throw std::logic_error("Ego is nearby intersection, and there might be overtaking vehicle.");
  }

  if (
    !is_stuck && utils::lane_change::is_delay_lane_change(
                   common_data_ptr_, candidate_path, filtered_objects_.target_lane_leading.stopped,
                   lane_change_debug_.collision_check_objects)) {
    throw std::logic_error(
      "Ego is not stuck and parked vehicle exists in the target lane. Skip lane change.");
  }

  const auto lc_start_velocity = candidate_path.info.velocity.prepare;
  const auto min_lc_velocity = lane_change_parameters_->trajectory.min_lane_changing_velocity;
  constexpr double margin = 0.1;
  // Check the lateral maneuver and one vehicle length of settling, not an
  // unchanged 400 m continuation. A distant reference-centerline corner is
  // handled by motion planning; it must not veto every local LC candidate.
  // The full output path and its downstream validation remain unchanged.
  if (
    lane_change_parameters_->safety.enable_target_lane_bound_check &&
    lc_start_velocity > min_lc_velocity + margin &&
    utils::lane_change::path_footprint_exceeds_target_lane_bound(
      common_data_ptr_,
      utils::lane_change::get_maneuver_check_path(
        candidate_path, planner_data_->parameters.vehicle_length),
      planner_data_->parameters.vehicle_info)) {
    throw std::logic_error("Path footprint exceeds target lane boundary. Skip lane change.");
  }

  candidate_path.prediction_coverage_deferred = false;
  if ((target_objects.empty()) || candidate_path.path.points.empty()) {
    RCLCPP_DEBUG(logger_, "There is nothing to check.");
    return true;
  }

  constexpr size_t decel_sampling_num = 1;
  auto ego_predicted_paths = utils::lane_change::convert_to_predicted_paths(
    common_data_ptr_, candidate_path, decel_sampling_num);
  // This candidate has not been published yet. The previous final trajectory
  // still follows the current lane, so requiring it to match the proposed LC
  // geometry makes every new cooperative candidate reject itself. Check the
  // candidate's executable speed profile and measured-motion hypothesis here;
  // isApprovedPathSafe separately requires the actual final output after approval.

  const auto safety_check_with_normal_rss = isLaneChangePathSafe(
    candidate_path, ego_predicted_paths, target_objects,
    common_data_ptr_->lc_param_ptr->safety.rss_params, lane_change_debug_.collision_check_objects);

  if (!safety_check_with_normal_rss.is_safe && is_stuck) {
    const auto safety_check_with_stuck_rss = isLaneChangePathSafe(
      candidate_path, ego_predicted_paths, target_objects,
      common_data_ptr_->lc_param_ptr->safety.rss_params_for_stuck,
      lane_change_debug_.collision_check_objects);
    return safety_check_with_stuck_rss.is_safe;
  }

  return safety_check_with_normal_rss.is_safe;
}

std::optional<PathWithLaneId> NormalLaneChange::compute_terminal_lane_change_path() const
{
  const auto & transient_data = common_data_ptr_->transient_data;
  const auto dist_to_terminal_start = transient_data.dist_to_terminal_start;
  const auto min_lc_velocity = lane_change_parameters_->trajectory.min_lane_changing_velocity;
  const auto current_velocity = getEgoVelocity();

  PathWithLaneId prepare_segment;
  try {
    if (!utils::lane_change::get_prepare_segment(
          common_data_ptr_, prev_module_output_.path, dist_to_terminal_start, prepare_segment)) {
      RCLCPP_DEBUG(logger_, "failed to get valid prepare segment for terminal LC path");
      return std::nullopt;
    }
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "failed to get terminal LC path: %s", e.what());
    return std::nullopt;
  }

  // t = 2 * d / (v1 + v2)
  const auto duration_to_lc_start =
    2.0 * dist_to_terminal_start / (current_velocity + min_lc_velocity);
  const auto lon_accel = std::invoke([&]() -> double {
    if (duration_to_lc_start < calculation::eps) {
      return 0.0;
    }
    return std::clamp(
      (min_lc_velocity - current_velocity) / duration_to_lc_start,
      lane_change_parameters_->trajectory.min_longitudinal_acc,
      common_data_ptr_->acceleration_limits.limit(std::max(current_velocity, min_lc_velocity),
        lane_change_parameters_->trajectory.max_longitudinal_acc));
  });
  const auto vel_on_prep = current_velocity + lon_accel * duration_to_lc_start;
  const LaneChangePhaseMetrics prep_metric(
    duration_to_lc_start, dist_to_terminal_start, vel_on_prep, lon_accel, lon_accel, 0.0);

  if (terminal_lane_change_path_) {
    terminal_lane_change_path_->info.set_prepare(prep_metric);
    if (prepare_segment.points.empty()) {
      terminal_lane_change_path_->path = terminal_lane_change_path_->shifted_path.path;
      return terminal_lane_change_path_->path;
    }
    prepare_segment.points.pop_back();
    terminal_lane_change_path_->path =
      utils::combinePath(prepare_segment, terminal_lane_change_path_->shifted_path.path);
    return terminal_lane_change_path_->path;
  }

  const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;
  const auto & target_lanes = common_data_ptr_->lanes_ptr->target;
  const auto shift_length =
    autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
      target_lanes, lane_changing_start_pose);

  const auto dist_lc_start_to_end_of_lanes = calculation::calc_dist_from_pose_to_terminal_end(
    common_data_ptr_, common_data_ptr_->lanes_ptr->target_neighbor,
    prepare_segment.points.back().point.pose);

  const auto max_lane_changing_length = std::invoke([&]() {
    double max_length = transient_data.dist_to_terminal_end - prep_metric.length;
    return std::min(
      max_length, dist_lc_start_to_end_of_lanes - transient_data.next_dist_buffer.min);
  });

  const auto max_path_velocity = prepare_segment.points.back().point.longitudinal_velocity_mps;
  constexpr double lane_changing_lon_accel{0.0};
  const auto lane_changing_metrics = calculation::calc_shift_phase_metrics(
    common_data_ptr_, shift_length, prep_metric.velocity, max_path_velocity,
    lane_changing_lon_accel, max_lane_changing_length);

  const auto sorted_lane_ids = utils::lane_change::get_sorted_lane_ids(common_data_ptr_);

  LaneChangePath candidate_path;
  for (const auto & lc_metric : lane_changing_metrics) {
    try {
      candidate_path = utils::lane_change::get_candidate_path(
        common_data_ptr_, prep_metric, lc_metric, prepare_segment, sorted_lane_ids, shift_length);
    } catch (const std::exception & e) {
      continue;
    }
    terminal_lane_change_path_ = candidate_path;
    return candidate_path.path;
  }

  return std::nullopt;
}

PathSafetyStatus NormalLaneChange::isApprovedPathSafe() const
{
  safety_diagnostic_samples_.clear();
  safety_rejected_pairs_ = 0;
  const auto & path = activeSafetyPath();
  hard_collision_detected_ = false;
  const auto & current_lanes = get_current_lanes();
  const auto & target_lanes = get_target_lanes();

  if (current_lanes.empty() || target_lanes.empty()) {
    return {true, true};
  }

  const auto target_objects = get_target_objects(filtered_objects_, current_lanes);

  CollisionCheckDebugMap debug_data;

  const auto has_overtaking_object = utils::lane_change::has_overtaking_turn_lane_object(
    common_data_ptr_, filtered_objects_.target_lane_trailing);

  if (has_overtaking_object) {
    return {false, true};
  }

  if (utils::lane_change::is_delay_lane_change(
        common_data_ptr_, path, filtered_objects_.target_lane_leading.stopped, debug_data)) {
    RCLCPP_DEBUG(logger_, "Lane change has been delayed.");
    return {false, false};
  }

  const auto decel_sampling_num =
    static_cast<size_t>(lane_change_parameters_->cancel.deceleration_sampling_num);
  auto ego_predicted_paths = utils::lane_change::convert_to_predicted_paths(
    common_data_ptr_, path, decel_sampling_num, true);
  {
    const auto feedback = finalVelocityPrediction(path);
    if (!feedback || (path.cooperative_overtake_target && !findOvertakeTarget())) {
      // Missing/unexecutable feedback is not evidence of an incoming rear object.
      publishApprovedSafetyDiagnostics(false, false);
      return {false, false};
    }
    ego_predicted_paths.push_back(*feedback);
  }
  const auto safety_status = isLaneChangePathSafe(
    path, ego_predicted_paths, target_objects, lane_change_parameters_->safety.rss_params_for_abort,
    debug_data, true);
  {
    // only for debug purpose
    lane_change_debug_.collision_check_objects.clear();
    lane_change_debug_.collision_check_object_debug_lifetime +=
      (stop_watch_.toc(getModuleTypeStr()) / 1000);
    if (lane_change_debug_.collision_check_object_debug_lifetime > 2.0) {
      stop_watch_.toc(getModuleTypeStr(), true);
      lane_change_debug_.collision_check_object_debug_lifetime = 0.0;
      lane_change_debug_.collision_check_objects_after_approval.clear();
    }

    if (!safety_status.is_safe) {
      lane_change_debug_.collision_check_objects_after_approval = debug_data;
    }
  }

  publishApprovedSafetyDiagnostics(safety_status.is_safe, safety_status.is_trailing_object);

  return safety_status;
}

PathSafetyStatus NormalLaneChange::evaluateApprovedPathWithUnsafeHysteresis(
  PathSafetyStatus approved_path_safety_status)
{
  if (!approved_path_safety_status.is_safe &&
      hard_collision_detected_) {
    return approved_path_safety_status;
  }
  if (!approved_path_safety_status.is_safe) {
    ++unsafe_hysteresis_count_;
    RCLCPP_DEBUG(
      logger_, "%s: Increasing hysteresis count to %d.", __func__, unsafe_hysteresis_count_);
  } else {
    if (unsafe_hysteresis_count_ > 0) {
      RCLCPP_DEBUG(logger_, "%s: Lane change is now SAFE. Resetting hysteresis count.", __func__);
    }
    unsafe_hysteresis_count_ = 0;
  }
  if (unsafe_hysteresis_count_ > lane_change_parameters_->cancel.th_unsafe_hysteresis) {
    RCLCPP_DEBUG(
      logger_, "%s: hysteresis count exceed threshold. lane change is now %s", __func__,
      (approved_path_safety_status.is_safe ? "safe" : "UNSAFE"));
    return approved_path_safety_status;
  }
  return {};
}

bool NormalLaneChange::isValidPath(const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & route_handler = planner_data_->route_handler;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, get_current_lanes()),
    utils::extendLanes(route_handler, get_target_lanes()));
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  const auto lanelets = utils::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_inside_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (autoware::experimental::lanelet2_utils::is_in_lanelet(point.point.pose, lanelet)) {
        is_inside_lanelet = true;
        break;
      }
    }
    if (!is_inside_lanelet) {
      return false;
    }
  }

  // check relative angle
  return utils::checkPathRelativeAngle(path, M_PI);
}

bool NormalLaneChange::isRequiredStop(const bool is_trailing_object)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  return common_data_ptr_->transient_data.is_ego_near_current_terminal_start &&
         isAbleToStopSafely() && is_trailing_object;
}

bool NormalLaneChange::calcAbortPath()
{
  // Prefer an immediate return; retain the configured envelope as the longest
  // candidate. Every timing uses the existing jerk and full object checks.
  const auto & c = lane_change_parameters_->cancel;
  for (const double delay : {0.0, c.delta_time * 0.5, c.delta_time}) {
    for (const double duration : {c.duration, c.duration * 0.75, c.duration * 0.5}) {
      if (duration > 0.0 && calcAbortPathForTiming(delay, duration)) return true;
    }
  }
  return false;
}

bool NormalLaneChange::calcAbortPathForTiming(const double delay, const double duration)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & route_handler = getRouteHandler();
  const auto & common_param = getCommonParam();
  const auto current_velocity =
    std::max(lane_change_parameters_->trajectory.min_lane_changing_velocity, getEgoVelocity());
  const auto current_pose = getEgoPose();
  const auto & selected_path = status_.lane_change_path;

  const auto ego_nearest_dist_threshold = common_param.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = common_param.ego_nearest_yaw_threshold;

  const auto current_min_dist_buffer = common_data_ptr_->transient_data.current_dist_buffer.min;

  const auto & lane_changing_path = selected_path.path;
  const auto & reference_lanelets = get_current_lanes();
  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end = std::max(
      lanelet::geometry::length3d(lanelet::LaneletSequence(reference_lanelets)) -
        current_min_dist_buffer,
      0.0);

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      lane_changing_path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto ego_pose_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    lane_changing_path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto get_abort_idx_and_distance = [&](const double param_time) {
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, 0.0);
    }

    const auto desired_distance = current_velocity * param_time;
    const auto & points = lane_changing_path.points;

    for (size_t idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const double distance =
        utils::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      if (distance > desired_distance) {
        return std::make_pair(idx, distance);
      }
    }

    return std::make_pair(ego_pose_idx, 0.0);
  };

  const auto [abort_start_idx, abort_start_dist] =
    get_abort_idx_and_distance(delay);
  const auto [abort_return_idx, abort_return_dist] = get_abort_idx_and_distance(delay + duration);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR(logger_, "abort start idx and return idx is equal. can't compute abort path.");
    return false;
  }

  const auto enough_abort_dist =
    abort_start_dist + abort_return_dist +
      calculation::calc_stopping_distance(common_data_ptr_->lc_param_ptr) <=
    common_data_ptr_->transient_data.dist_to_terminal_start;

  if (!enough_abort_dist) {
    RCLCPP_ERROR(logger_, "insufficient distance to abort.");
    return false;
  }

  const auto abort_start_pose = lane_changing_path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = lane_changing_path.points.at(abort_return_idx).point.pose;
  const auto shift_length = autoware::experimental::lanelet2_utils::get_arc_coordinates(
                              reference_lanelets, abort_return_pose)
                              .distance;

  ShiftLine shift_line;
  shift_line.start = abort_start_pose;
  shift_line.end = abort_return_pose;
  shift_line.end_shift_length = -shift_length;
  shift_line.start_idx = abort_start_idx;
  shift_line.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(lane_changing_path);
  path_shifter.addShiftLine(shift_line);
  const auto lateral_jerk = autoware::motion_utils::calc_jerk_from_lat_lon_distance(
    shift_line.end_shift_length, abort_return_dist - abort_start_dist, current_velocity);
  path_shifter.setVelocity(current_velocity);
  const auto lateral_acc_range =
    lane_change_parameters_->trajectory.lat_acc_map.find(current_velocity);
  const double & max_lateral_acc = lateral_acc_range.second;
  path_shifter.setLateralAccelerationLimit(max_lateral_acc);

  if (std::abs(lateral_jerk) > lane_change_parameters_->cancel.max_lateral_jerk) {
    RCLCPP_ERROR(logger_, "Aborting jerk is too strong. lateral_jerk = %f", lateral_jerk);
    return false;
  }

  ShiftedPath shifted_path;
  // offset front side
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(logger_, "failed to generate abort shifted path.");
    return false;
  }
  if (abort_return_idx >= shifted_path.path.points.size()) return false;

  auto reference_lane_segment = prev_module_output_.path;
  {
    // const auto terminal_path =
    //   calcTerminalLaneChangePath(reference_lanelets, get_target_lanes());
    // if (terminal_path) {
    //   reference_lane_segment = terminal_path->path;
    // }
    const auto return_pose = shifted_path.path.points.at(abort_return_idx).point.pose;
    const auto seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_lane_segment.points, return_pose, common_param.ego_nearest_dist_threshold,
      common_param.ego_nearest_yaw_threshold);
    reference_lane_segment.points = autoware::motion_utils::cropPoints(
      reference_lane_segment.points, return_pose.position, seg_idx,
      common_param.forward_path_length, 0.0);
  }

  auto abort_path = selected_path;
  abort_path.cooperative_overtake_target.reset();
  abort_path.shifted_path = shifted_path;
  abort_path.info.shift_line = shift_line;

  {
    PathWithLaneId aborting_path;
    // Rebuilding only the points must not drop the approved path's frame/stamp.
    aborting_path.header = lane_changing_path.header;
    aborting_path.points.insert(
      aborting_path.points.begin(), shifted_path.path.points.begin(),
      shifted_path.path.points.begin() + static_cast<int>(abort_return_idx));

    if (!reference_lane_segment.points.empty()) {
      abort_path.path = utils::combinePath(aborting_path, reference_lane_segment);
    } else {
      abort_path.path = aborting_path;
    }
  }

  // Return geometry and metadata must agree. In particular, safety prediction
  // must follow the return segment, not the original LC endpoint and duration.
  abort_path.type = lane_change::PathType::SpatialSpeedProfile;
  abort_path.info.lane_changing_start = abort_start_pose;
  abort_path.info.lane_changing_end = shifted_path.path.points.at(abort_return_idx).point.pose;
  abort_path.info.length = {abort_start_dist, abort_return_dist-abort_start_dist};
  abort_path.info.duration = {abort_start_dist/current_velocity,
    (abort_return_dist-abort_start_dist)/current_velocity};
  const auto previous_stop_pose = lane_change_stop_pose_;
  insert_stop_point(get_current_lanes(), abort_path.path);
  try {
    const auto predictions = utils::lane_change::convert_to_predicted_paths(
      common_data_ptr_, abort_path, 1, true);
    CollisionCheckDebugMap debug;
    // Include leaders and rear vehicles in both lanes before calling this
    // feasible return path safe. No cooperative relaxation on a return path.
    if (!isLaneChangePathSafe(abort_path, predictions, allOvertakeObjects(),
        lane_change_parameters_->safety.rss_params_for_abort, debug, true).is_safe) {
      RCLCPP_WARN(logger_, "Reject abort path: collision or RSS conflict on actual return path.");
      lane_change_stop_pose_ = previous_stop_pose;
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Reject abort prediction: %s", e.what());
    lane_change_stop_pose_ = previous_stop_pose;
    return false;
  }
  abort_path_ = std::make_shared<LaneChangePath>(abort_path);
  return true;
}

std::optional<std::vector<ExtendedPredictedObject>>
NormalLaneChange::find_colliding_object_if_all_paths_collide(
  const LaneChangePath & lane_change_path,
  const std::vector<std::vector<PoseWithVelocityStamped>> & ego_predicted_paths,
  const ExtendedPredictedObjects & objects,
  const utils::path_safety_checker::RSSparams & rss_params, CollisionCheckDebugMap & debug_data,
  const bool is_approved) const
{
  std::vector<ExtendedPredictedObject> colliding_objects;
  std::unordered_set<size_t> objects_idx;

  const auto check_for_collision = [&](const auto & ego_predicted_path, const auto & object) {
    return is_colliding(
      lane_change_path, object, ego_predicted_path, rss_params, debug_data, is_approved);
  };

  const auto check_for_collisions = [&](const auto & ego_predicted_path) {
    std::vector<ExtendedPredictedObject> current_colliding_objects;
    current_colliding_objects.reserve(objects.size());

    for (const auto & [idx, object] : objects | ranges::views::enumerate) {
      const auto is_colliding_object = check_for_collision(ego_predicted_path, object);
      if (!is_colliding_object) {
        continue;
      }
      // Every hypothesis needs its own nonempty collision set. Deduplication
      // across hypotheses must not turn a repeated collision into a safe path.
      current_colliding_objects.push_back(object);
      if (objects_idx.find(idx) == objects_idx.end()) {
        colliding_objects.push_back(object);
        objects_idx.insert(idx);
      }
    }

    if (current_colliding_objects.empty()) {
      return false;
    }

    return true;
  };

  if (ranges::all_of(ego_predicted_paths, check_for_collisions)) {
    return colliding_objects;
  }

  return std::nullopt;
}

PathSafetyStatus NormalLaneChange::isLaneChangePathSafe(
  const LaneChangePath & lane_change_path,
  const std::vector<std::vector<PoseWithVelocityStamped>> & ego_predicted_paths,
  const lane_change::TargetObjects & collision_check_objects,
  const utils::path_safety_checker::RSSparams & rss_params, CollisionCheckDebugMap & debug_data,
  const bool is_approved) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  lane_change_path.prediction_coverage_deferred = false;
  constexpr auto is_safe = true;
  constexpr auto is_moving_object_behind_ego = true;
  if (ego_predicted_paths.empty()) {
    if (lane_change_path.type == lane_change::PathType::SpatialSpeedProfile) {
      return {!is_safe, !is_moving_object_behind_ego};
    }
    return {is_safe, !is_moving_object_behind_ego};
  }

  const auto check_for_colliding_objects =
    [&](decltype(ego_predicted_paths) & ego_pred_paths, const auto & objects) {
      return find_colliding_object_if_all_paths_collide(
        lane_change_path, ego_pred_paths, objects, rss_params, debug_data, is_approved);
    };

  const auto check_for_moving_objects = [this](const auto & object) {
    return utils::lane_change::is_moving_object(common_data_ptr_, object);
  };

  // Published motion and actual-motion hypotheses are requirements, not
  // interchangeable alternatives. An unpublished deceleration must not rescue
  // a different, unsafe output profile.
  for (const auto & prediction : ego_predicted_paths) {
    if (prediction.empty()) return {!is_safe, !is_moving_object_behind_ego};
    if (const auto collision = check_for_colliding_objects(
          {prediction}, collision_check_objects.trailing)) {
      return {!is_safe, ranges::any_of(*collision, check_for_moving_objects)};
    }
    if (check_for_colliding_objects({prediction}, collision_check_objects.leading)) {
      return {!is_safe, !is_moving_object_behind_ego};
    }
  }
  return {is_safe, !is_moving_object_behind_ego};
}

bool NormalLaneChange::is_colliding(
  const LaneChangePath & lane_change_path, const ExtendedPredictedObject & obj,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const RSSparams & rss_param,
  CollisionCheckDebugMap & debug_data, const bool is_approved) const
{
  if (lane_change_path.path.points.empty() || ego_predicted_path.empty()) return false;
  if (is_approved) {
    for (const auto & object_path : obj.predicted_paths) {
      for (const auto & object_pose : object_path.path) {
        const auto ego = utils::path_safety_checker::calc_interpolated_pose_with_velocity(
          ego_predicted_path, object_pose.time);
        if (!ego) continue;
        const auto polygon = utils::lane_change::get_ego_footprint(
          ego->pose, common_data_ptr_->bpp_param_ptr->vehicle_info);
        if (boost::geometry::intersects(polygon, object_pose.polygon)) {
          hard_collision_detected_ = true;
          break;
        }
      }
    }
  }

  // Never interpret the unpredicted tail of a maneuver as a checked gap.
  // Every object path used by this checker must cover this ego hypothesis.
  // Shorter lateral candidates remain searchable; do not manufacture an NPC
  // forecast by copying its last pose after the predictor's horizon.
  bool covered = !obj.predicted_paths.empty();
  double available = std::numeric_limits<double>::infinity();
  for (const auto & object_path : obj.predicted_paths) {
    covered = covered && !object_path.path.empty();
    if (object_path.path.empty()) { available = 0.0; continue; }
    available = std::min(available, object_path.path.back().time);
    covered = covered && object_path.path.front().time <= ego_predicted_path.front().time + 1e-3 &&
      object_path.path.back().time + 1e-3 >= ego_predicted_path.back().time;
  }
  if (!covered) {
    // A long preparation may retain a conditional plan. Only the observed
    // prefix is checked; publication holds before lateral entry until every
    // remaining hypothesis has complete coverage on fresh predictions.
    const auto split = is_approved
      ? utils::lane_change::remaining_prepare_time(
          lane_change_path.path, ego_predicted_path, lane_change_path.info.lane_changing_start)
      : std::optional<double>{lane_change_path.info.duration.prepare};
    const double to_start = calcSignedArcLength(lane_change_path.path.points,
      getEgoPosition(), lane_change_path.info.lane_changing_start.position);
    const double braking = getEgoVelocity() * getEgoVelocity() /
      (2.0 * std::max(0.1, std::abs(planner_data_->parameters.min_acc)));
    const bool has_prefix = std::all_of(obj.predicted_paths.begin(), obj.predicted_paths.end(),
      [&](const auto & path) { return !path.path.empty() &&
        path.path.front().time <= ego_predicted_path.front().time + 1e-3; });
    if (has_prefix && std::isfinite(available) && split && *split > 1e-3 &&
        available + 1e-3 >= lane_change_path.info.duration.lane_changing &&
        to_start > braking + planner_data_->parameters.base_link2front &&
        utils::isEgoWithinOriginalLane(common_data_ptr_->lanes_polygon_ptr->current,
          getEgoPose(), planner_data_->parameters, 0.0) &&
        !lane_change_path.cooperative_overtake_target && !isAbortState()) {
      std::vector<PoseWithVelocityStamped> prefix;
      for (const auto & point : ego_predicted_path) {
        if (point.time <= available + 1e-6) prefix.push_back(point);
      }
      if (const auto end = utils::path_safety_checker::calc_interpolated_pose_with_velocity(
            ego_predicted_path, available)) {
        if (prefix.empty() || prefix.back().time < available - 1e-6) prefix.push_back(*end);
      }
      if (prefix.size() >= 2) {
        const bool collision = is_colliding(
          lane_change_path, obj, prefix, rss_param, debug_data, is_approved);
        if (!collision) lane_change_path.prediction_coverage_deferred = true;
        return collision;
      }
    }
    auto & debug = debug_data[obj.uuid.uuid];
    debug.is_safe = false;
    debug.unsafe_reason = "prediction_coverage_short";
    debug.expected_object_pose = obj.initial_pose;
    utils::path_safety_checker::RSSCollisionTrace trace;
    trace.time = available;
    captureSafetyDiagnostic("coverage", obj, lane_change_path, ego_predicted_path,
      rss_param, trace, debug);
    return true;
  }

  if (lane_change_path.cooperative_overtake_target &&
      obj.uuid == *lane_change_path.cooperative_overtake_target && usesCooperativePolicy(obj)) {
    utils::path_safety_checker::RSSCollisionTrace trace;
    const bool safe = utils::lane_change::check_cooperative_rear(
      common_data_ptr_->current_lanes_path, ego_predicted_path, obj,
      common_data_ptr_->bpp_param_ptr->vehicle_info, lane_change_parameters_->overtake,
      rss_param.lateral_distance_max_threshold, debug_data, &trace);
    const auto debug = debug_data.find(obj.uuid.uuid);
    if (!safe && debug != debug_data.end()) {
      if (is_approved && debug->second.unsafe_reason == "overlap_polygon") {
        hard_collision_detected_ = true;
      }
      captureSafetyDiagnostic("cooperative_rear", obj, lane_change_path,
        ego_predicted_path, rss_param, trace, debug->second);
    }
    return !safe;
  }

  // Before approval the prediction begins at the candidate's preparation start.
  // After approval, derive the remaining time separately for each current
  // motion hypothesis, including a hypothesis that stops before the LC start.
  const auto split = is_approved
    ? utils::lane_change::remaining_prepare_time(
        lane_change_path.path, ego_predicted_path, lane_change_path.info.lane_changing_start)
    : std::optional<double>{lane_change_path.info.duration.prepare};
  if (!split) return true;
  const auto yaw_threshold = common_data_ptr_->lc_param_ptr->safety.collision_check.th_yaw_diff;
  const auto & vehicle_info = common_data_ptr_->bpp_param_ptr->vehicle_info;
  const ExtendedPredictedObjects objects{obj};
  const auto & parked_threshold = lane_change_parameters_->safety.th_stopped_object_velocity;
  const auto & lane_changing_params =
    (!is_approved && obj.initial_twist.linear.x <= parked_threshold)
      ? lane_change_parameters_->safety.rss_params_for_parked
      : rss_param;
  const bool observe = safety_diagnostics_pub_ &&
    safety_diagnostics_pub_.getNumSubscribers();
  const auto result = utils::lane_change::check_safety_by_phase(
    lane_change_path.path, ego_predicted_path, vehicle_info, objects, debug_data,
    common_data_ptr_->lc_param_ptr->safety.rss_params_for_prepare, lane_changing_params,
    *split, yaw_threshold, observe);
  const auto debug = debug_data.find(obj.uuid.uuid);
  if (!result.safe && is_approved && debug != debug_data.end() &&
      debug->second.unsafe_reason == "overlap_polygon") hard_collision_detected_ = true;
  if (!result.safe && observe && debug != debug_data.end()) {
    captureSafetyDiagnostic(result.stage, obj, lane_change_path, ego_predicted_path,
      lane_changing_params, result.trace, debug->second, *split);
  }
  return !result.safe;
}

double NormalLaneChange::get_max_velocity_for_safety_check() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto external_velocity_limit_ptr = planner_data_->external_limit_max_velocity;
  if (external_velocity_limit_ptr) {
    return std::min(
      static_cast<double>(external_velocity_limit_ptr->max_velocity), getCommonParam().max_vel);
  }

  return getCommonParam().max_vel;
}

bool NormalLaneChange::is_ego_stuck() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & lc_param_ptr = common_data_ptr_->lc_param_ptr;

  if (std::abs(common_data_ptr_->get_ego_speed()) > lc_param_ptr->th_stop_velocity) {
    RCLCPP_DEBUG(logger_, "Ego is still moving, not in stuck");
    return false;
  }

  // Ego is just stopped, not sure it is in stuck yet.
  if (getStopTime() < lc_param_ptr->th_stop_time) {
    RCLCPP_DEBUG(logger_, "Ego is just stopped, counting for stuck judge... (%f)", getStopTime());
    return false;
  }

  // Check if any stationary object exist in obstacle_check_distance
  const auto & current_lanes_path = common_data_ptr_->current_lanes_path;
  const auto & ego_pose = common_data_ptr_->get_ego_pose();
  const auto rss_dist = calcRssDistance(
    0.0, lc_param_ptr->trajectory.min_lane_changing_velocity, lc_param_ptr->safety.rss_params);

  // It is difficult to define the detection range. If it is too short, the stuck will not be
  // determined, even though you are stuck by an obstacle. If it is too long,
  // the ego will be judged to be stuck by a distant vehicle, even though the ego is only
  // stopped at a traffic light. Essentially, the calculation should be based on the information of
  // the stop reason, but this is outside the scope of one module. I keep it as a TODO.
  constexpr auto detection_distance_margin = 10.0;
  const auto obstacle_check_distance = common_data_ptr_->transient_data.lane_changing_length.max +
                                       rss_dist + common_data_ptr_->bpp_param_ptr->base_link2front +
                                       detection_distance_margin;
  const auto has_object_blocking = std::any_of(
    filtered_objects_.current_lane.begin(), filtered_objects_.current_lane.end(),
    [&](const auto & object) {
      // Note: it needs chattering prevention.
      if (utils::lane_change::is_moving_object(common_data_ptr_, object)) {  // check if stationary
        return false;
      }

      const auto ego_to_obj_dist =
        calcSignedArcLength(
          current_lanes_path.points, ego_pose.position, object.initial_pose.position) -
        obstacle_check_distance;
      return ego_to_obj_dist < 0.0;
    });

  lane_change_debug_.is_stuck = has_object_blocking;
  return has_object_blocking;
}

void NormalLaneChange::set_stop_pose(
  const double arc_length_to_stop_pose, PathWithLaneId & path, const std::string & reason)
{
  const auto stop_idx = autoware::motion_utils::insertStopPoint(
    arc_length_to_stop_pose, path.points);
  if (!stop_idx) {
    lane_change_stop_pose_.reset();
    return;
  }
  lane_change_stop_pose_ = PoseWithDetailOpt(PoseWithDetail(path.points.at(*stop_idx).point.pose, reason));
}

void NormalLaneChange::updateStopTime()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto current_vel = getEgoVelocity();

  if (std::abs(current_vel) > lane_change_parameters_->th_stop_velocity) {
    stop_time_ = 0.0;
  } else {
    const double duration = stop_watch_.toc("stop_time");
    // clip stop time
    if (stop_time_ + duration * 0.001 > lane_change_parameters_->th_stop_time) {
      constexpr double eps = 0.1;
      stop_time_ = lane_change_parameters_->th_stop_time + eps;
    } else {
      stop_time_ += duration * 0.001;
    }
  }

  stop_watch_.tic("stop_time");
}

void NormalLaneChange::update_dist_from_intersection()
{
  auto & transient_data = common_data_ptr_->transient_data;
  const auto & route_handler_ptr = common_data_ptr_->route_handler_ptr;

  if (
    transient_data.in_intersection && transient_data.in_turn_direction_lane &&
    path_after_intersection_.empty()) {
    auto path_after_intersection = route_handler_ptr->getCenterLinePath(
      common_data_ptr_->lanes_ptr->target_neighbor, 0.0, std::numeric_limits<double>::max());
    path_after_intersection_ = std::move(path_after_intersection.points);
    transient_data.dist_from_prev_intersection = 0.0;
    return;
  }

  if (
    transient_data.in_intersection || transient_data.in_turn_direction_lane ||
    path_after_intersection_.empty()) {
    return;
  }

  const auto & path_points = path_after_intersection_;
  const auto & front_point = path_points.front().point.pose.position;
  const auto & ego_position = common_data_ptr_->get_ego_pose().position;
  transient_data.dist_from_prev_intersection =
    calcSignedArcLength(path_points, front_point, ego_position);

  if (
    transient_data.dist_from_prev_intersection >= 0.0 &&
    transient_data.dist_from_prev_intersection <=
      common_data_ptr_->lc_param_ptr->backward_length_from_intersection) {
    return;
  }

  path_after_intersection_.clear();
  transient_data.dist_from_prev_intersection = std::numeric_limits<double>::max();
}

bool NormalLaneChange::is_ego_in_current_or_target_lanes() const
{
  lanelet::ConstLanelet current_lane;
  if (!common_data_ptr_->route_handler_ptr->getClosestLaneletWithinRoute(
        common_data_ptr_->get_ego_pose(), &current_lane)) {
    return false;
  }

  const auto & current_lanes = get_current_lanes();
  const auto in_current =
    utils::lane_change::is_lanelet_in_lanelet_collections(current_lanes, current_lane);

  if (in_current) {
    return true;
  }

  const auto & target_lanes = get_target_lanes();
  const auto in_target =
    utils::lane_change::is_lanelet_in_lanelet_collections(target_lanes, current_lane);

  return in_target;
}

bool NormalLaneChange::hasMissedLaneChangePath() const
{
  if (!lane_change_parameters_->enable_path_miss_detection) {
    return false;
  }

  if (status_.lane_change_path.path.points.empty()) {
    return false;
  }

  const auto & current_pose = getEgoPose();
  const auto & path_points = status_.lane_change_path.path.points;

  updateExecutionManeuver(status_.lane_change_path);
  const double required = executionRecoveryTime(
    current_pose, getEgoVelocity(), executionProfile(status_.lane_change_path));
  const double allowance = lane_change_parameters_->execution_recovery_time;
  if (ego_recovery_.expired(ros::Time::now().toSec(), required <= allowance, allowance)) {
    RCLCPP_WARN(logger_, "LC progress requests replanning: allowance %.2f s (estimated %.2f s)",
                allowance, required);
    execution_replan_requested_ = true;
  }

  // Check longitudinal deviation - if ego has passed the end but is not in target lane
  const auto dist_to_end = utils::getSignedDistance(
    current_pose, status_.lane_change_path.info.shift_line.end, get_target_lanes());

  if (dist_to_end < -lane_change_parameters_->path_miss_threshold_longitudinal) {
    // Passed end pose significantly
    const auto & target_lanes_poly = common_data_ptr_->lanes_polygon_ptr->target;

    const auto ego_footprint = utils::lane_change::get_ego_footprint(
      current_pose, common_data_ptr_->bpp_param_ptr->vehicle_info);
    const auto ego_in_target_lane = !boost::geometry::disjoint(target_lanes_poly, ego_footprint);

    if (!ego_in_target_lane) {
      RCLCPP_WARN(
        logger_, "Path miss detected: passed end pose by %.2f but ego footprint not in target lane",
        std::abs(dist_to_end));
      return true;
    }
  }

  return false;
}
}  // namespace autoware::behavior_path_planner
