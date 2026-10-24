// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/overtake.hpp"
#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace autoware::behavior_path_planner
{
namespace lc = utils::lane_change;
namespace safety = utils::path_safety_checker;

void NormalLaneChange::receiveFinalTrajectory(
  const autoware_planning_msgs::TrajectoryConstPtr & message)
{
  std::lock_guard<std::mutex> lock(final_trajectory_mutex_);
  final_trajectory_ = message;
  final_trajectory_received_ = ros::Time::now();
}

const char * NormalLaneChange::overtakePhaseName() const
{
  switch (overtake_.phase) {
    case OvertakePhase::Idle: return "idle";
    case OvertakePhase::Accelerating: return "accelerating_in_current_lane";
    case OvertakePhase::Ready: return "gap_ready_waiting_for_safe_candidate";
    case OvertakePhase::Changing: return "changing_lane";
    case OvertakePhase::Unavailable: return "front_preparation_unavailable";
  }
  return "invalid";
}

lane_change::TargetObjects NormalLaneChange::allOvertakeObjects() const
{
  lane_change::TargetObjects result{{}, {}};
  std::set<decltype(ExtendedPredictedObject{}.uuid.uuid)> seen;
  for (const auto * group : {&filtered_objects_.target_lane_leading.moving,
      &filtered_objects_.target_lane_leading.stopped,
      &filtered_objects_.target_lane_leading.stopped_at_bound,
      &filtered_objects_.target_lane_trailing, &filtered_objects_.current_lane,
      &filtered_objects_.others}) {
    for (const auto & object : *group) {
      if (seen.insert(object.uuid.uuid).second) {
        (object.dist_from_ego < 0.0 ? result.trailing : result.leading).push_back(object);
      }
    }
  }
  return result;
}

const ExtendedPredictedObject * NormalLaneChange::findOvertakeTarget() const
{
  if (!overtake_.target) return nullptr;
  for (const auto * group : {&filtered_objects_.target_lane_leading.moving,
      &filtered_objects_.target_lane_trailing, &filtered_objects_.current_lane,
      &filtered_objects_.others}) {
    for (const auto & object : *group) {
      if (object.uuid == *overtake_.target) return &object;
    }
  }
  return nullptr;
}

bool NormalLaneChange::usesCooperativePolicy(const ExtendedPredictedObject & object) const
{
  return lane_change_parameters_->overtake.enable &&
         lane_change_parameters_->overtake.assume_cooperative_npc &&
         overtake_.target && object.uuid == *overtake_.target && !isAbortState() &&
         (overtake_.phase == OvertakePhase::Ready ||
          overtake_.phase == OvertakePhase::Changing);
}

const LaneChangePath & NormalLaneChange::activeSafetyPath() const
{
  return isAbortState() && abort_path_ ? *abort_path_ : status_.lane_change_path;
}

void NormalLaneChange::updateOvertake(const bool is_approved)
{
  const auto & p = lane_change_parameters_->overtake;
  overtake_prepare_path_.reset();
  if (!p.enable || !p.assume_cooperative_npc || type_ != LaneChangeModuleType::NORMAL) return;
  if (is_approved) {
    if (overtake_.target) overtake_.phase = OvertakePhase::Changing;
    return;
  }
  const auto unavailable = [&](const char * reason) {
    overtake_.phase = OvertakePhase::Unavailable;
    overtake_.reason = reason;
  };
  // Reevaluate current geometry and predictions every cycle. An old target or
  // failed preparation never latches a timer that suppresses normal LC search.
  if (overtake_.target && !findOvertakeTarget()) {
    overtake_ = OvertakeState{};
  }
  if (!overtake_.target) {
    if (getEgoVelocity() < p.minimum_ego_speed ||
        common_data_ptr_->transient_data.in_intersection ||
        common_data_ptr_->transient_data.in_turn_direction_lane ||
        common_data_ptr_->transient_data.dist_to_terminal_start <
          getEgoVelocity() * 3.0 + p.minimum_gap) return;
    const ExtendedPredictedObject * target = nullptr;
    for (const auto * group : {&filtered_objects_.target_lane_leading.moving,
        &filtered_objects_.target_lane_trailing}) {
      for (const auto & object : *group) {
        const auto label = object.classification.label;
        if (label != autoware_perception_msgs::ObjectClassification::CAR &&
            label != autoware_perception_msgs::ObjectClassification::TRUCK &&
            label != autoware_perception_msgs::ObjectClassification::BUS &&
            label != autoware_perception_msgs::ObjectClassification::TRAILER) continue;
        const double yaw = tf2::getYaw(object.initial_pose.orientation) -
                           tf2::getYaw(getEgoPose().orientation);
        if (std::cos(yaw) < std::cos(M_PI / 6.0) ||
            object.initial_twist.linear.x < 1.0 ||
            std::abs(object.dist_from_ego) > p.activation_distance ||
            object.predicted_paths.empty()) continue;
        if (!target || std::abs(object.dist_from_ego) < std::abs(target->dist_from_ego)) {
          target = &object;
        }
      }
    }
    if (!target) return;
    overtake_.target = target->uuid;
    overtake_.phase = OvertakePhase::Accelerating;
  }
  const auto * target = findOvertakeTarget();
  if (!target) { unavailable("tracked_target_missing"); return; }
  overtake_.gap = lc::rear_body_gap(common_data_ptr_->current_lanes_path,
    getEgoPose(), target->initial_polygon, getCommonParam().vehicle_info);
  const double required = lc::cooperative_rear_distance(
    getEgoVelocity(), std::hypot(target->initial_twist.linear.x,
      target->initial_twist.linear.y), p);
  if (!buildOvertakePreparation(*target)) {
    unavailable("no_safe_acceleration_or_merge_space"); return;
  }
  overtake_.phase = overtake_.gap >= required ? OvertakePhase::Ready : OvertakePhase::Accelerating;
  overtake_.reason = overtake_.phase == OvertakePhase::Ready ?
    "measured_body_gap_ready" : "waiting_for_measured_body_gap";
}

bool NormalLaneChange::buildOvertakePreparation(const ExtendedPredictedObject & target)
{
  const auto & p = lane_change_parameters_->overtake;
  const auto & bpp = getCommonParam();
  if (prev_module_output_.path.points.size() < 3 || target.predicted_paths.empty()) return false;
  const auto objects = allOvertakeObjects();
  auto all_objects = objects.leading;
  all_objects.insert(all_objects.end(), objects.trailing.begin(), objects.trailing.end());
  const double max_acc = std::min(bpp.max_acc, lane_change_parameters_->trajectory.max_longitudinal_acc);
  const auto & reference = common_data_ptr_->current_lanes_path;
  const auto & target_reference = common_data_ptr_->target_lanes_path;
  const auto & current_polygon = common_data_ptr_->lanes_polygon_ptr->current;
  if (target_reference.points.size() < 2 || current_polygon.size() < 3 ||
      !utils::isEgoWithinOriginalLane(current_polygon, getEgoPose(), bpp)) return false;
  for (double acceleration : {max_acc, std::min(max_acc, 2.0), std::min(max_acc, 1.0)}) {
    if (acceleration <= 0.0) continue;
    auto path = prev_module_output_.path;
    const auto first = planner_data_->findEgoSegmentIndex(path.points);
    const double start = autoware::motion_utils::calcSignedArcLength(path.points,
      path.points.front().point.pose.position, getEgoPosition());
    std::vector<double> s, limits;
    double arc = 0.0;
    for (size_t i = first; i < path.points.size(); ++i) {
      if (i > first) arc += autoware_utils::calc_distance2d(path.points[i-1], path.points[i]);
      s.push_back(arc);
      limits.push_back(std::min<double>(path.points[i].point.longitudinal_velocity_mps,
        get_max_velocity_for_safety_check()));
    }
    std::vector<double> velocities;
    try {
      velocities = lc::make_speed_profile(s, limits, getEgoVelocity(), acceleration,
        std::abs(bpp.min_acc), common_data_ptr_->acceleration_limits);
    } catch (const std::exception &) { continue; }
    for (size_t i = 0; i < velocities.size(); ++i) {
      path.points[first+i].point.longitudinal_velocity_mps = velocities[i];
    }
    // Prediction coverage and remaining physical space bound this preparation,
    // not elapsed time since a target was first selected.
    double horizon = lc::speed_profile_duration(s, velocities);
    for (const auto & object_path : target.predicted_paths) {
      if (object_path.path.empty()) { horizon = 0.0; break; }
      horizon = std::min(horizon, object_path.path.back().time);
    }
    if (!std::isfinite(horizon) || horizon <= 0.0) continue;
    std::vector<PoseWithVelocityStamped> prediction;
    std::optional<double> gap_time;
    bool within_current_lane = true;
    for (double t = 0; t <= horizon + 1e-6; t += 0.1) {
      const auto [distance, speed] = lc::sample_speed_profile(s, velocities, t);
      const auto ego = autoware::motion_utils::calcInterpolatedPose(path.points, start+distance);
      if (!utils::isEgoWithinOriginalLane(current_polygon, ego, bpp)) {
        within_current_lane = false;
        break;
      }
      prediction.emplace_back(t, ego, speed);
      bool all_behind = true;
      for (const auto & object_path : target.predicted_paths) {
        const auto object = safety::calc_interpolated_object_pose(object_path.path, t, target.shape);
        if (!object || lc::rear_body_gap(reference, ego, object->polygon, bpp.vehicle_info) <
            lc::cooperative_rear_distance(speed, object->velocity, p)) {
          all_behind = false;
          break;
        }
      }
      if (all_behind) { gap_time = t; break; }
    }
    if (!within_current_lane || !gap_time || prediction.size() < 1) continue;
    if (prediction.size() == 1) {
      const auto [distance, speed] = lc::sample_speed_profile(s, velocities, 0.1);
      const auto next = autoware::motion_utils::calcInterpolatedPose(path.points, start+distance);
      if (!utils::isEgoWithinOriginalLane(current_polygon, next, bpp)) continue;
      prediction.emplace_back(0.1, next, speed);
    }
    const double prepare_distance = autoware::motion_utils::calcSignedArcLength(
      reference.points, getEgoPosition(), prediction.back().pose.position);
    // terminal_start already reserves the necessary lateral-maneuver buffer.
    if (prepare_distance >= common_data_ptr_->transient_data.dist_to_terminal_start) continue;
    safety::CollisionCheckDebugMap debug;
    // Include current-lane leaders regardless of the normal LC filtering switch.
    if (!safety::check_safety_with_rss(path, prediction, bpp.vehicle_info, all_objects,
        debug, lane_change_parameters_->safety.rss_params_for_prepare, true, 1.0, M_PI)) continue;

    // Preflight the target-lane slot at the predicted passing time. This coarse
    // gate never approves a maneuver; the complete LC is checked after the
    // actual measured gap has been achieved.
    const auto last = safety::calc_interpolated_pose_with_velocity(prediction, *gap_time);
    if (!last) continue;
    const double target_s = autoware::motion_utils::calcSignedArcLength(target_reference.points,
      target_reference.points.front().point.pose.position, last->pose.position);
    const auto merge_pose = autoware::motion_utils::calcInterpolatedPose(target_reference.points, target_s);
    safety::ExtendedPredictedObjects slot_objects;
    bool slot_covered = true;
    for (const auto & object : all_objects) {
      if (object.uuid == target.uuid) continue;
      auto copy = object;
      copy.predicted_paths.clear();
      for (const auto & object_path : object.predicted_paths) {
        const auto at = safety::calc_interpolated_object_pose(object_path.path, *gap_time, object.shape);
        if (!at) { slot_covered = false; break; }
        safety::PredictedPathWithPolygon single;
        single.confidence = object_path.confidence;
        auto snapshot = *at;
        snapshot.time = 0.0;
        single.path.push_back(snapshot);
        copy.predicted_paths.push_back(single);
      }
      if (copy.predicted_paths.empty()) slot_covered = false;
      slot_objects.push_back(std::move(copy));
    }
    // The core interpolator requires two ego points; a one-point snapshot
    // would silently skip every object. Rebase this instantaneous slot check.
    const std::vector<PoseWithVelocityStamped> slot_ego{
      {0.0, merge_pose, last->velocity}, {0.001, merge_pose, last->velocity}};
    if (!slot_covered || !safety::check_safety_with_rss(target_reference,
        slot_ego, bpp.vehicle_info, slot_objects, debug,
        lane_change_parameters_->safety.rss_params, true, 1.0, M_PI)) continue;
    overtake_prepare_path_ = std::move(path);
    return true;
  }
  return false;
}

bool NormalLaneChange::prepareCandidateForSearch(
  LaneChangePath & candidate, const CandidateSearchMode mode) const
{
  const auto * target = findOvertakeTarget();
  const double duration = candidate.info.duration.prepare + candidate.info.duration.lane_changing;
  const auto slot = target ? lc::predicted_merge_slot(
    common_data_ptr_->current_lanes_path, candidate.info.lane_changing_end,
    duration, *target, getCommonParam().vehicle_info) : lc::MergeSlot::Unavailable;
  if (target && slot == lc::MergeSlot::Unavailable) {
    ++unavailable_slot_candidates_;
    required_slot_prediction_time_ = std::max(required_slot_prediction_time_, duration);
    return false;
  }
  const bool ahead = slot == lc::MergeSlot::Ahead;
  if ((mode == CandidateSearchMode::Ahead && !ahead) ||
      (mode == CandidateSearchMode::Other && ahead)) return false;

  // A rear-slot/ordinary candidate must not inherit a failed front attempt's
  // cooperative assumptions. It still checks the tracked object with normal RSS.
  candidate.cooperative_overtake_target.reset();
  if (ahead && overtake_.phase == OvertakePhase::Ready) {
    candidate.cooperative_overtake_target = overtake_.target;
  }
  if (ahead) ++ahead_candidates_checked_;
  else ++other_candidates_checked_;
  return true;
}

std::optional<std::vector<PoseWithVelocityStamped>> NormalLaneChange::finalVelocityPrediction(
  const LaneChangePath & candidate) const
{
  autoware_planning_msgs::TrajectoryConstPtr feedback;
  ros::Time received;
  {
    std::lock_guard<std::mutex> lock(final_trajectory_mutex_);
    feedback = final_trajectory_;
    received = final_trajectory_received_;
  }
  const auto reject = [&](const char * reason)
    -> std::optional<std::vector<PoseWithVelocityStamped>> {
    overtake_feedback_reason_ = reason;
    return std::nullopt;
  };
  const auto now = ros::Time::now();
  feedback_candidate_frame_ = candidate.path.header.frame_id;
  feedback_trajectory_frame_ = feedback ? feedback->header.frame_id : "";
  if (!feedback || feedback->points.size() < 2) return reject("final_trajectory_missing");
  if (now < received || (now-received).toSec() > lane_change_parameters_->overtake.feedback_timeout ||
      feedback->header.stamp.isZero() || now < feedback->header.stamp ||
      (now-feedback->header.stamp).toSec() > lane_change_parameters_->overtake.feedback_timeout) {
    return reject("final_trajectory_stale");
  }
  if (feedback->header.frame_id.empty() || candidate.path.header.frame_id.empty()) {
    return reject("feedback_frame_missing");
  }
  if (feedback->header.frame_id != candidate.path.header.frame_id) return reject("feedback_frame_mismatch");
  const auto nearest = std::min_element(feedback->points.begin(), feedback->points.end(),
    [&](const auto & a, const auto & b) {
      return autoware_utils::calc_distance2d(a.pose, getEgoPose()) <
             autoware_utils::calc_distance2d(b.pose, getEgoPose());
    });
  if (autoware_utils::calc_distance2d(nearest->pose, getEgoPose()) > 3.0) {
    return reject("feedback_off_route");
  }
  const auto & reference = common_data_ptr_->current_lanes_path.points;
  if (reference.size() < 2) return reject("reference_unavailable");
  const double end = autoware::motion_utils::calcSignedArcLength(reference,
    getEgoPosition(), candidate.info.lane_changing_end.position);
  std::vector<double> positions, velocities;
  for (const auto & point : feedback->points) {
    const double s = autoware::motion_utils::calcSignedArcLength(
      reference, getEgoPosition(), point.pose.position);
    const auto segment = autoware::motion_utils::findNearestSegmentIndex(
      candidate.path.points, point.pose.position);
    const auto & a = candidate.path.points[segment].point.pose.position;
    const auto & b = candidate.path.points[segment + 1].point.pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y;
    const double ratio = std::clamp(((point.pose.position.x-a.x)*dx +
      (point.pose.position.y-a.y)*dy)/std::max(1e-9, dx*dx+dy*dy), 0.0, 1.0);
    if (s >= 0 && s <= end + 1.0 && std::hypot(point.pose.position.x-a.x-ratio*dx,
                            point.pose.position.y-a.y-ratio*dy) > 0.5) {
      return reject("feedback_maneuver_geometry_mismatch");
    }
    if (!std::isfinite(s) || !std::isfinite(point.longitudinal_velocity_mps)) return reject("feedback_nonfinite");
    if (!positions.empty() && s <= positions.back()+1e-3) continue;
    positions.push_back(s);
    velocities.push_back(std::max(0.0, static_cast<double>(point.longitudinal_velocity_mps)));
  }
  if (positions.size() < 2 || positions.front() > 1.0) return reject("feedback_off_route");
  if (positions.back() < end) return reject("feedback_shorter_than_maneuver");
  auto capped = candidate;
  // Recheck the final geometry and speed themselves. Projecting only its speed
  // onto the old candidate silently ignores optimizer-induced lateral changes.
  capped.path.points.clear();
  for (const auto & point : feedback->points) {
    PathPointWithLaneId output;
    output.point.pose = point.pose;
    output.point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    capped.path.points.push_back(output);
  }
  capped.type = lane_change::PathType::SpatialSpeedProfile;
  try {
    const auto predictions = lc::convert_to_predicted_paths(common_data_ptr_, capped, 1, false);
    if (predictions.empty() || predictions.front().empty()) return reject("feedback_prediction_empty");
    overtake_feedback_reason_ = "final_speed_profile_checked";
    return predictions.front();
  } catch (const std::exception &) { return reject("feedback_speed_profile_stops"); }
}
}
