// Diagnostic observation only: counterfactual checks never approve a maneuver.
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/safety_check.hpp"

#include <nlohmann/json.hpp>
#include <std_msgs/String.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>

namespace autoware::behavior_path_planner
{
namespace
{
using Json = nlohmann::json;
namespace check = utils::path_safety_checker;
Json pose_json(const geometry_msgs::Pose & p)
{
  return {{"x", p.position.x}, {"y", p.position.y}, {"yaw", tf2::getYaw(p.orientation)}};
}
Json polygon_json(const autoware_utils_geometry::Polygon2d & polygon)
{
  auto result = Json::array();
  for (const auto & p : polygon.outer()) result.push_back({p.x(), p.y()});
  return result;
}
Json params_json(const check::RSSparams & p)
{
  return {{"front_deceleration", p.front_vehicle_deceleration},
    {"rear_deceleration", p.rear_vehicle_deceleration},
    {"reaction_time", p.rear_vehicle_reaction_time},
    {"safety_time", p.rear_vehicle_safety_time_margin},
    {"minimum_longitudinal", p.longitudinal_distance_min_threshold},
    {"lateral_margin", p.lateral_distance_max_threshold}, {"policy", p.extended_polygon_policy}};
}
}

void NormalLaneChange::captureSafetyDiagnostic(
  const char * stage, const ExtendedPredictedObject & object,
  const LaneChangePath & candidate, const std::vector<PoseWithVelocityStamped> & prediction,
  const RSSparams & execution_params, const check::RSSCollisionTrace & trace,
  const CollisionCheckDebug & debug, const double prepare_end_time) const
{
  if (!debug.is_safe) ++safety_rejected_pairs_;
  // First observation for each object/stage, at most four per planning cycle.
  // Collision failures take precedence over informational coverage observations.
  // Search counts are complete; the snapshots are intentionally bounded.
  for (const auto & sample : safety_diagnostic_samples_) {
    if (sample.stage == stage && sample.object.uuid == object.uuid) return;
  }
  if (safety_diagnostic_samples_.size() >= 4) {
    if (debug.is_safe) return;
    const auto observation = std::find_if(safety_diagnostic_samples_.begin(),
      safety_diagnostic_samples_.end(), [](const auto & sample) { return sample.debug.is_safe; });
    if (observation == safety_diagnostic_samples_.end()) return;
    safety_diagnostic_samples_.erase(observation);
  }
  safety_diagnostic_samples_.push_back(
    {stage, object, candidate, prediction, execution_params, trace, debug, prepare_end_time});
}

void NormalLaneChange::publishApprovedSafetyDiagnostics(const bool safe, const bool trailing) const
{
  publishSafetyDiagnostics(0.0, true, safe, trailing);
}

void NormalLaneChange::publishSafetyDiagnostics(
  const double search_ms, const bool approved, const bool safe, const bool trailing) const
{
  if (!safety_diagnostics_pub_ || !safety_diagnostics_pub_.getNumSubscribers()) return;
  const auto started = std::chrono::steady_clock::now();
  const auto & vehicle = common_data_ptr_->bpp_param_ptr->vehicle_info;
  const auto & params = *common_data_ptr_->lc_param_ptr;
  const auto ids = [](const auto & lanes) {
    auto result = Json::array();
    for (const auto & lane : lanes) result.push_back(lane.id());
    return result;
  };
  const auto uuid_text = [](const auto & id) {
    std::ostringstream text;
    for (auto byte : id.uuid) text << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    return text.str();
  };
  Json document{{"schema_version", 2}, {"stamp", ros::Time::now().toSec()},
    {"check_context", approved ? (isAbortState() ? "approved_abort" : "approved_lane_change") : "candidate_search"},
    {"safety_check_mode", "phase_scoped"},
    {"side", direction_ == Direction::LEFT ? "left" : "right"},
    {"valid", status_.is_valid_path}, {"safe", approved ? safe : status_.is_safe},
    {"unsafe_trailing", trailing},
    {"execution_recovery_time_s", params.execution_recovery_time},
    {"always_update_approved", params.always_update_approved},
    {"allow_decelerating_lane_change", params.trajectory.allow_decelerating_lane_change},
    {"prediction_coverage_pending", activeSafetyPath().prediction_coverage_deferred},
    {"required_recovery_time_s", std::isfinite(execution_required_recovery_time_)
      ? Json(execution_required_recovery_time_) : Json(nullptr)},
    {"ego_pose", pose_json(getEgoPose())}, {"ego_speed", getEgoVelocity()},
    {"current_lanes", ids(get_current_lanes())}, {"target_lanes", ids(get_target_lanes())},
    {"search_ms", search_ms}, {"search_limit_ms", params.time_limit},
    {"rejected_pairs", safety_rejected_pairs_}, {"snapshot_limit", 4},
    {"sample_policy", "first_observation_per_object_and_stage_failures_first"},
    {"prepare_params", params_json(params.safety.rss_params_for_prepare)},
    {"execution_params", params_json(params.safety.rss_params)},
    {"counterfactual_affects_decision", false}, {"samples", Json::array()}};
  document["candidate_search"] = {{"prepare_metrics",search_prepare_metrics_},
    {"generated",search_generated_}, {"safety_checked",search_checked_},
    {"safe",search_safe_}, {"budget_exhausted",search_budget_exhausted_},
    {"compact_sampling",params.trajectory.compact_sampling},
    {"abort_reentry_rejections",abort_reentry_rejections_},
    {"rolling_plan",rolling_plan_reason_}, {"replan_requested",execution_replan_requested_},
    {"recovery_timeout_is_failure",false}};
  document["overtake"] = {{"enabled", params.overtake.enable},
    {"assume_cooperative_npc", params.overtake.assume_cooperative_npc},
    {"phase", overtakePhaseName()}, {"reason", overtake_.reason},
    {"target_uuid", overtake_.target ? Json(uuid_text(*overtake_.target)) : Json(nullptr)},
    {"measured_rear_body_gap_m", overtake_.gap}, {"minimum_gap_m", params.overtake.minimum_gap},
    {"npc_reaction_time", params.overtake.npc_reaction_time},
    {"npc_max_deceleration", params.overtake.npc_max_deceleration},
    {"downstream_feedback", overtake_feedback_reason_},
    {"feedback_candidate_frame", feedback_candidate_frame_},
    {"feedback_trajectory_frame", feedback_trajectory_frame_},
    {"ahead_candidates_checked", ahead_candidates_checked_},
    {"other_candidates_checked", other_candidates_checked_},
    {"unavailable_slot_candidates", unavailable_slot_candidates_},
    {"required_slot_prediction_time", required_slot_prediction_time_},
    {"selected_merge_slot", selected_merge_slot_},
    {"continuous_search", true}};

  if (status_.is_valid_path) {
    const auto & candidate = approved ? activeSafetyPath() : status_.lane_change_path;
    document["selected_candidate"] = {
      {"prepare_acceleration", candidate.info.longitudinal_acceleration.prepare},
      {"execution_acceleration", candidate.info.longitudinal_acceleration.lane_changing},
      {"prepare_velocity", candidate.info.velocity.prepare},
      {"execution_velocity", candidate.info.velocity.lane_changing},
      {"prepare_duration", candidate.info.duration.prepare},
      {"execution_duration", candidate.info.duration.lane_changing}};
    document["selected_candidate"]["frame_id"] = candidate.path.header.frame_id;
  }
  if (const auto stop = getStopPose()) {
    document["stop_pose"] = {{"pose", pose_json(stop->pose)}, {"reason", stop->detail}};
  }

  for (const auto & sample : safety_diagnostic_samples_) {
    const auto & trace = sample.trace;
    const auto & debug = sample.debug;
    const auto & candidate = sample.candidate;
    const double split = sample.prepare_end_time;
    std::ostringstream uuid;
    for (const auto byte : sample.object.uuid.uuid) {
      uuid << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    const auto scoped_safe = [&](const auto & object, const auto & execution_params) {
      CollisionCheckDebugMap ignored;
      return utils::lane_change::check_safety_by_phase(
        candidate.path, sample.ego_prediction, vehicle, {object}, ignored,
        params.safety.rss_params_for_prepare, execution_params, split,
        params.safety.collision_check.th_yaw_diff).safe;
    };
    Json record{{"object_uuid", uuid.str()}, {"object_label", sample.object.classification.label},
      {"object_initial_pose", pose_json(sample.object.initial_pose)},
      {"object_initial_velocity", {sample.object.initial_twist.linear.x,
        sample.object.initial_twist.linear.y}},
      {"checked_stage", sample.stage}, {"unsafe_reason", debug.unsafe_reason},
      {"coverage_warning", sample.stage == "coverage" ? "prediction_coverage_short" : ""},
      {"required_prediction_time", sample.ego_prediction.empty() ? 0.0 :
        std::max(sample.ego_prediction.back().time, approved ? 0.0 : candidate.info.duration.sum())},
      {"distance_policy", sample.stage == "cooperative_rear" ? "cooperative_rear" : "rss"},
      {"collision_time", trace.time}, {"predicted_path_index", trace.predicted_path_index},
      {"predicted_path_confidence", trace.path_confidence},
      {"prepare_duration", split}, {"execution_duration", candidate.info.duration.lane_changing},
      {"predicted_phase", trace.time < split ? "prepare" : "execution"},
      {"outside_stage_window", sample.stage == "prepare" ? trace.time >= split : trace.time < split},
      {"ego_velocity", trace.ego_velocity}, {"object_velocity", trace.object_velocity},
      {"object_is_front", trace.object_is_front}, {"rss_distance", trace.rss_distance},
      {"longitudinal_extension", trace.longitudinal_extension},
      {"lateral_margin", trace.lateral_margin},
      {"expected_ego_pose", pose_json(debug.expected_ego_pose)},
      {"expected_object_pose", pose_json(debug.expected_object_pose)},
      {"extended_ego_polygon", polygon_json(debug.extended_ego_polygon)},
      {"extended_object_polygon", polygon_json(debug.extended_object_polygon)},
      {"candidate_prepare_acceleration", candidate.info.longitudinal_acceleration.prepare},
      {"candidate_execution_acceleration", candidate.info.longitudinal_acceleration.lane_changing},
      {"phase_split_safe_for_object", scoped_safe(sample.object, sample.execution_params)},
      {"ego_prediction", Json::array()}, {"object_predictions", Json::array()}};
    for (const auto & point : sample.ego_prediction) {
      record["ego_prediction"].push_back({point.time, point.pose.position.x,
        point.pose.position.y, tf2::getYaw(point.pose.orientation), point.velocity});
    }
    for (const auto & path : sample.object.predicted_paths) {
      Json prediction{{"confidence", path.confidence}, {"points", Json::array()}};
      for (const auto & point : path.path) {
        prediction["points"].push_back({point.time, point.pose.position.x,
          point.pose.position.y, tf2::getYaw(point.pose.orientation), point.velocity});
      }
      record["object_predictions"].push_back(std::move(prediction));
    }
    // For the first snapshot, check every filtered target object with this exact
    // ego hypothesis. This is evidence about one candidate, not an approval.
    if (!approved && &sample == &safety_diagnostic_samples_.front()) {
      const auto targets = get_target_objects(filtered_objects_, get_current_lanes());
      bool all_safe = true;
      size_t count = 0;
      for (const auto * objects : {&targets.leading, &targets.trailing}) {
        for (const auto & object : *objects) {
          ++count;
          const auto & execution_params = object.initial_twist.linear.x <=
            params.safety.th_stopped_object_velocity ? params.safety.rss_params_for_parked :
            params.safety.rss_params;
          if (!scoped_safe(object, execution_params)) all_safe = false;
        }
      }
      record["phase_split_safe_all_target_objects"] = all_safe;
      record["counterfactual_object_count"] = count;
      record["counterfactual_scope"] = "same_candidate_and_ego_hypothesis_all_filtered_target_objects";
    }
    document["samples"].push_back(std::move(record));
  }
  document["diagnostic_compute_ms"] = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  std_msgs::String message;
  message.data = document.dump();
  safety_diagnostics_pub_.publish(message);
}
}
