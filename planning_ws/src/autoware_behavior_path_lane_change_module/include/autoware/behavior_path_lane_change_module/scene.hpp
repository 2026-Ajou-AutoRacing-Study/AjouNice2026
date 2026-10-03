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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_lane_change_module/base_class.hpp"
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/utils/bounded_output_policy.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace utils::lane_change { struct PreparedSafetyWorld; }
namespace utils::path_safety_checker { class PreparedPathProjection; }
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::behavior_path_planner::utils::path_safety_checker::
  PoseWithVelocityAndPolygonStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;
using autoware::route_handler::Direction;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_perception_msgs::PredictedObjects;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using lane_change::LanesPolygon;
using utils::path_safety_checker::ExtendedPredictedObjects;
using utils::path_safety_checker::RSSparams;

class NormalLaneChange : public LaneChangeBase
{
public:
  NormalLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
    Direction direction);

  NormalLaneChange(const NormalLaneChange &) = delete;
  NormalLaneChange(NormalLaneChange &&) = delete;
  NormalLaneChange & operator=(const NormalLaneChange &) = delete;
  NormalLaneChange & operator=(NormalLaneChange &&) = delete;
  ~NormalLaneChange() override = default;

  void update_lanes(const bool is_approved) final;

  void update_transient_data(const bool is_approved) final;
  // Recompute only pose-dependent geometry; never advance stop/stuck timers.
  void refresh_transient_geometry(const bool is_approved);
  bool acquire_fresh_output_context();

  void update_filtered_objects() final;

  void updateLaneChangeStatus() override;

  std::pair<bool, bool> getSafePath(LaneChangePath & safe_path) const override;

  LaneChangePath getLaneChangePath() const override;

  BehaviorModuleOutput getTerminalLaneChangePath() const override;

  BehaviorModuleOutput generateOutput() override;

  void extendOutputDrivableArea(BehaviorModuleOutput & output) const override;

  void insert_stop_point(
    const lanelet::ConstLanelets & lanelets, PathWithLaneId & path,
    const bool is_waiting_approval = false) override;

  void insert_stop_point_on_current_lanes(
    PathWithLaneId & path, const bool is_waiting_approval = false);

  PathWithLaneId getReferencePath() const override;

  std::optional<PathWithLaneId> extendPath() override;

  // Extend the path that will actually be published, including terminal candidates.
  std::optional<PathWithLaneId> extendPath(const PathWithLaneId & path) const;

  void resetParameters() override;

  TurnSignalInfo updateOutputTurnSignal() const override;

  bool calcAbortPath() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  PathSafetyStatus evaluateApprovedPathWithUnsafeHysteresis(
    PathSafetyStatus approved_path_safety_status) override;

  bool isRequiredStop(const bool is_trailing_object) override;

  bool hasFinishedLaneChange() const override;

  bool isAbleToReturnCurrentLane() const override;

  bool is_near_terminal_end() const final;

  bool isEgoOnPreparePhase() const override;

  bool isAbleToStopSafely() const override;

  bool hasFinishedAbort() const override;

  bool isAbortState() const override;

  std::optional<std::string> isLaneChangeRequired() final;

  bool isStoppedAtRedTrafficLight() const override;

  bool is_near_regulatory_element() const final;

  TurnSignalInfo get_current_turn_signal_info() const final;

  bool hasMissedLaneChangePath() const override;

protected:
  lanelet::ConstLanelets get_lane_change_lanes(const lanelet::ConstLanelets & current_lanes) const;

  TurnSignalInfo get_terminal_turn_signal_info() const final;

  lane_change::TargetObjects get_target_objects(
    const FilteredLanesObjects & filtered_objects,
    const lanelet::ConstLanelets & current_lanes) const;

  FilteredLanesObjects filter_objects() const;

  void filterOncomingObjects(PredictedObjects & objects) const;

  std::vector<LaneChangePhaseMetrics> get_prepare_metrics() const;
  std::vector<LaneChangePhaseMetrics> get_lane_changing_metrics(
    const PathWithLaneId & prep_segment, const LaneChangePhaseMetrics & prep_metrics,
    const double shift_length, const double dist_to_reg_element,
    lane_change::MetricsDebug & debug_metrics) const;

  bool get_lane_change_paths(LaneChangePaths & candidate_paths) const;

  bool get_path_using_frenet(
    const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
    const lane_change::TargetObjects & target_objects,
    const std::vector<std::vector<int64_t>> & sorted_lane_ids,
    LaneChangePaths & candidate_paths) const;

  bool get_path_using_path_shifter(
    const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
    const lane_change::TargetObjects & target_objects,
    const std::vector<std::vector<int64_t>> & sorted_lane_ids,
    LaneChangePaths & candidate_paths) const;

  bool check_candidate_path_safety(
    const LaneChangePath & candidate_path, const lane_change::TargetObjects & target_objects) const;
  bool check_reachable_candidate_admission(
    const LaneChangePath & candidate_path, const ros::WallTime & deadline) const;

  bool check_strict_maneuver_safety(
    const LaneChangePath & candidate, const PathWithLaneId & final_path,
    std::string & reason, bool approved, bool current_lane_following = false) const;
  std::shared_ptr<const utils::lane_change::PreparedSafetyWorld> prepare_admission_world() const;
  bool check_strict_maneuver_safety(
    const LaneChangePath & candidate, const PathWithLaneId & final_path,
    std::string & reason, bool approved, bool current_lane_following,
    const utils::lane_change::PreparedSafetyWorld * world) const;
  void enforce_strict_maneuver_safety(BehaviorModuleOutput & output);
  std::optional<utils::lane_change::SelectedOutputIdentity> selected_output_identity() const;
  void remember_selected_output(const LaneChangePath & model, const PathWithLaneId & path);
  bool defer_pre_adjacency_request() const;
  void publishStrictSafetySnapshot(
    const std::string & stage,const LaneChangePath & candidate,const PathWithLaneId & original_path,
    const PathWithLaneId * selected_path,const ros::Time & stamp,
    const std::string & original_reason,const std::string & cancel_reason,
    const std::string & brake_reason,
    const std::vector<double> * attempted_targets = nullptr) const;

  std::optional<PathWithLaneId> compute_terminal_lane_change_path() const;

  bool isValidPath(const PathWithLaneId & path) const override;

  std::optional<std::vector<ExtendedPredictedObject>> find_colliding_object_if_all_paths_collide(
    const LaneChangePath & lane_change_path,
    const std::vector<std::vector<PoseWithVelocityStamped>> & ego_predicted_paths,
    const ExtendedPredictedObjects & objects,
    const utils::path_safety_checker::RSSparams & rss_params,
    CollisionCheckDebugMap & debug_dataconst, const bool is_approved) const;

  PathSafetyStatus isLaneChangePathSafe(
    const LaneChangePath & lane_change_path,
    const std::vector<std::vector<PoseWithVelocityStamped>> & ego_predicted_paths,
    const lane_change::TargetObjects & collision_check_objects,
    const utils::path_safety_checker::RSSparams & rss_params, CollisionCheckDebugMap & debug_data,
    const bool is_approved = false) const;

  bool is_colliding(
    const LaneChangePath & lane_change_path, const ExtendedPredictedObject & obj,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const RSSparams & rss_param,
    CollisionCheckDebugMap & debug_data, const bool is_approved) const;
  bool is_colliding(
    const LaneChangePath & lane_change_path, const ExtendedPredictedObject & obj,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const RSSparams & rss_param,
    CollisionCheckDebugMap & debug_data, const bool is_approved,
    const utils::path_safety_checker::PreparedPathProjection * projection) const;

  double get_max_velocity_for_safety_check() const;

  bool is_ego_stuck() const;

  void set_stop_pose(
    const double arc_length_to_stop_pose, PathWithLaneId & path, const std::string & reason = "");

  void updateStopTime();

  double getStopTime() const { return stop_time_; }

  const lanelet::ConstLanelets & get_target_lanes() const
  {
    return common_data_ptr_->lanes_ptr->target;
  }

  void update_dist_from_intersection();

  /**
   * @brief Check whether the ego vehicle is currently located in either the
   *        current lanes or the target lanes used for lane change planning.
   *
   * @return true   ego vehicle is on one of the current or target lanelets.
   */
  bool is_ego_in_current_or_target_lanes() const final;

  std::vector<PathPointWithLaneId> path_after_intersection_;
  uint64_t selected_output_cycle_{0};
  bool transient_is_approved_{true};
  std::optional<LaneChangePath> selected_output_model_;
  std::optional<utils::lane_change::SelectedOutputIdentity> selected_output_identity_;
  double stop_time_{0.0};
};
}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_
