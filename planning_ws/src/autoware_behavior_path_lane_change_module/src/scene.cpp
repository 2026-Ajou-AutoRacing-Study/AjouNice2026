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
#include "autoware/behavior_path_planner_common/interface/fresh_input_provider.hpp"

#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepare_candidate_selection.hpp"
#include "autoware/behavior_path_lane_change_module/utils/full_envelope_preference.hpp"
#include "autoware/behavior_path_lane_change_module/utils/reachable_admission.hpp"
#include "autoware/behavior_path_lane_change_module/utils/preferred_request_defer.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/world_candidate.hpp"
#include "autoware/behavior_path_lane_change_module/utils/generation_rejection.hpp"
#include "autoware/behavior_path_lane_change_module/utils/speed_refresh.hpp"
#include "autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepared_safety_world.hpp"
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/prepared_path_projection.hpp>
#include "autoware/behavior_path_lane_change_module/utils/measured_motion_policy.hpp"
#include "autoware/behavior_path_lane_change_module/utils/bounded_output_policy.hpp"
#include "autoware/behavior_path_lane_change_module/utils/selected_output_prediction.hpp"
#include "autoware/behavior_path_lane_change_module/utils/pre_adjacency_policy.hpp"
#include "autoware/behavior_path_lane_change_module/utils/future_mandatory_chain.hpp"
#include "autoware/behavior_path_lane_change_module/utils/future_handoff.hpp"
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include "autoware/behavior_path_lane_change_module/utils/lateral_speed_cap.hpp"
#include "autoware/behavior_path_lane_change_module/utils/hard_speed_envelope.hpp"
#include "autoware/behavior_path_lane_change_module/utils/rss_phase.hpp"
#include "autoware/behavior_path_lane_change_module/utils/frenet_safety.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
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
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <std_msgs/String.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <mutex>
#include "autoware/behavior_path_lane_change_module/utils/current_lane_cancel.hpp"
#include "autoware/behavior_path_lane_change_module/utils/preferred_path_hazard.hpp"
#include "autoware/behavior_path_lane_change_module/utils/unapproved_candidate_diagnostic.hpp"
#include "autoware/behavior_path_lane_change_module/utils/safety_snapshot.hpp"
#include <iomanip>
#include <sstream>

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
#include <ros/param.h>

namespace autoware::behavior_path_planner
{
using autoware_adapi_v1_msgs::OperationModeState;
using autoware::motion_utils::calcSignedArcLength;
using utils::lane_change::create_lanes_polygon;
namespace calculation = utils::lane_change::calculation;
using utils::path_safety_checker::filter::velocity_filter;

namespace
{
thread_local std::string * candidate_reason_sink=nullptr;
struct CandidateReasonScope {
  std::string * previous;
  explicit CandidateReasonScope(std::string * sink):previous(candidate_reason_sink){candidate_reason_sink=sink;}
  ~CandidateReasonScope(){candidate_reason_sink=previous;}
};
void candidateReason(const std::string & value) {
  if(candidate_reason_sink)*candidate_reason_sink=value;
}
template<class Vehicle>
std::unique_ptr<utils::lane_change::CurrentExitCertificate> currentExitCertificate(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::ConstLanelets & current,const lanelet::ConstLanelets & target,
  const Vehicle & vehicle,size_t begin,size_t end,bool normal)
{
  bool enabled=false,strict=false,continuous=false,partial=false;
  ros::param::getCached("/planning/lc_certified_target_only_speed_reference",enabled);
  ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
  ros::param::getCached("/planning/lc_continuous_bounded_output",continuous);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial);
  if(!enabled||!strict||!continuous||!partial||!normal||begin>end)return nullptr;
  using Polygon=decltype(utils::toPolygon2d(std::declval<lanelet::ConstLanelet>()));
  const auto combine=[](const lanelet::ConstLanelets & lanes) {
    boost::geometry::model::multi_polygon<Polygon> result;
    for(const auto & lane:lanes) {
      const auto polygon=utils::toPolygon2d(lane);
      if(result.empty()){result.push_back(polygon);continue;}
      decltype(result) merged;boost::geometry::union_(result,polygon,merged);
      result=std::move(merged);
    }
    return result;
  };
  return std::make_unique<utils::lane_change::CurrentExitCertificate>(
    path,combine(current),combine(target),vehicle,begin,end);
}
template<class Certificate,class... Args>
std::vector<double> certifiedEnvelope(Certificate make_certificate,const Args &... args)
{
  try {return utils::lane_change::make_hard_speed_envelope(args...);}
  catch(const std::logic_error & error) {
    const std::string reason=error.what();
    if(reason.find("speed reference does not cover candidate reference=current ")!=0 &&
       reason.find("speed reference does not cover candidate reference=upstream ")!=0)throw;
    const auto certificate=make_certificate();
    if(!certificate)throw;
    return utils::lane_change::make_hard_speed_envelope_certified(args...,certificate.get());
  }
}
ros::Publisher & strictFailurePublisher()
{
  static ros::NodeHandle nh;
  static auto publisher = nh.advertise<std_msgs::String>(
    "/planning/debug/lane_change_safety_failure", 10, false);
  return publisher;
}
ros::Publisher & strictSnapshotPublisher()
{
  static ros::NodeHandle nh;
  static auto publisher=nh.advertise<std_msgs::String>(
    "/planning/debug/lane_change_safety_snapshot",1,false);
  return publisher;
}
}  // namespace

NormalLaneChange::NormalLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: LaneChangeBase(parameters, type, direction)
{
  // Register before driving so the first failure is not lost to connection setup.
  (void)strictFailurePublisher();
  (void)strictSnapshotPublisher();
  stop_watch_.tic(getModuleTypeStr());
  stop_watch_.tic("stop_time");
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
  transient_is_approved_=is_approved;
  ++selected_output_cycle_;
  selected_output_model_.reset();
  selected_output_identity_.reset();
  if (
    !common_data_ptr_ || !common_data_ptr_->is_data_available() ||
    !common_data_ptr_->is_lanes_available()) {
    return;
  }

  refresh_transient_geometry(is_approved);
  auto & transient_data = common_data_ptr_->transient_data;

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
  RCLCPP_DEBUG(
    logger_, "is_ego_near_current_terminal_start: %s",
    (transient_data.is_ego_near_current_terminal_start ? "true" : "false"));
}

void NormalLaneChange::refresh_transient_geometry(const bool is_approved)
{
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
}

void NormalLaneChange::update_filtered_objects()
{
  filtered_objects_ = filter_objects();
}

void NormalLaneChange::updateLaneChangeStatus()
{
  ++selected_output_cycle_;
  selected_output_model_.reset();
  selected_output_identity_.reset();
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  bool skip_unrequested_avoidance=false;
  ros::param::getCached("/planning/lc_skip_unrequested_avoidance_search",skip_unrequested_avoidance);
  // updateData has already refreshed special avoidance targets and transient
  // geometry. Never prune an approved/abort maneuver or another module type.
  if(skip_unrequested_avoidance && getModuleType()==LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE &&
     !transient_is_approved_ && current_lane_change_state_==LaneChangeStates::Normal && common_data_ptr_ &&
     common_data_ptr_->is_data_available() && !specialRequiredCheck()) {
    status_=LaneChangeStatus{};
    terminal_lane_change_path_.reset();
    lane_change_debug_.valid_paths.clear();
    status_.lane_change_path.path.header=getRouteHeader();
    ROS_INFO_THROTTLE(1.0,"Avoidance LC candidate search skipped: fresh special request false");
    return;
  }
  const auto [found_valid_path, found_safe_path] = getSafePath(status_.lane_change_path);

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
  }

  // This is the final extended/cut candidate, before RTC/approval. The input
  // epoch is the generation epoch, not a newly invented fresh publication.
  utils::lane_change::generation_diagnostic_nonfatal([&] {
    bool audit=false;
    ros::param::getCached("/planning/lc_candidate_reason_audit",audit);
    utils::lane_change::audit_unapproved_candidate(audit,
      getModuleType()==LaneChangeModuleType::NORMAL,
      transient_is_approved_||isAbortState()||isStopState(),found_safe_path,[&] {
        publishStrictSafetySnapshot("unapproved_request_candidate",safe_path,
          safe_path.path,nullptr,ros::Time::now(),"native_candidate_complete_before_approval","","");
      });
  });
  return {true, found_safe_path};
}

bool NormalLaneChange::defer_pre_adjacency_request() const
{
  namespace lc=utils::lane_change;
  lc::PreAdjacencyContext c;
  bool strict=false,bounded=false,grid=false;
  ros::param::getCached("/planning/lc_defer_pre_adjacency_normal",c.enabled);
  if(!c.enabled)return false;
  ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
  ros::param::getCached("/planning/lc_continuous_bounded_output",bounded);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",grid);
  c.enabled=c.enabled&&strict&&bounded&&grid;
  c.normal=getModuleType()==LaneChangeModuleType::NORMAL && getDirection()==Direction::LEFT;
  c.approved=transient_is_approved_ || isAbortState() || isStopState();
  if(!c.enabled || !c.normal || c.approved)return false;
  try {
    const auto handler=getRouteHandler();
    if(!handler || !handler->isHandlerReady() || !planner_data_->self_odometry ||
       !planner_data_->dynamic_object || !common_data_ptr_->current_acceleration)return false;
    std::ostringstream uuid;
    for(const auto byte:handler->getRouteUuid().uuid)
      uuid<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
    if(uuid.str()!=passing_intent::route_uuid)return false;
    const auto & current=get_current_lanes();
    const auto & target=get_target_lanes();
    if(current.size()<2 || target.empty() || current[0].id()!=201086 ||
       current[1].id()!=201456 || target.front().id()!=201457)return false;
    const auto shared=target.front().rightBound();
    if(shared.empty() || current[0].leftBound().empty() ||
       current[1].leftBound().empty())return false;
    // Exact live v85 topology, not an XY-only region or a trusted config hash.
    c.topology_verified=shared.id()==105497 && shared.front().id()==231223 &&
      current[1].leftBound().id()==shared.id() &&
      current[0].leftBound().back().id()==shared.front().id() &&
      std::hypot(shared.front().x()-195.52914932894055,
                 shared.front().y()-801.3759934902191)<1e-3;
    c.chain_valid=true;
    const auto graph=handler->getRoutingGraphPtr();
    if(!graph)return false;
    for(const auto & lane:current)
      if(!handler->isRouteLanelet(lane))c.chain_valid=false;
    if(!handler->isRouteLanelet(target.front()))c.chain_valid=false;
    for(size_t i=1;i<current.size();++i) {
      const auto next=graph->following(current[i-1]);
      if(std::none_of(next.begin(),next.end(),[&](const auto & lane){return lane.id()==current[i].id();}))
        c.chain_valid=false;
    }
    const double now=ros::Time::now().toSec();
    const double odom=planner_data_->self_odometry->header.stamp.toSec();
    const double objects=planner_data_->dynamic_object->header.stamp.toSec();
    const double acceleration=common_data_ptr_->current_acceleration->header.stamp.toSec();
    c.fresh_inputs=odom>0 && objects>0 && acceleration>0 &&
      std::isfinite(now) && now-odom>=-.01 && now-odom<=.5 &&
      odom-objects>=-.01 && odom-objects<=.5 && now-acceleration>=-.01 &&
      now-acceleration<=.5 && planner_data_->self_odometry->header.frame_id==
        planner_data_->dynamic_object->header.frame_id;
    const auto & transient=common_data_ptr_->transient_data;
    c.distances_valid=transient.has_valid_lc_distance;
    c.near_terminal=is_near_terminal_end();
    c.target_start=calculation::calc_ego_dist_to_lanes_start(common_data_ptr_,current,target);
    c.remaining=transient.dist_to_terminal_end;
    c.required_lc=transient.current_dist_buffer.max;
    c.speed=getEgoVelocity();
    c.acceleration=common_data_ptr_->current_acceleration->accel.accel.linear.x;
    c.front_margin=getCommonParam().vehicle_info.max_longitudinal_offset_m;
    const auto & rss=lane_change_parameters_->safety.rss_params;
    c.reaction=rss.rear_vehicle_reaction_time+rss.rear_vehicle_safety_time_margin;
    double validator=0,smoother=0,jerk=0;
    if(!ros::param::getCached("/planning_validator/trajectory_checker/forward_trajectory_length/acceleration",validator) ||
       !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother) ||
       !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jerk) ||
       !std::isfinite(validator)||!std::isfinite(smoother)||!std::isfinite(jerk)||
       validator>=0||smoother>=0||jerk>=0||
       !std::isfinite(lane_change_parameters_->trajectory.min_longitudinal_acc)||
       lane_change_parameters_->trajectory.min_longitudinal_acc>=0)return false;
    c.deceleration=std::min({-validator,-smoother,-lane_change_parameters_->trajectory.min_longitudinal_acc});
    c.jerk=-jerk;
    const auto decision=lc::defer_pre_adjacency(c);
    if(decision.defer)ROS_INFO_THROTTLE(1.0,
      "LC pre-adjacency request deferred: target_start=%.3f remaining=%.3f reserve=%.3f; existing lane-follow remains owner",
      c.target_start,c.remaining,decision.reserve);
    // This approves NO trajectory. Existing lane-follow/BVP/cruise/control
    // retain their normal checks and may slow/stop for an obstacle. Re-evaluate
    // the request next cycle; legal adjacency/mandatory reserve end deferral.
    return decision.defer;
  } catch(const std::exception & error) {
    ROS_DEBUG_THROTTLE(1.0,"LC pre-adjacency deferral unavailable: %s",error.what());
    return false;
  }
}

std::optional<std::string> NormalLaneChange::isLaneChangeRequired()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Observe THIS request callback, not a later candidate or publication epoch.
  // Diagnostic failures must never change the request decision.
  bool request_audit=false;
  double audit_now=0,audit_odom=0,audit_raw=0,audit_acc=0;
  try {
    ros::param::getCached("/planning/lc_candidate_reason_audit",request_audit);
    request_audit=request_audit&&getModuleType()==LaneChangeModuleType::NORMAL&&
      getDirection()==Direction::LEFT&&!transient_is_approved_&&!isAbortState()&&!isStopState();
    if(request_audit) {
      audit_now=ros::Time::now().toSec();
      if(planner_data_&&planner_data_->self_odometry)audit_odom=planner_data_->self_odometry->header.stamp.toSec();
      if(planner_data_&&planner_data_->dynamic_object)audit_raw=planner_data_->dynamic_object->header.stamp.toSec();
      if(common_data_ptr_&&common_data_ptr_->current_acceleration)audit_acc=common_data_ptr_->current_acceleration->header.stamp.toSec();
    }
  }catch(...){request_audit=false;}
  const char * audit_reason="native_inputs_unavailable";
  const char * audit_utility="not_evaluated";
  bool audit_allowed=false,audit_near=false,audit_extended=false;
  size_t audit_containing=0,audit_current_count=0,audit_target_count=0;
  double audit_min=0,audit_terminal=0,audit_remaining=0,audit_reserve=0,audit_horizon=0;
  double audit_target_gap=0,audit_current_gap=0,audit_target_rss=0,audit_current_rss=0;
  double audit_speed=0,audit_target_speed=0,audit_current_speed=0,audit_max_acc=0;
  bool audit_predicate=false,audit_target_lead=false,audit_current_lead=false;
  bool audit_refreshed=false;
  uint64_t audit_input_cycle=0;
  struct RequestAuditExit {
    std::function<void()> emit;
    ~RequestAuditExit() noexcept {try{emit();}catch(...){}}
  };
  std::optional<RequestAuditExit> request_audit_exit;
  try {if(request_audit) {
  request_audit_exit.emplace();
  request_audit_exit->emit=[&] {
    if(!request_audit)return;
    static std::string last_key;
    static double last_stamp=0;
    const std::string key=std::string(audit_reason)+":"+audit_utility+(audit_allowed?":1":":0");
    if(key==last_key&&audit_now>=last_stamp&&audit_now-last_stamp<1.)return;
    const auto finite=[](double value) {
      if(!std::isfinite(value))return std::string("null");
      std::ostringstream number;number<<std::setprecision(17)<<value;return number.str();
    };
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"stamp\":"<<audit_now<<",\"odom\":"<<audit_odom<<",\"objects\":"<<audit_raw
      <<",\"acceleration\":"<<audit_acc<<",\"requested\":"<<(audit_allowed?"true":"false")
      <<",\"reason\":\""<<audit_reason<<"\",\"utility\":\""<<audit_utility
      <<"\",\"native_min\":"<<finite(audit_min)<<",\"terminal\":"<<finite(audit_terminal)
      <<",\"near_terminal\":"<<(audit_near?"true":"false")
      <<",\"extended\":"<<(audit_extended?"true":"false")
      <<",\"containing\":"<<audit_containing<<",\"current_count\":"<<audit_current_count
      <<",\"target_count\":"<<audit_target_count<<",\"remaining\":"<<finite(audit_remaining)
      <<",\"stop_plus_travel\":"<<finite(audit_reserve)<<",\"H\":"<<finite(audit_horizon)
      <<",\"target_gap\":"<<finite(audit_target_gap)<<",\"current_gap\":"<<finite(audit_current_gap)
      <<",\"target_rss\":"<<finite(audit_target_rss)<<",\"current_rss\":"<<finite(audit_current_rss)
      <<",\"predicate_evaluated\":"<<(audit_predicate?"true":"false")
      <<",\"target_lead\":"<<(audit_target_lead?"true":"false")
      <<",\"current_lead\":"<<(audit_current_lead?"true":"false")
      <<",\"speed\":"<<finite(audit_speed)<<",\"target_speed\":"<<finite(audit_target_speed)
      <<",\"current_speed\":"<<finite(audit_current_speed)<<",\"max_acc\":"<<finite(audit_max_acc)
      <<",\"input_refreshed\":"<<(audit_refreshed?"true":"false")<<",\"input_cycle\":"<<audit_input_cycle<<"}";
    static ros::NodeHandle nh;
    static auto pub=nh.advertise<std_msgs::String>("/planning/debug/lc_preference_decision",20);
    std_msgs::String message;message.data=out.str();pub.publish(message);
    last_key=key;last_stamp=audit_now;
  };
  }}catch(...){request_audit=false;}

  if (!common_data_ptr_) {audit_reason="common_data_missing";return {"LC unavailable: common data missing."};}
  if (!common_data_ptr_->is_data_available()) {
    audit_reason="planner_data_missing";
    return {"LC unavailable: planner data pointers missing."};
  }
  if (common_data_ptr_->lanes_ptr->current.empty()) {
    audit_reason="current_lanes_empty";
    return {"LC unavailable: current lane sequence empty."};
  }
  if (common_data_ptr_->lanes_ptr->target.empty()) {
    audit_reason="target_lanes_empty";
    return {"LC unavailable: target lane sequence empty."};
  }
  if (common_data_ptr_->lanes_ptr->target_neighbor.empty()) {
    audit_reason="target_neighbor_empty";
    return {"LC unavailable: target-neighbor lane sequence empty."};
  }

  const auto & current_lanes = common_data_ptr_->lanes_ptr->current;
  const auto & target_lanes = common_data_ptr_->lanes_ptr->target;

  if(defer_pre_adjacency_request()) {
    audit_reason="pre_adjacency_deferred";
    return {"Experiment: defer future LC reservation until legal adjacent lane begins."};
  }

  const auto defer_unhelpful_preference=[&]() {
    namespace policy=lane_change::preferred_request_defer;
    namespace lc=utils::lane_change;
    policy::Context c;
    ros::param::getCached("/planning/lc_defer_unhelpful_preferred_request",c.enabled);
    bool strict=false,bounded=false,partial=false;
    ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
    ros::param::getCached("/planning/lc_continuous_bounded_output",bounded);
    ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial);
    c.normal=getModuleType()==LaneChangeModuleType::NORMAL && getDirection()==Direction::LEFT;
    c.approved=transient_is_approved_||isAbortState()||isStopState();
    c.near_terminal=is_near_terminal_end();
    audit_near=c.near_terminal;
    if(!c.enabled||!strict||!bounded||!partial||!c.normal||c.approved||c.near_terminal) {
      audit_utility=!c.enabled?"utility_disabled":!strict?"strict_disabled":!bounded?"bounded_disabled":
        !partial?"partial_disabled":!c.normal?"not_normal_left":c.approved?"already_approved":"near_terminal";
      return false;
    }
    try {
      const auto & native=common_data_ptr_->transient_data;
      audit_min=native.current_dist_buffer.min;audit_terminal=native.dist_to_terminal_end;
      if(!native.has_valid_lc_distance ||
         !std::isfinite(native.current_dist_buffer.min) ||
         !std::isfinite(native.current_dist_buffer.max) ||
         !std::isfinite(native.dist_to_terminal_end) ||
         native.current_dist_buffer.min<0 ||
         native.current_dist_buffer.max<native.current_dist_buffer.min ||
         native.dist_to_terminal_end<=0){audit_utility="native_distance_invalid";return false;}
      const auto handler=getRouteHandler();
      auto utility_planner=planner_data_;
      auto odom=planner_data_->self_odometry;auto objects=planner_data_->dynamic_object;
      auto accel=common_data_ptr_->current_acceleration;
      const auto * utility_parameters=&getCommonParam();
      if(!handler||!handler->isHandlerReady()||!odom||!objects||!accel||
         !common_data_ptr_->acceleration_limits.enabled()){audit_utility="missing_inputs_or_acceleration_map";return false;}
      const auto epoch=handler->getRouteHeader();const auto uuid=handler->getRouteUuid();
      const auto map=handler->getLaneletMapPtr();
      const auto route_message=handler->getRouteMessage();
      double now=ros::Time::now().toSec();
      const auto valid_epoch=[&] {
        return policy::request_epoch_valid(now,odom->header.stamp.toSec(),objects->header.stamp.toSec(),
          accel->header.stamp.toSec(),odom->header.frame_id,objects->header.frame_id,epoch.frame_id);
      };
      c.fresh=valid_epoch();
      std::optional<fresh_commit::Snapshot> fresh_input;
      if(!c.fresh) {
        bool refresh_enabled=false;
        ros::param::getCached("/planning/lc_fresh_commit_input",refresh_enabled);
        if(refresh_enabled) {
          fresh_input=fresh_commit::acquire(); // At most once; actual messages, no wait/interpolation.
          if(!fresh_input||!fresh_input->planner_data||!fresh_input->owner||!fresh_input->cycle||
             !fresh_input->route_message_identity||fresh_input->route_handler!=handler.get()||
             fresh_input->map_identity!=map.get()||fresh_input->route_uuid!=uuid||
             fresh_input->route_stamp!=epoch.stamp||fresh_input->planner_data->route_handler!=handler||
             handler->getRouteMessage()!=route_message||handler->getLaneletMapPtr()!=map||
             handler->getRouteUuid()!=uuid||handler->getRouteHeader()!=epoch) {
            audit_utility="utility_fresh_identity_unavailable";return false;
          }
          utility_planner=fresh_input->planner_data;
          if(!utility_planner->self_odometry||!utility_planner->dynamic_object||!utility_planner->self_acceleration) {
            audit_utility="utility_fresh_inputs_missing";return false;
          }
          odom=utility_planner->self_odometry;objects=utility_planner->dynamic_object;
          accel=utility_planner->self_acceleration;utility_parameters=&utility_planner->parameters;
          now=fresh_input->acquired_at.toSec();c.fresh=valid_epoch();
          audit_refreshed=true;audit_input_cycle=fresh_input->cycle;
          audit_odom=odom->header.stamp.toSec();audit_raw=objects->header.stamp.toSec();audit_acc=accel->header.stamp.toSec();
        }
      }
      if(!c.fresh){audit_utility="input_epoch_invalid";return false;}
      const auto & utility_pose=odom->pose.pose;
      const auto & utility_position=utility_pose.position;
      const double utility_speed=odom->twist.twist.linear.x;
      const double age=(odom->header.stamp-objects->header.stamp).toSec();
      const auto graph=handler->getRoutingGraphPtr();if(!graph){audit_utility="missing_graph";return false;}
      // Resolve a fork only using the ACTUAL immediately next route segment's
      // unique preferred successor, shared by both legal prefixes. Otherwise
      // stop before the unresolved fork: no global preferred-ID lookup.
      if(!route_message||route_message->uuid!=uuid||route_message->header!=epoch){audit_utility="route_epoch_invalid";return false;}
      std::vector<policy::OrderedRouteSegment> ordered_route;
      for(const auto & segment:route_message->segments) {
        policy::OrderedRouteSegment item;item.preferred=segment.preferred_primitive.id;
        for(const auto & primitive:segment.primitives)item.primitives.push_back(primitive.id);
        ordered_route.push_back(std::move(item));
      }
      lanelet::ConstLanelets containing;
      for(const auto & lane:current_lanes)
        if(lc::contains_direction_compatible_pose({lane},utility_pose,(*utility_parameters).ego_nearest_yaw_threshold))
          containing.push_back(lane);
      audit_containing=containing.size();
      if(containing.size()!=1||target_lanes.empty()){audit_utility="ego_lane_not_unique_or_target_empty";return false;}
      if(fresh_input) {
        // Recompute native minimum/terminal geometry privately; never update
        // timers, object filters, approved paths, or module/shared pointers.
        auto local=std::make_shared<lane_change::CommonData>(*common_data_ptr_);
        local->lanes_ptr=std::make_shared<lane_change::Lanes>(*common_data_ptr_->lanes_ptr);
        local->self_odometry_ptr=odom;local->current_acceleration=accel;
        local->bpp_param_ptr=std::make_shared<BehaviorPathPlannerParameters>(*utility_parameters);
        local->lanes_ptr->ego_lane=containing.front();
        const double active_signal_duration=signal_activation_time_?
          (clock_.now()-signal_activation_time_.value()).toSec():0.;
        local->transient_data.lane_change_prepare_duration=calculation::calc_actual_prepare_duration(
          local,utility_speed,active_signal_duration);
        const auto distance=calculation::calc_lc_length_and_dist_buffer(local,current_lanes);
        const double terminal=calculation::calc_dist_from_pose_to_terminal_end(local,current_lanes,utility_pose);
        const double backward=calculation::calc_stopping_distance(local->lc_param_ptr);
        if(!distance||!std::isfinite(terminal)||!std::isfinite(backward)||terminal<=0||
           terminal<distance->second.min+backward) {
          audit_utility="utility_fresh_native_reserve_unavailable";return false;
        }
        local->transient_data.current_dist_buffer=distance->second;
        local->transient_data.dist_to_terminal_end=terminal;
        local->transient_data.dist_to_terminal_start=terminal-distance->second.min;
        const double max_prepare=calculation::calc_maximum_prepare_length(local);
        if(!std::isfinite(max_prepare)||max_prepare<0) {
          audit_utility="utility_fresh_prepare_invalid";return false;
        }
        c.near_terminal=c.near_terminal||local->transient_data.dist_to_terminal_start<max_prepare;
        audit_near=c.near_terminal;
        if(c.near_terminal){audit_utility="utility_fresh_near_terminal";return false;}
        audit_min=distance->second.min;audit_terminal=terminal;
      }
      const auto neighbor=graph->left(containing.front());
      if(!neighbor||std::none_of(target_lanes.begin(),target_lanes.end(),[&](const auto & lane){return lane.id()==neighbor->id();})){audit_utility="target_not_left_neighbor";return false;}
      const auto following=[&](int64_t id) {
        std::vector<int64_t> result;
        for(const auto & lane:graph->following(handler->getLaneletsFromId(id)))
          if(handler->isRouteLanelet(lane))result.push_back(lane.id());
        return result;
      };
      const auto compatible=[&](int64_t from,int64_t to) {
        const auto a=handler->getLaneletsFromId(from).centerline2d();
        const auto b=handler->getLaneletsFromId(to).centerline2d();
        if(a.size()<2||b.size()<2)return false;
        const double ax=a.back().x()-a[a.size()-2].x(),ay=a.back().y()-a[a.size()-2].y();
        const double bx=b[1].x()-b.front().x(),by=b[1].y()-b.front().y();
        const double join=std::hypot(a.back().x()-b.front().x(),a.back().y()-b.front().y());
        const double angle=std::atan2(ax*by-ay*bx,ax*bx+ay*by);
        return std::isfinite(ax+ay+bx+by+join+angle)&&std::hypot(ax,ay)>0&&std::hypot(bx,by)>0&&
          join<=1e-7&&std::abs(angle)<=autoware_utils::deg2rad(5.0);
      };
      const auto cp_ids=policy::preferred_route_prefix(containing.front().id(),ordered_route,following,compatible);
      const auto tp_ids=policy::preferred_route_prefix(neighbor->id(),ordered_route,following,compatible);
      const auto allowed=[&](int64_t id){return handler->isRouteLanelet(handler->getLaneletsFromId(id));};
      const auto base_current=policy::unique_route_prefix(containing.front().id(),allowed,following);
      const auto base_target=policy::unique_route_prefix(neighbor->id(),allowed,following);
      const auto proven=policy::compose_route_prefixes(base_current,base_target,cp_ids,tp_ids);
      audit_extended=proven.extended;
      audit_current_count=proven.current.lanes.size();audit_target_count=proven.target.lanes.size();
      if(!proven.valid){audit_utility="route_prefix_unproved";return false;}
      const auto chain=[&](const auto & ids) {
        lanelet::ConstLanelets lanes;
        for(auto id:ids)lanes.push_back(handler->getLaneletsFromId(id));
        return lanes;
      };
      const auto current=chain(proven.current.lanes),target=chain(proven.target.lanes);
      const auto path=handler->getCenterLinePath(current,0.,lanelet::geometry::length3d(lanelet::LaneletSequence(current)));
      const auto target_path=handler->getCenterLinePath(target,0.,lanelet::geometry::length3d(lanelet::LaneletSequence(target)));
      if(path.points.size()<2||target_path.points.size()<2){audit_utility="reference_path_short";return false;}
      c.remaining=calcSignedArcLength(path.points,utility_position,path.points.back().point.pose.position);
      c.speed=utility_speed;
      std::vector<std::vector<double>> prediction_times;
      for(const auto & raw:objects->objects) {
        const auto object=lc::observed_maneuver_object(raw,lane_change_parameters_->safety.collision_check.prediction_time_resolution);
        bool covered=false;
        for(const auto & hypothesis:object.predicted_paths) {
          if(!std::isfinite(hypothesis.confidence)){audit_utility="prediction_confidence_invalid";return false;}
          if(hypothesis.confidence<=0)continue;
          std::vector<double> times;
          for(const auto & point:hypothesis.path)times.push_back(point.time);
          prediction_times.push_back(std::move(times));covered=true;
        }
        if(!covered){audit_utility="prediction_missing";return false;}
      }
      c.horizon=policy::prediction_horizon(age,prediction_times);
      if(!std::isfinite(c.horizon)||c.horizon<=0){audit_utility="prediction_horizon_invalid";return false;}
      const double requested_acc=std::min((*utility_parameters).max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc);
      c.max_acc=policy::acceleration_upper_bound(accel->accel.accel.linear.x,requested_acc,
        common_data_ptr_->acceleration_limits.limit(c.speed,requested_acc));
      // No assumed speed-cap clamp: preserve a conservative calibrated upper
      // barrier and measured acceleration, not a promised control response.
      if(!std::isfinite(c.max_acc)||c.max_acc<0){audit_utility="acceleration_bound_invalid";return false;}
      double decel=0,jerk=0;
      if(!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",decel)||
         !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jerk)||decel>=0||jerk>=0){audit_utility="braking_parameters_invalid";return false;}
      decel=std::max({decel,(*utility_parameters).min_acc,lane_change_parameters_->trajectory.min_longitudinal_acc});
      const double future_v=c.speed+c.max_acc*c.horizon;
      const auto & rss=lane_change_parameters_->safety.rss_params;
      const double reaction=rss.rear_vehicle_reaction_time+rss.rear_vehicle_safety_time_margin;
      // This proven common-route continuation does not require the optional
      // three-LC preferred-lane MAX buffer. Native valid-distance and near-
      // terminal MIN-buffer priority above remain unchanged globally. Reserve
      // stopping before the proven continuation ends (or an unresolved fork),
      // NOT completion of a future LC.
      c.future_reserve=passing_intent::stop_distance(future_v,c.max_acc,-decel,-jerk)+
        future_v*reaction+(*utility_parameters).vehicle_info.max_longitudinal_offset_m;
      using Polygon=decltype(utils::toPolygon2d(current.front()));
      const auto polygon=[](const lanelet::ConstLanelets & lanes) {
        boost::geometry::model::multi_polygon<Polygon> result;
        for(const auto & lane:lanes) {const auto p=utils::toPolygon2d(lane);
          if(result.empty()){result.push_back(p);continue;}
          decltype(result) merged;boost::geometry::union_(result,p,merged);result=std::move(merged);}
        return result;
      };
      const auto cp=polygon(current),tp=polygon(target);
      bool input_path_hazard_enabled=false;
      ros::param::getCached("/planning/lc_path_hazard_preferred_request",input_path_hazard_enabled);
      const auto & input_path=prev_module_output_.path;
      c.current_gap=c.target_gap=std::numeric_limits<double>::infinity();
      const auto valid_pose=[](const geometry_msgs::Pose & pose) {
        const auto & p=pose.position;const auto & q=pose.orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        return std::isfinite(p.x+p.y+p.z+norm)&&std::abs(norm-1.)<=1e-6;
      };
      for(const auto & raw:objects->objects) {
        const auto & twist=raw.kinematics.initial_twist_with_covariance.twist;
        const auto & dimensions=raw.shape.dimensions;
        if(!valid_pose(raw.kinematics.initial_pose_with_covariance.pose)||
           !std::isfinite(twist.linear.x+twist.linear.y+twist.linear.z+
             twist.angular.x+twist.angular.y+twist.angular.z)||
           !std::isfinite(dimensions.x+dimensions.y+dimensions.z)||
           dimensions.x<=0||dimensions.y<=0||dimensions.z<=0){audit_utility="raw_shape_pose_twist_invalid";return false;}
        const auto & covariance=raw.kinematics.initial_twist_with_covariance.covariance;
        for(double value:covariance)if(!std::isfinite(value)){audit_utility="raw_covariance_nonfinite";return false;}
        if(covariance[0]<0){audit_utility="raw_covariance_negative";return false;}
        const auto object=lc::observed_maneuver_object(raw,lane_change_parameters_->safety.collision_check.prediction_time_resolution);
        if(object.predicted_paths.empty()){audit_utility="object_predictions_empty";return false;}
        const auto aligned=utils::path_safety_checker::calc_interpolated_object_pose(
          object.predicted_paths.front().path,age,object.shape);
        if(!aligned||!valid_pose(aligned->pose)||!boost::geometry::is_valid(aligned->polygon)||
           !std::isfinite(aligned->velocity)||aligned->velocity<0){audit_utility="aligned_object_invalid";return false;}
        if(autoware_utils::calc_distance2d(aligned->pose,utility_pose)>
           c.speed*c.horizon+.5*c.max_acc*c.horizon*c.horizon+(*utility_parameters).vehicle_info.vehicle_length_m)continue;
        // A legal lanelet continuation is not necessarily the published path's
        // clear corridor: at a merge the input path can meet a target-lane lead.
        // Release only this optional request deferral; all NORMAL candidate,
        // fresh-commit, bounded-output, raw RSS and RTC gates remain unchanged.
        // The immutable input path carries a route header, not a wall-time
        // publication header. Its epoch is the current manager input/handler.
        if(input_path_hazard_enabled&&input_path.points.size()>=2&&
           input_path.header.frame_id==epoch.frame_id) {
          namespace hazard=lane_change::preferred_path_hazard;
          const double input_gap=calcSignedArcLength(input_path.points,utility_position,aligned->pose.position);
          const double distance=autoware_utils::calc_distance2d(aligned->pose,utility_pose);
          const double speed=std::max(0.,aligned->velocity-std::sqrt(covariance[0]));
          const bool aligned_epoch=handler->getRouteUuid()==uuid&&handler->getRouteHeader().stamp==epoch.stamp;
          if(hazard::eligible(input_path_hazard_enabled,c.normal,c.approved,c.fresh,aligned_epoch,
               0.,c.horizon,distance,input_gap,c.speed,speed,c.max_acc,(*utility_parameters).vehicle_info.vehicle_length_m)&&
             hazard::intersects_input_path(input_path,aligned->pose,aligned->polygon,
               (*utility_parameters).vehicle_info,(*utility_parameters).ego_nearest_yaw_threshold)) {
            ROS_INFO_THROTTLE(1.0,
              "LC preferred request path hazard: release_defer=1 gap=%.3f distance=%.3f object_v=%.3f ego_v=%.3f prediction=0 H=%.3f; NORMAL candidate checks unchanged",
              input_gap,distance,speed,c.speed,c.horizon);
            {audit_utility="input_path_hazard_now";return false;}
          }
          for(const auto & hypothesis:object.predicted_paths) {
            if(!std::isfinite(hypothesis.confidence)||hypothesis.confidence<=0)continue;
            for(const auto & predicted:hypothesis.path) {
              const double t=predicted.time-age;
              const bool same_epoch=handler->getRouteUuid()==uuid&&handler->getRouteHeader().stamp==epoch.stamp;
              if(!hazard::eligible(input_path_hazard_enabled,c.normal,c.approved,c.fresh,same_epoch,
                   t,c.horizon,distance,input_gap,c.speed,speed,c.max_acc,
                   (*utility_parameters).vehicle_info.vehicle_length_m))continue;
              if(hazard::intersects_input_path(input_path,predicted.pose,predicted.polygon,
                   (*utility_parameters).vehicle_info,(*utility_parameters).ego_nearest_yaw_threshold)) {
                ROS_INFO_THROTTLE(1.0,
                  "LC preferred request path hazard: release_defer=1 gap=%.3f distance=%.3f object_v=%.3f ego_v=%.3f prediction=%.3f H=%.3f; NORMAL candidate checks unchanged",
                  input_gap,distance,speed,c.speed,t,c.horizon);
                {audit_utility="input_path_hazard_future";return false;}
              }
            }
          }
        }
        const bool in_current=boost::geometry::within(aligned->polygon,cp);
        const bool in_target=boost::geometry::within(aligned->polygon,tp);
        if(in_current&&in_target){audit_utility="object_in_both_corridors";return false;}
        if(!in_current&&!in_target) {
          if(boost::geometry::intersects(aligned->polygon,cp)||boost::geometry::intersects(aligned->polygon,tp)){audit_utility="object_straddles_corridor";return false;}
          continue;
        }
        const auto & reference=in_current?path:target_path;
        const size_t index=autoware::motion_utils::findNearestIndex(reference.points,aligned->pose.position);
        const double delta_yaw=tf2::getYaw(aligned->pose.orientation)-tf2::getYaw(reference.points[index].point.pose.orientation);
        const double heading=std::atan2(std::sin(delta_yaw),std::cos(delta_yaw));
        if(std::abs(heading)>(*utility_parameters).ego_nearest_yaw_threshold){audit_utility="object_heading_mismatch";return false;}
        double gap=std::numeric_limits<double>::infinity();
        for(const auto & corner:aligned->polygon.outer()) {
          geometry_msgs::Point p;p.x=corner.x();p.y=corner.y();
          gap=std::min(gap,calcSignedArcLength(reference.points,utility_position,p)-
            (*utility_parameters).vehicle_info.max_longitudinal_offset_m);
        }
        if(gap<=0)continue;
        const double uncertainty=std::sqrt(covariance[0]);
        const double speed=std::max(0.,aligned->velocity*std::cos(heading)+(in_current?-uncertainty:uncertainty));
        if(in_current&&gap<c.current_gap){c.current_lead=true;c.current_gap=gap;c.current_speed=speed;}
        if(in_target&&gap<c.target_gap){c.target_lead=true;c.target_gap=gap;c.target_speed=speed;}
      }
      const auto required_gap=[&](double leader_speed) {
        return std::max(utils::path_safety_checker::calc_rss_distance(leader_speed,c.speed,rss),
          utils::path_safety_checker::calc_minimum_longitudinal_length(leader_speed,c.speed,rss));
      };
      if(c.target_lead)c.target_rss_gap=required_gap(c.target_speed);
      if(c.current_lead)c.current_rss_gap=required_gap(c.current_speed);
      c.topology=handler->getRouteMessage().get()==route_message.get()&&
        handler->getRouteUuid()==uuid&&handler->getRouteHeader()==epoch&&
        handler->getLaneletMapPtr()==map&&(!fresh_input||
          (fresh_input->owner&&fresh_input->cycle==audit_input_cycle&&
           fresh_input->planner_data==utility_planner&&fresh_input->route_handler==handler.get()&&
           fresh_input->map_identity==map.get()&&fresh_input->route_message_identity));
      const bool defer=policy::evaluate(c);
      audit_utility=defer?"deferred":"predicate_false";
      audit_remaining=c.remaining;audit_horizon=c.horizon;
      audit_reserve=c.future_reserve+c.speed*c.horizon+.5*c.max_acc*c.horizon*c.horizon;
      audit_target_gap=c.target_gap;audit_current_gap=c.current_gap;
      audit_target_rss=c.target_rss_gap;audit_current_rss=c.current_rss_gap;
      audit_predicate=true;audit_target_lead=c.target_lead;audit_current_lead=c.current_lead;
      audit_speed=c.speed;audit_target_speed=c.target_speed;audit_current_speed=c.current_speed;
      audit_max_acc=c.max_acc;
      ROS_INFO_THROTTLE(1.,"LC preferred request utility: defer=%d target_gap=%.3f target_v=%.3f current_lead=%d current_gap=%.3f H=%.3f remaining=%.3f future_reserve=%.3f extended=%d prefix_end=%ld",defer,c.target_gap,c.target_speed,c.current_lead,c.current_gap,c.horizon,c.remaining,c.future_reserve,proven.extended,static_cast<long>(proven.current.lanes.back()));
      return defer;
    }catch(const std::exception & error) {
      ROS_DEBUG_THROTTLE(1.,"LC preferred request utility unavailable: %s",error.what());{audit_utility="utility_exception";return false;}
    }
  };
  if(defer_unhelpful_preference()) {
    audit_reason="optional_preference_deferred";
    return {"Experiment: retain legal current lane; close target leader and mandatory reserve preserved."};
  }

  const auto ego_dist_to_target_start =
    calculation::calc_ego_dist_to_lanes_start(common_data_ptr_, current_lanes, target_lanes);
  const auto max_prepare_length = calculation::calc_maximum_prepare_length(common_data_ptr_);

  if (ego_dist_to_target_start > max_prepare_length) {
    audit_reason="target_start_distant";
    return {"Ego is far from target lane start."};
  }

  if (is_near_regulatory_element()) {
    audit_reason="regulatory_element";
    return {"Ego is close to regulatory element."};
  }

  if (
    is_near_terminal_end() && planner_data_ && planner_data_->operation_mode &&
    planner_data_->operation_mode->mode != OperationModeState::AUTONOMOUS) {
    audit_reason="manual_near_terminal";
    return {"Ego is in MANUAL Mode and near terminal end, don't run LC module."};
  }

  audit_reason="native_request_allowed";audit_allowed=true;
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


bool NormalLaneChange::acquire_fresh_output_context()
{
  bool enabled=false;
  ros::param::getCached("/planning/lc_fresh_commit_input",enabled);
  if(!enabled||getModuleType()!=LaneChangeModuleType::NORMAL)return true;
  const auto started=ros::WallTime::now();
  const auto original=planner_data_;
  try {
    bool strict=false,bounded=false,partial=false;
    ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
    ros::param::getCached("/planning/lc_continuous_bounded_output",bounded);
    ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial);
    if(!strict||!bounded||!partial)throw std::logic_error("fresh_commit_requires_bounded_strict");
    if(!original||!original->route_handler||!common_data_ptr_||!common_data_ptr_->is_data_available())
      throw std::logic_error("fresh_commit_missing_original_context");
    const auto route=original->route_handler;
    const auto uuid=route->getRouteUuid();
    const auto header=route->getRouteHeader();
    const auto map=route->getLaneletMapPtr();
    const auto input=fresh_commit::acquire();
    if(!input||!input->planner_data)throw std::logic_error("fresh_commit_provider_unavailable");
    const auto fresh=input->planner_data;
    if(!input->owner||!input->cycle||input->route_handler!=route.get()||
       input->map_identity!=map.get()||input->route_uuid!=uuid||input->route_stamp!=header.stamp||
       fresh->route_handler!=route||route->getRouteUuid()!=uuid||
       route->getRouteHeader().stamp!=header.stamp||route->getLaneletMapPtr()!=map)
      throw std::logic_error("fresh_commit_route_epoch_changed");
    if(!fresh->self_odometry||!fresh->self_acceleration||!fresh->dynamic_object)
      throw std::logic_error("fresh_commit_missing_motion_or_objects");
    const auto valid_stamp=[&](const ros::Time&stamp) {
      const double age=(input->acquired_at-stamp).toSec();
      return std::isfinite(age)&&age>=-.010&&age<=.5;
    };
    if(!valid_stamp(fresh->self_odometry->header.stamp)||
       !valid_stamp(fresh->self_acceleration->header.stamp)||
       !valid_stamp(fresh->dynamic_object->header.stamp))
      throw std::logic_error("fresh_commit_stale_inputs");
    if(fresh->self_odometry->header.frame_id!=fresh->dynamic_object->header.frame_id||
       !std::isfinite(fresh->self_odometry->twist.twist.linear.x+
          fresh->self_acceleration->accel.accel.linear.x))
      throw std::logic_error("fresh_commit_invalid_frame_or_motion");
    auto local=std::make_shared<lane_change::CommonData>(*common_data_ptr_);
    local->lanes_ptr=std::make_shared<lane_change::Lanes>(*common_data_ptr_->lanes_ptr);
    local->lanes_polygon_ptr=std::make_shared<lane_change::LanesPolygon>(*common_data_ptr_->lanes_polygon_ptr);
    local->bpp_param_ptr=std::make_shared<BehaviorPathPlannerParameters>(fresh->parameters);
    local->lc_param_ptr=std::make_shared<LaneChangeParameters>(*lane_change_parameters_);
    local->self_odometry_ptr=fresh->self_odometry;
    local->current_acceleration=fresh->self_acceleration;
    bool contained=false;
    for(const auto & lanes:{local->lanes_ptr->current,local->lanes_ptr->target}) {
      for(const auto&lane:lanes)if(utils::lane_change::contains_direction_compatible_pose(
          {lane},fresh->self_odometry->pose.pose,fresh->parameters.ego_nearest_yaw_threshold)) {
        local->lanes_ptr->ego_lane=lane;contained=true;break;
      }
      if(contained)break;
    }
    if(!contained)throw std::logic_error("fresh_commit_ego_outside_approved_lane_chain");
    // Publish nothing and mutate no shared PlannerData. Keep this module-private
    // epoch through postProcess; next normal setData replaces it next cycle.
    planner_data_=fresh;
    common_data_ptr_=std::move(local);
    refresh_transient_geometry(true);
    update_filtered_objects();
    if(route->getRouteUuid()!=uuid||route->getRouteHeader().stamp!=header.stamp||
       route->getLaneletMapPtr()!=map)throw std::logic_error("fresh_commit_route_changed_during_refresh");
    ROS_INFO_THROTTLE(1.0,"LC fresh commit context: odom_before=%.9f odom_after=%.9f objects=%.9f refresh_ms=%.3f",
      original->self_odometry?original->self_odometry->header.stamp.toSec():0.,
      fresh->self_odometry->header.stamp.toSec(),fresh->dynamic_object->header.stamp.toSec(),
      (ros::WallTime::now()-started).toSec()*1000.);
    return true;
  } catch(const std::exception & error) {
    // The existing strict unavailable-input path owns the failure decision.
    // Never silently fall back to old raw objects or invent a new stop policy.
    auto unavailable=original?std::make_shared<PlannerData>(*original):std::make_shared<PlannerData>();
    unavailable->dynamic_object.reset();
    planner_data_=std::move(unavailable);
    ROS_WARN_THROTTLE(1.0,"LC fresh commit unavailable: %s",error.what());
    return false;
  }
}

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  ++selected_output_cycle_;
  selected_output_model_.reset();
  selected_output_identity_.reset();
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!status_.is_valid_path) {
    RCLCPP_DEBUG(logger_, "No valid path found. Returning previous module's path as output.");
    insert_stop_point(get_current_lanes(), prev_module_output_.path);
    auto output=prev_module_output_;
    acquire_fresh_output_context();
    enforce_strict_maneuver_safety(output);
    return output;
  }

  auto output = prev_module_output_;
  bool rejected_shortened_lc_output = false;
  if (isAbortState() && abort_path_) {
    output.path = abort_path_->path;
    insert_stop_point(get_current_lanes(), output.path);
  } else {
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

  extendOutputDrivableArea(output);

  bool refresh_enabled = false;
  ros::param::getCached("/planning/approved_lc_speed_refresh", refresh_enabled);
  if (refresh_enabled && !rejected_shortened_lc_output &&
      lane_change_parameters_->trajectory.min_prepare_duration <= 1e-6 &&
      !isEgoOnPreparePhase() && !isAbortState() && !isStopState()) {
    // Opt-in experiment: a rejected proposal cannot alter the approved path.
    try {
      auto original = status_.lane_change_path;
      original.path = output.path;  // Already extended, stopped and drivable-area processed.
      const auto & points = original.path.points;
      const auto & current_ref = common_data_ptr_->current_lanes_path;
      const auto & target_lanes = get_target_lanes();
      if (target_lanes.empty()) throw std::logic_error("missing target lanes");
      const auto target_ref = getRouteHandler()->getCenterLinePath(
        target_lanes, 0.0, lanelet::geometry::length3d(lanelet::LaneletSequence(target_lanes)));
      const auto & upstream = prev_module_output_.path;
      if (points.size() < 3 || current_ref.points.size() < 2 ||
          target_ref.points.size() < 2 || upstream.points.size() < 2) {
        throw std::logic_error("missing speed reference");
      }
      const size_t begin = planner_data_->findEgoIndex(points);
      const size_t lc_end = autoware::motion_utils::findNearestIndex(
        points, original.info.lane_changing_end.position);
      const size_t refresh_end = points.size() - 1;
      if (begin >= lc_end || lc_end >= refresh_end || autoware_utils::calc_distance2d(
            points[lc_end].point.pose.position, original.info.lane_changing_end.position) > 1.0) {
        throw std::logic_error("no remaining LC interval");
      }
      const auto & params = lane_change_parameters_->trajectory;
      const auto & bpp = getCommonParam();
      if (params.lat_acc_map.base_max_acc.empty()) {
        throw std::logic_error("missing lateral acceleration map");
      }
      const double lateral_limit = *std::min_element(
        params.lat_acc_map.base_max_acc.begin(), params.lat_acc_map.base_max_acc.end());
      if (!std::isfinite(lateral_limit) || lateral_limit <= 0.0) {
        throw std::logic_error("invalid lateral acceleration limit");
      }
      const auto certificate=[&](){return currentExitCertificate(original.path,get_current_lanes(),get_target_lanes(),
        bpp.vehicle_info,begin,lc_end,getModuleType()==LaneChangeModuleType::NORMAL);};
      const auto caps=certifiedEnvelope(certificate,
        original.path,current_ref,target_ref,upstream,begin,lc_end,getEgoPosition(),
        bpp.max_vel,lateral_limit,autoware::motion_utils::findNearestIndex(
          original.path.points,original.info.lane_changing_start.position),true,nullptr,0,nullptr);
      auto proposal = original;
      proposal.path = utils::lane_change::refresh_path_speed(
        original.path, begin, refresh_end, caps, getEgoVelocity(),
        std::min(bpp.max_acc, params.max_longitudinal_acc),
        -std::max(bpp.min_acc, params.min_longitudinal_acc), common_data_ptr_->acceleration_limits);
      proposal.type = lane_change::PathType::SpatialSpeedProfile;
      // RSS metadata still ends at the physical LC endpoint, NOT the refreshed
      // normal-following suffix. The latter is constrained downstream by MVP.
      const auto metrics = utils::lane_change::refresh_lc_metrics(proposal.path, begin, lc_end);
      // Actual-motion extrapolation also needs the new horizon and speed cap;
      // retaining old frozen-profile metadata would silently truncate its check.
      proposal.info.duration.lane_changing = std::max(
        proposal.info.duration.lane_changing, metrics.duration);
      proposal.info.velocity.prepare = metrics.maximum_velocity;
      proposal.info.velocity.lane_changing = proposal.info.velocity.prepare;
      proposal.info.terminal_lane_changing_velocity = metrics.terminal_velocity;
      for (auto & point : proposal.shifted_path.path.points) {
        const size_t idx = autoware::motion_utils::findNearestIndex(
          proposal.path.points, point.point.pose.position);
        if (autoware_utils::calc_distance2d(point, proposal.path.points[idx]) < 1e-4) {
          point.point.longitudinal_velocity_mps =
            proposal.path.points[idx].point.longitudinal_velocity_mps;
        }
      }
      const auto predictions = utils::lane_change::convert_to_predicted_paths(
        common_data_ptr_, proposal, 1, true);
      if (!utils::lane_change::complete_refresh_predictions(predictions)) {
        throw std::logic_error("missing planned/actual prediction");
      }
      CollisionCheckDebugMap debug;
      const auto objects = get_target_objects(filtered_objects_, get_current_lanes());
      if (utils::lane_change::has_overtaking_turn_lane_object(
            common_data_ptr_, filtered_objects_.target_lane_trailing) ||
          utils::lane_change::is_delay_lane_change(
            common_data_ptr_, proposal, filtered_objects_.target_lane_leading.stopped, debug)) {
        throw std::logic_error("existing approved-path policy rejected");
      }
      const auto safety_check = [&](const LaneChangePath & final_proposal) {
        return isLaneChangePathSafe(final_proposal, predictions, objects,
            lane_change_parameters_->safety.rss_params, debug, true).is_safe &&
          isLaneChangePathSafe(final_proposal, predictions, objects,
            lane_change_parameters_->safety.rss_params_for_abort, debug, true).is_safe;
      };
      if (!utils::lane_change::commit_final_speed_refresh(
            status_.lane_change_path, output.path, std::move(proposal),
            rejected_shortened_lc_output, safety_check)) {
        throw std::logic_error("planned/actual RSS rejected");
      }
      ROS_INFO_THROTTLE(5.0, "Approved LC speed refresh accepted (geometry unchanged)");
    } catch (const std::exception & e) {
      ROS_INFO_THROTTLE(5.0, "Approved LC speed refresh retained original: %s", e.what());
    }
  }

  acquire_fresh_output_context();
  enforce_strict_maneuver_safety(output);

  // A validated current-lane cancellation owns its upstream turn signal and
  // drivable area. Do not overwrite them with the rejected LC metadata.
  if (isCancelState()) return output;

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
  transient_is_approved_=true;
  ++selected_output_cycle_;
  selected_output_model_.reset();
  selected_output_identity_.reset();
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

  // Optional early mandatory return: preserve a proved *following* chain from
  // the legal neighbor available NOW to the unchanged native future target.
  // This selects geometry only. Request, regulatory, candidate, RTC and full
  // safety gates below remain authoritative; never graft global preferences.
  const auto anticipated = [&]() -> lanelet::ConstLanelets {
    namespace fc = utils::lane_change::future_chain;
    fc::Gate gate;
    bool strict=false, bounded=false, grid=false;
    ros::param::getCached("/planning/lc_future_mandatory_anticipation", gate.enabled);
    if (!gate.enabled) return {};
    ros::param::getCached("/planning/lc_strict_maneuver_safety", strict);
    ros::param::getCached("/planning/lc_continuous_bounded_output", bounded);
    ros::param::getCached("/planning/lc_partial_acceleration_candidates", grid);
    gate.enabled=gate.enabled && strict && bounded && grid;
    gate.normal=getModuleType()==LaneChangeModuleType::NORMAL;
    gate.approved=transient_is_approved_;
    gate.aborting=isAbortState(); gate.stopping=isStopState();
    if (!gate.enabled || !gate.normal || gate.approved || gate.aborting || gate.stopping ||
        (getDirection()!=Direction::RIGHT && getDirection()!=Direction::LEFT)) return {};
    try {
      if (!route_handler || !route_handler->isHandlerReady() || !planner_data_ ||
          !planner_data_->self_odometry || !planner_data_->dynamic_object ||
          !common_data_ptr_->current_acceleration) return {};
      const auto graph=route_handler->getRoutingGraphPtr();
      const auto map=route_handler->getLaneletMapPtr();
      if (!graph || !map) return {};
      const auto route_uuid=route_handler->getRouteUuid();
      const auto route_stamp=route_handler->getRouteHeader().stamp;
      // RouteHandler intentionally keeps its route message private. Observe the
      // same latched route and require exact handler identity before using its
      // section ordering. This caches input only, never a graph/safety verdict.
      struct RouteInput {
        std::mutex mutex;
        autoware_planning_msgs::LaneletRoute::ConstPtr message;
        ros::Subscriber subscriber;
        RouteInput() {
          ros::NodeHandle nh;
          subscriber=nh.subscribe<autoware_planning_msgs::LaneletRoute>(
            "/planning/mission_planning/route",1,
            [this](const autoware_planning_msgs::LaneletRoute::ConstPtr & incoming) {
              std::lock_guard<std::mutex> lock(mutex); message=incoming;
            });
        }
      };
      static RouteInput route_input;
      autoware_planning_msgs::LaneletRoute::ConstPtr route_message;
      {
        std::lock_guard<std::mutex> lock(route_input.mutex);
        route_message=route_input.message;
      }
      if (!fc::same_route_epoch(route_message,route_uuid,route_stamp) ||
          route_message->segments.empty()) return {};
      const auto odom=planner_data_->self_odometry;
      const auto objects=planner_data_->dynamic_object;
      const auto accel=common_data_ptr_->current_acceleration;
      const double now=ros::Time::now().toSec(), ot=odom->header.stamp.toSec();
      const double dt=ot-objects->header.stamp.toSec(), at=accel->header.stamp.toSec();
      gate.fresh_route=ot>0 && objects->header.stamp.toSec()>0 && at>0 &&
        std::isfinite(now) && now-ot>=-.01 && now-ot<=.5 && dt>=-.01 && dt<=.5 &&
        now-at>=-.01 && now-at<=.5 && odom->header.frame_id==objects->header.frame_id;
      if (!gate.fresh_route) return {};
      lanelet::ConstLanelets containing;
      for (const auto & lane:current_lanes) {
        if (!route_handler->isRouteLanelet(lane)) return {};
        if (utils::lane_change::contains_direction_compatible_pose(
              {lane},odom->pose.pose,getCommonParam().ego_nearest_yaw_threshold))
          containing.push_back(lane);
      }
      // Ambiguous overlapping route polygons must not pick an arbitrary side.
      if (containing.size()!=1) return {};
      const auto & ego_lane=containing.front();
      const auto neighbor=getDirection()==Direction::RIGHT ? graph->right(ego_lane) : graph->left(ego_lane);
      if (!neighbor || !route_handler->isRouteLanelet(*neighbor)) return {};
      gate.legal_adjacent=true; gate.same_direction=true; // Vehicle graph legal edge, not adjacentLeft/Right.
      const auto preferred=route_handler->getPreferredLanelets();
      gate.future_mandatory=std::any_of(preferred.begin(),preferred.end(),[&](const auto & lane) {
        return lane.id()==lane_change_lane->id();
      }) && route_handler->getNumLaneToPreferredLane(ego_lane,getDirection())==0;
      const auto target_bound=getDirection()==Direction::RIGHT ?
        lane_change_lane->leftBound() : lane_change_lane->rightBound();
      if (target_bound.empty()) return {};
      auto future_start=odom->pose.pose;
      future_start.position.x=target_bound.front().x();
      future_start.position.y=target_bound.front().y();
      future_start.position.z=target_bound.front().z();
      gate.target_distance=utils::getSignedDistance(odom->pose.pose,future_start,current_lanes);
      // update_lanes runs before transient refresh. Never use prior-cycle
      // terminal distances/buffers/ego lane to decide this route anticipation.
      auto fresh=std::make_shared<lane_change::CommonData>(*common_data_ptr_);
      fresh->self_odometry_ptr=odom;
      fresh->lanes_ptr=std::make_shared<lane_change::Lanes>();
      fresh->lanes_ptr->current=current_lanes;
      fresh->lanes_ptr->ego_lane=ego_lane;
      if (prev_module_output_.path.points.size()<2) return {};
      fresh->transient_data=lane_change::TransientData{};
      const auto nearest=autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_module_output_.path.points,odom->pose.pose,getCommonParam().ego_nearest_dist_threshold,
        getCommonParam().ego_nearest_yaw_threshold);
      fresh->transient_data.current_path_velocity=
        prev_module_output_.path.points.at(nearest).point.longitudinal_velocity_mps;
      if (!std::isfinite(fresh->transient_data.current_path_velocity) ||
          fresh->transient_data.current_path_velocity<0) return {};
      const auto lengths=calculation::calc_lc_length_and_dist_buffer(fresh,current_lanes);
      if (!lengths) return {};
      gate.required_lc=lengths->first.max;
      const double speed=odom->twist.twist.linear.x;
      const double acceleration=accel->accel.accel.linear.x;
      const auto & rss=lane_change_parameters_->safety.rss_params;
      const double reaction=rss.rear_vehicle_reaction_time+rss.rear_vehicle_safety_time_margin;
      double validator=0, smoother=0, jerk=0;
      if (!ros::param::getCached("/planning_validator/trajectory_checker/forward_trajectory_length/acceleration",validator) ||
          !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother) ||
          !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jerk)) return {};
      const double lc_acc=lane_change_parameters_->trajectory.min_longitudinal_acc;
      for (double value:{speed,acceleration,reaction,validator,smoother,jerk,lc_acc})
        if (!std::isfinite(value)) return {};
      if (speed<0 || reaction<0 || validator>=0 || smoother>=0 || jerk>=0 || lc_acc>=0) return {};
      const double stop=passing_intent::stop_distance(
        speed,acceleration,std::min({-validator,-smoother,-lc_acc}),-jerk);
      gate.return_reserve=std::max(150.0,std::max(lengths->second.max,stop)+speed*reaction+
        .5*std::max(0.0,acceleration)*reaction*reaction+getCommonParam().vehicle_info.max_longitudinal_offset_m);
      const double prepare=lane_change_parameters_->trajectory.max_prepare_duration;
      if (!std::isfinite(prepare) || prepare<0) return {};
      gate.motion_ahead=speed*prepare+.5*std::max(0.0,acceleration)*prepare*prepare;
      if (!fc::applicable(gate)) return {};

      std::map<fc::Id,fc::Node> topology;
      int order=0;
      for (const auto & section:route_message->segments) {
        for (const auto & primitive:section.primitives) {
        if (topology.size()>=1024) return {}; // bounded per-cycle proof
        const auto lane=route_handler->getLaneletsFromId(primitive.id);
        if (!route_handler->isRouteLanelet(lane)) return {};
        if (topology.count(lane.id())) return {}; // repeated route primitive/loop is ambiguous
        fc::Node node;
        node.route_order=order;
        const auto center=lane.centerline();
        for (size_t i=1;i<center.size();++i)
          node.length+=std::hypot(center[i].x()-center[i-1].x(),center[i].y()-center[i-1].y());
        node.route=true;
        node.forward=!route_handler->isNoDrivableLane(lane);
        for (const auto & next:graph->following(lane))
          if (route_handler->isRouteLanelet(next)) node.following.push_back(next.id());
        topology.emplace(lane.id(),std::move(node));
        }
        ++order;
      }
      // Do not drift back onto the current branch before the future target.
      for (const auto & lane:current_lanes)
        if (lane.id()!=lane_change_lane->id()) topology.at(lane.id()).forward=false;
      const auto proof=fc::prove(topology,neighbor->id(),lane_change_lane->id());
      if (!proof.valid) return {};
      lanelet::ConstLanelets selected;
      std::set<lanelet::Id> seen;
      for (const auto id:proof.chain) {
        selected.push_back(route_handler->getLaneletsFromId(id)); seen.insert(id);
      }
      const auto suffix=route_handler->getLaneletSequence(
        *lane_change_lane,getEgoPose(),0.0,planner_data_->parameters.forward_path_length);
      if (suffix.empty() || suffix.front().id()!=lane_change_lane->id()) return {};
      for (size_t i=1;i<suffix.size();++i) {
        if (!seen.insert(suffix[i].id()).second || !route_handler->isRouteLanelet(suffix[i])) return {};
        const auto next=graph->following(selected.back());
        if (std::none_of(next.begin(),next.end(),[&](const auto & lane){return lane.id()==suffix[i].id();})) return {};
        selected.push_back(suffix[i]);
      }
      if (route_handler->getRouteUuid()!=route_uuid || route_handler->getRouteHeader().stamp!=route_stamp ||
          route_handler->getLaneletMapPtr()!=map || route_handler->getRoutingGraphPtr()!=graph) return {};
      {
        std::lock_guard<std::mutex> lock(route_input.mutex);
        if (!fc::same_route_generation(route_message,route_input.message)) return {};
      }
      ROS_INFO_THROTTLE(1.0,"LC future mandatory anticipation: ego=%ld legal_neighbor=%ld native_future=%ld distance=%.3f reserve=%.3f chain=%zu; candidate checks unchanged",
        ego_lane.id(),neighbor->id(),lane_change_lane->id(),gate.target_distance,gate.return_reserve,proof.chain.size());
      return selected;
    } catch (const std::exception & error) {
      ROS_DEBUG_THROTTLE(1.0,"LC future mandatory native fallback: %s",error.what());
      return {};
    }
  }();
  if (!anticipated.empty()) return anticipated;

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

  if (common_data_ptr_->transient_data.in_turn_direction_lane) {
    return true;
  }

  const auto nearest_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const double ego_velocity =
    std::max(getEgoVelocity(), lane_change_parameters_->trajectory.min_lane_changing_velocity);
  const double estimated_travel_dist = ego_velocity * lane_change_parameters_->cancel.delta_time;

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > estimated_travel_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      auto is_ego_within_original_lane = utils::isEgoWithinOriginalLane(
        curr_lanes_poly, estimated_pose, planner_data_->parameters,
        lane_change_parameters_->cancel.overhang_tolerance);
      lane_change_debug_.is_able_to_return_to_current_lane = is_ego_within_original_lane;
      return is_ego_within_original_lane;
    }
  }

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
  ExtendedPredictedObjects leading_objects = filtered_objects.target_lane_leading.moving;
  auto insert_leading_objects = [&](const auto & objects) {
    ranges::actions::insert(leading_objects, leading_objects.end(), objects);
  };

  insert_leading_objects(filtered_objects.target_lane_leading.stopped);
  insert_leading_objects(filtered_objects.target_lane_leading.stopped_at_bound);
  const auto chk_obj_in_curr_lanes =
    lane_change_parameters_->safety.collision_check.check_current_lane;
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
      prep_metric.duration > 1e-6) {
    // Preparation can be speed-limited while the execution road permits
    // acceleration. Do not reuse the preparation's (often zero) acceleration
    // as the only execution sample.
    std::vector<LaneChangePhaseMetrics> metrics;
    std::vector<std::vector<LaneChangePhaseMetrics>> groups;
    std::vector<size_t> counts;
    for (const double acceleration : calculation::calc_lon_acceleration_samples(
           common_data_ptr_, max_path_velocity, 0.0)) {
      const auto samples = calculation::calc_shift_phase_metrics(
        common_data_ptr_, shift_length, prep_metric.velocity, max_path_velocity,
        acceleration, max_lane_changing_length);
      groups.push_back(samples);
      counts.push_back(samples.size());
    }
    for (const auto & [group, sample] : calculation::joint_sample_order(counts)) {
      metrics.push_back(groups[group][sample]);
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

  const auto prepare_phase_metrics = get_prepare_metrics();

  const auto sorted_lane_ids = utils::lane_change::get_sorted_lane_ids(common_data_ptr_);
  if (
    common_data_ptr_->lc_param_ptr->frenet.enable &&
    common_data_ptr_->transient_data.is_ego_near_current_terminal_start) {
    return get_path_using_frenet(
      prepare_phase_metrics, target_objects, sorted_lane_ids, candidate_paths);
  }

  return get_path_using_path_shifter(
    prepare_phase_metrics, target_objects, sorted_lane_ids, candidate_paths);
}

bool NormalLaneChange::get_path_using_frenet(
  const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
  const lane_change::TargetObjects & target_objects,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids,
  LaneChangePaths & candidate_paths) const
{
  stop_watch_.tic(__func__);
  constexpr auto found_safe_path = true;
  const auto admission_deadline=ros::WallTime::now()+ros::WallDuration(lane_change_parameters_->time_limit*.001);
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
    if (stop_watch_.toc(__func__) >= lane_change_parameters_->time_limit) break;
    const auto frenet_candidates = utils::lane_change::generate_frenet_candidates(
      common_data_ptr_, prev_module_output_.path, batch);
    generated_count += frenet_candidates.size();
    RCLCPP_DEBUG(
      logger_, "Generated %lu candidate paths in %2.2f[ms]", frenet_candidates.size(),
      stop_watch_.toc(__func__));

    candidate_paths.reserve(frenet_candidates.size());
    lane_change_debug_.frenet_states.reserve(generated_count);
    for (const auto & frenet_candidate : frenet_candidates) {
      if (stop_watch_.toc(__func__) >= lane_change_parameters_->time_limit) {
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

      try {
        bool strict_final=false,continuous_final=false,reachable_final=false;
        ros::param::getCached("/planning/lc_strict_maneuver_safety",strict_final);
        ros::param::getCached("/planning/lc_continuous_bounded_output",continuous_final);
        ros::param::getCached("/planning/lc_reachable_admission",reachable_final);
        const bool final_checks=strict_final&&continuous_final&&reachable_final;
        bool native_safe=false;
        const bool accepted=lane_change::reachable_admission::finalize_and_check(
          final_checks,*candidate_path_opt,
          [&](auto&path){utils::lane_change::append_target_ref_to_candidate(
            path,common_data_ptr_->lc_param_ptr->frenet.th_curvature_smoothing);},
          [&](const auto&path){native_safe=check_candidate_path_safety(path,target_objects);return native_safe;},
          [&](const auto&path){return check_reachable_candidate_admission(path,admission_deadline);});
        // Preserve the old admission-failure path: do not retain this rejected
        // candidate as the native fallback/debug path below.
        if(native_safe&&!accepted)continue;
        if (accepted) {
          RCLCPP_DEBUG(
            logger_, "Found safe path after %lu candidate(s). Total time: %2.2f[ms]",
            frenet_candidates.size(), stop_watch_.toc(__func__));
          candidate_paths.push_back(*candidate_path_opt);
          return found_safe_path;
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
  return !found_safe_path;
}

bool NormalLaneChange::get_path_using_path_shifter(
  const std::vector<LaneChangePhaseMetrics> & prepare_metrics,
  const lane_change::TargetObjects & target_objects,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids,
  LaneChangePaths & candidate_paths) const
{
  stop_watch_.tic(__func__);
  const auto & target_lanes = get_target_lanes();
  candidate_paths.reserve(
    prepare_metrics.size() * lane_change_parameters_->trajectory.lat_acc_sampling_num);
  const auto admission_deadline=ros::WallTime::now()+ros::WallDuration(lane_change_parameters_->time_limit*.001);

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

  struct SearchGroup {
    LaneChangePhaseMetrics prep;
    PathWithLaneId segment;
    double shift;
    std::vector<LaneChangePhaseMetrics> metrics;
    size_t debug_index;
  };
  const bool competition = lane_change_parameters_->trajectory.min_prepare_duration <= 1e-6;
  bool diverse_search = false;
  ros::param::getCached("/planning/lc_diverse_candidate_search", diverse_search);
  diverse_search = diverse_search && competition;
  lane_change::SafePrepareCandidateSelection best_safe_prepare;
  bool envelope_preference=false,score_strict=false,score_continuous=false,score_partial=false;
  ros::param::getCached("/planning/lc_full_envelope_preference",envelope_preference);
  ros::param::getCached("/planning/lc_strict_maneuver_safety",score_strict);
  ros::param::getCached("/planning/lc_continuous_bounded_output",score_continuous);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",score_partial);
  envelope_preference=envelope_preference&&diverse_search&&score_strict&&score_continuous&&
    score_partial&&getModuleType()==LaneChangeModuleType::NORMAL;
  lane_change::full_envelope::Selection best_envelope;
  std::optional<decltype(lane_change_debug_.collision_check_objects)> best_envelope_debug;
  size_t envelope_attempts=0,envelope_covered=0,envelope_feasible=0;double envelope_ms=0;
  bool audit_enabled=false;ros::param::getCached("/planning/lc_candidate_reason_audit",audit_enabled);
  struct AuditRow {size_t ordinal;std::optional<size_t> index;double prepare,lat,length;int type;std::string reason;
    double prepare_acc=std::numeric_limits<double>::quiet_NaN(),changing_acc=std::numeric_limits<double>::quiet_NaN();
    bool world_constructor=false;};
  size_t audit_ordinal=0;
  std::vector<AuditRow> audit_rows;
  std::optional<size_t> audit_safe_index,audit_envelope_index;
  const auto audit_cycle=ros::WallTime::now().toNSec();
  std::optional<decltype(lane_change_debug_.collision_check_objects)> best_safe_debug;
  std::optional<double> first_safe_loss;
  double selected_safe_loss=0.0;
  double first_safe_progress=-1.0,selected_safe_progress=-1.0;
  double first_safe_prepare=-1.0,selected_safe_prepare=-1.0;
  const auto finish_best_safe = [&]() {
    const bool selected_envelope=envelope_preference&&best_envelope.append_selected(candidate_paths);
    const bool any_selected=selected_envelope||best_safe_prepare.append_selected(candidate_paths);
    if (selected_envelope&&best_envelope_debug) lane_change_debug_.collision_check_objects=*best_envelope_debug;
    else if (best_safe_debug) lane_change_debug_.collision_check_objects = *best_safe_debug;
    if(audit_enabled) {
      static ros::WallTime last_audit;
      const auto now=ros::WallTime::now();
      if((now-last_audit).toSec()>=1.0) {
        last_audit=now;
        const auto selected=selected_envelope?audit_envelope_index:audit_safe_index;
        const double selected_lat=any_selected?candidate_paths.back().info.lateral_acceleration:std::numeric_limits<double>::infinity();
        std::ostringstream audit;
        for(const auto&r:audit_rows)if((selected&&r.index&&*r.index==*selected)||r.lat<selected_lat)
          audit<<" {ordinal="<<r.ordinal<<" candidate="<<(r.index?static_cast<long long>(*r.index):-1LL)<<" prep="<<r.prepare<<" lat="<<r.lat<<" length="<<r.length
            <<" type="<<r.type<<" prepare_acc="<<r.prepare_acc<<" changing_acc="<<r.changing_acc
            <<" world_constructor="<<r.world_constructor<<" reason="<<r.reason<<"}";
        ROS_INFO("LC paired candidate reasons: cycle=%llu selected=%lld envelope=%d odom=%.9f objects=%.9f ego_v=%.6f rows=%s",
          static_cast<unsigned long long>(audit_cycle),selected?static_cast<long long>(*selected):-1LL,
          selected_envelope,planner_data_->self_odometry->header.stamp.toSec(),
          planner_data_->dynamic_object?planner_data_->dynamic_object->header.stamp.toSec():0.,getEgoVelocity(),
          audit.str().c_str());
        ROS_INFO("LC paired candidate capture_ms=%.3f",(ros::WallTime::now()-now).toSec()*1000.);
      }
    }
    if(!any_selected)return false;
    if(envelope_preference)ROS_INFO_THROTTLE(1.0,
      "LC full envelope preference: feasible=%zu covered=%zu attempted=%zu selected=%d score_distance=150 score_time=%.3f score_ms=%.3f",
      envelope_feasible,envelope_covered,envelope_attempts,selected_envelope,best_envelope.time(),envelope_ms);
    ROS_INFO_THROTTLE(1.0,"LC preparation selection: first_loss=%.3f selected_loss=%.3f first_progress=%.3f selected_progress=%.3f first_prepare=%.3f selected_prepare=%.3f candidates=%zu elapsed_ms=%.3f",
      first_safe_loss.value_or(0.0),selected_safe_loss,first_safe_progress,selected_safe_progress,
      first_safe_prepare,selected_safe_prepare,candidate_paths.size(),stop_watch_.toc("get_path_using_path_shifter"));
    return true;
  };
  std::vector<size_t> preparation_order;
  if (diverse_search) preparation_order = calculation::diverse_phase_order(prepare_metrics, true);
  std::vector<std::optional<SearchGroup>> groups(prepare_metrics.size());
  calculation::JointSampleQueue pending(prepare_metrics.size(), competition);
  while (!pending.empty()) {
    if (stop_watch_.toc(__func__) >= lane_change_parameters_->time_limit) return finish_best_safe();
    const auto [group_index, metric_index] = pending.pop();
    const auto & prep_metric = prepare_metrics[
      diverse_search ? preparation_order[group_index] : group_index];
    if (!groups[group_index]) {
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
    auto lane_changing_metrics = get_lane_changing_metrics(
      prepare_segment, prep_metric, shift_length, dist_to_next_regulatory_element, debug_metrics);
    if (diverse_search) {
      const auto metric_order = calculation::diverse_phase_order(
        lane_changing_metrics, false, group_index);
      auto ordered_metrics = lane_changing_metrics;
      for (size_t i = 0; i < metric_order.size(); ++i) {
        ordered_metrics[i] = lane_changing_metrics[metric_order[i]];
      }
      lane_changing_metrics = std::move(ordered_metrics);
    }

    // Keep preparation speeds independent from the execution speed samples.
    utils::lane_change::set_prepare_velocity(
      prepare_segment, common_data_ptr_->get_ego_speed(), prep_metric.velocity);

    groups[group_index] = SearchGroup{prep_metric, std::move(prepare_segment), shift_length,
      lane_changing_metrics, lane_change_debug_.lane_change_metrics.size() - 1};
    }
      const auto & group = *groups[group_index];
      if (metric_index >= group.metrics.size()) continue;
      // Initialize only the group about to be checked. In particular a safe
      // first candidate is checked before spending budget preparing other groups.
      // Competition uses breadth; ordinary mode retains streaming depth-first.
      if (metric_index + 1 < group.metrics.size()) {
        pending.refine(group_index, metric_index + 1);
      }
      const auto & prepare_segment = group.segment;
      const auto shift_length = group.shift;
      const auto & lc_metric = group.metrics[metric_index];
      auto & debug_metrics = lane_change_debug_.lane_change_metrics[group.debug_index];
      if (stop_watch_.toc(__func__) >= lane_change_parameters_->time_limit) {
        RCLCPP_DEBUG(logger_, "Time limit reached and no safe path was found.");
        return finish_best_safe();
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

      AuditRow * audit_row=nullptr;
      if(audit_enabled&&audit_rows.size()<128) {
        audit_rows.push_back({audit_ordinal,std::nullopt,prep_metric.duration,
          lc_metric.lat_accel,lc_metric.length,-1,"generation_not_completed"});
        audit_row=&audit_rows.back();
        audit_row->prepare_acc=prep_metric.actual_lon_accel;
        audit_row->changing_acc=lc_metric.actual_lon_accel;
      }
      ++audit_ordinal;
      LaneChangePath candidate_path;
      try {
        bool physical_endpoint=false,endpoint_strict=false,endpoint_continuous=false,endpoint_partial=false;
        ros::param::getCached("/planning/lc_physical_endpoint_consistency",physical_endpoint);
        ros::param::getCached("/planning/lc_strict_maneuver_safety",endpoint_strict);
        ros::param::getCached("/planning/lc_continuous_bounded_output",endpoint_continuous);
        ros::param::getCached("/planning/lc_partial_acceleration_candidates",endpoint_partial);
        physical_endpoint=physical_endpoint&&endpoint_strict&&endpoint_continuous&&endpoint_partial&&
          getModuleType()==LaneChangeModuleType::NORMAL;
        try {
          candidate_path = utils::lane_change::get_candidate_path(
            common_data_ptr_, prep_metric, lc_metric, prepare_segment, sorted_lane_ids, shift_length,
            physical_endpoint, &prev_module_output_.path);
        } catch (const std::logic_error &) {
          // An additional constructor, NOT an alternative safety/approval path.
          // Native candidates retain priority and default-OFF behavior exactly.
          namespace lc = utils::lane_change;
          bool requested=false,reachable=false;
          ros::param::getCached("/planning/lc_world_candidate",requested);
          ros::param::getCached("/planning/lc_reachable_admission",reachable);
          if (!lc::world_candidate::enabled(requested,
                getModuleType()==LaneChangeModuleType::NORMAL,endpoint_strict,
                endpoint_continuous,endpoint_partial,reachable,physical_endpoint) ||
              ros::WallTime::now()>=admission_deadline) throw;
          if (!planner_data_->self_odometry || !planner_data_->dynamic_object ||
              !common_data_ptr_->current_acceleration) throw;
          const double age=(planner_data_->self_odometry->header.stamp-
                            planner_data_->dynamic_object->header.stamp).toSec();
          if (!std::isfinite(age)||age<-.010||age>.5) throw;
          double smoother_acc=0,jmin=0,jmax=0;
          if (!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother_acc)||
              !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jmin)||
              !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/max_jerk",jmax)) throw;
          const auto world=prepare_admission_world();
          const double horizon=world->horizon+std::max(0.,-age);
          const double amin=std::max({getCommonParam().min_acc,
            lane_change_parameters_->trajectory.min_longitudinal_acc,smoother_acc});
          bool rising=false,invariant=false;
          ros::param::getCached("/planning/interpolate_rising_velocity",rising);
          ros::param::getCached("/planning/lc_anchor_invariant_curvature",invariant);
          lc::world_candidate::Services services;
          services.native_reference=[&](const lc::world_candidate::Input & input){
            return lc::world_candidate_native_reference(input.common,input.prepare_metric,
              input.changing_metric,input.prepare,physical_endpoint);
          };
          services.anchor=[&](const PathWithLaneId & path){
            return lc::anchor_maneuver_at_ego(path,
              planner_data_->findEgoSegmentIndex(path.points),getEgoPosition(),rising);
          };
          services.hard_caps=[&](const PathWithLaneId & path,size_t begin,size_t start,
                                 size_t end,const PathWithLaneId & before){
            const auto & lanes=get_target_lanes();
            if(lanes.empty())throw std::logic_error("world missing target lanes");
            const auto target=getRouteHandler()->getCenterLinePath(lanes,0.,
              lanelet::geometry::length3d(lanelet::LaneletSequence(lanes)));
            const auto & lat=lane_change_parameters_->trajectory.lat_acc_map.base_max_acc;
            if(lat.empty())throw std::logic_error("world missing lateral limits");
            const auto certificate=[&](){return currentExitCertificate(path,get_current_lanes(),
              lanes,getCommonParam().vehicle_info,begin,end,true);};
            return certifiedEnvelope(certificate,path,common_data_ptr_->current_lanes_path,
              target,prev_module_output_.path,begin,end,getEgoPosition(),getCommonParam().max_vel,
              *std::min_element(lat.begin(),lat.end()),start,true,
              invariant?&before:nullptr,begin,nullptr);
          };
          candidate_path=lc::world_candidate::make({common_data_ptr_,prep_metric,lc_metric,
            prepare_segment,sorted_lane_ids,shift_length,admission_deadline,horizon,amin,jmin,jmax},
            services).candidate;
          if(audit_row)audit_row->world_constructor=true;
          // Even a constructed path must still fit the ORIGINAL search budget.
          if(ros::WallTime::now()>=admission_deadline)
            throw std::logic_error("world candidate search deadline exhausted");
          ROS_DEBUG_THROTTLE(1.,"LC world proposal constructed; native/strict/reachable checks pending");
        }
      } catch (const utils::lane_change::GenerationRejection & e) {
        if(audit_row)audit_row->reason=std::string("generation_exception:")+e.what();
        debug_print_lat(std::string("Reject: ") + e.what());
        utils::lane_change::generation_diagnostic_nonfatal([&] {
          namespace lc=utils::lane_change;
          boost::property_tree::ptree packet;
          packet.put("schema","lc_generation_rejection_v1");
          packet.put("stage","generation_rejected");packet.put("completed",false);
          packet.put("module",getModuleTypeStr());packet.put("ordinal",audit_ordinal-1);
          packet.put("direction",getDirection()==Direction::LEFT?"left":
            (getDirection()==Direction::RIGHT?"right":"none"));
          packet.put("stamp",ros::Time::now().toSec());packet.put("reason",e.what());
          packet.put("yaw",e.yaw);packet.put("yaw_threshold",e.threshold);
          packet.add_child("prepare_segment",lc::snapshot_ros_message(e.prepare));
          packet.put("upstream_exact_available",e.upstream.has_value());
          if(e.upstream) {
            packet.add_child("upstream",lc::snapshot_ros_message(*e.upstream));
            packet.put("upstream_provenance","generation_prev_module_output_path");
          }
          packet.add_child("shifted_path",lc::snapshot_ros_message(e.shifted.path));
          packet.add_child("target_reference",lc::snapshot_ros_message(e.reference));
          boost::property_tree::ptree shifts;
          for(const auto value:e.shifted.shift_length){boost::property_tree::ptree item;item.put("",value);shifts.push_back({"",item});}
          packet.add_child("shift_length",shifts);
          const auto phase=[&](const char * name,const auto & value){
            packet.put(std::string("input_info.")+name+".prepare",value.prepare);
            packet.put(std::string("input_info.")+name+".changing",value.lane_changing);};
          phase("acceleration",e.info.longitudinal_acceleration);phase("velocity",e.info.velocity);
          phase("length",e.info.length);phase("duration",e.info.duration);
          packet.put("input_info.lateral_acceleration",e.info.lateral_acceleration);
          packet.put("input_info.terminal_velocity",e.info.terminal_lane_changing_velocity);
          packet.add_child("input_info.start",lc::snapshot_ros_message(e.info.lane_changing_start));
          packet.add_child("input_info.end",lc::snapshot_ros_message(e.info.lane_changing_end));
          packet.add_child("input_info.shift_start",lc::snapshot_ros_message(e.info.shift_line.start));
          packet.add_child("input_info.shift_end",lc::snapshot_ros_message(e.info.shift_line.end));
          packet.put("input_info.shift_start_idx",e.info.shift_line.start_idx);
          packet.put("input_info.shift_end_idx",e.info.shift_line.end_idx);
          packet.put("input_info.shift_start_length",e.info.shift_line.start_shift_length);
          packet.put("input_info.shift_end_length",e.info.shift_line.end_shift_length);
          if(planner_data_->self_odometry)packet.add_child("odometry",lc::snapshot_ros_message(*planner_data_->self_odometry));
          if(planner_data_->dynamic_object)packet.add_child("objects",lc::snapshot_ros_message(*planner_data_->dynamic_object));
          if(common_data_ptr_->current_acceleration)packet.add_child("acceleration",lc::snapshot_ros_message(*common_data_ptr_->current_acceleration));
          packet.add_child("route_header",lc::snapshot_ros_message(getRouteHeader()));
          packet.add_child("route_uuid",lc::snapshot_ros_message(getRouteHandler()->getRouteUuid()));
          const auto lane_ids=[&](const char * key,const auto & lanes){
            boost::property_tree::ptree ids;
            for(const auto & lane:lanes){boost::property_tree::ptree id;id.put("",lane.id());ids.push_back({"",id});}
            packet.add_child(key,ids);};
          lane_ids("current_lane_ids",get_current_lanes());lane_ids("target_lane_ids",get_target_lanes());
          std::ostringstream stream;boost::property_tree::write_json(stream,packet,false);
          std_msgs::String message;message.data=stream.str();strictSnapshotPublisher().publish(message);
        });
        continue;
      } catch (const std::exception & e) {
        if(audit_row)audit_row->reason=std::string("generation_exception:")+e.what();
        debug_print_lat(std::string("Reject: ") + e.what());
        continue;
      }

      candidate_paths.push_back(candidate_path);
      debug_metrics.lc_metrics.back().second = static_cast<int>(candidate_paths.size()) - 1;

      std::string * audit_reason=nullptr;
      if(audit_row) {
        audit_row->index=candidate_paths.size()-1;
        audit_row->prepare=candidate_path.info.duration.prepare;
        audit_row->lat=candidate_path.info.lateral_acceleration;
        audit_row->length=candidate_path.info.length.lane_changing;
        audit_row->type=static_cast<int>(candidate_path.type);
        audit_row->prepare_acc=candidate_path.info.longitudinal_acceleration.prepare;
        audit_row->changing_acc=candidate_path.info.longitudinal_acceleration.lane_changing;
        audit_row->reason="not_evaluated";
        audit_reason=&audit_row->reason;
      }
      try {
        CandidateReasonScope audit_scope(audit_reason);
        candidateReason("native_guard_unknown");
        if (check_candidate_path_safety(candidate_path, target_objects) &&
            check_reachable_candidate_admission(candidate_path,admission_deadline)) {
          debug_print_lat("ACCEPT!!!: it is valid and safe!");
          if (!diverse_search) return true;
          // Spatial-profile construction refreshes these metadata from the
          // actual published preparation endpoint, not the sampled acceleration.
          const double loss=lane_change::prepare_speed_loss(getEgoVelocity(),
            candidate_path.info.duration.prepare,candidate_path.info.velocity.prepare);
          double progress=-1.0;
          try {
            // Score the same spatial maneuver / reachable normal tail used by
            // strict safety, not raw road-speed ceilings after the LC endpoint.
            // Scoring failure must never discard an already safety-checked path.
            namespace lc=utils::lane_change;
            bool rising=false;
            ros::param::getCached("/planning/interpolate_rising_velocity",rising);
            const auto anchored=lc::anchor_maneuver_at_ego(candidate_path.path,
              planner_data_->findEgoSegmentIndex(candidate_path.path.points),getEgoPosition(),rising);
            const size_t begin=anchored.second;
            const size_t finish=std::max(begin,autoware::motion_utils::findNearestIndex(
              anchored.first.points,candidate_path.info.lane_changing_end.position));
            const size_t prepare=autoware::motion_utils::findNearestIndex(
              anchored.first.points,candidate_path.info.lane_changing_start.position);
            const auto modeled=lc::maneuver_prediction_profile(anchored.first,begin,finish,prepare,
              candidate_path.type==lane_change::PathType::SpatialSpeedProfile,getEgoVelocity(),
              candidate_path.info.longitudinal_acceleration.prepare,
              candidate_path.info.longitudinal_acceleration.lane_changing,
              std::min(getCommonParam().max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc),
              -std::max(getCommonParam().min_acc,lane_change_parameters_->trajectory.min_longitudinal_acc),
              common_data_ptr_->acceleration_limits);
            const auto full_arc=lc::safety_arc_lengths(modeled);
            std::vector<double> arc,speeds;
            for(size_t i=begin;i<full_arc.size();++i) {
              arc.push_back(full_arc[i]-full_arc[begin]);
              speeds.push_back(modeled.points[i].point.longitudinal_velocity_mps);
            }
            progress=lc::sample_speed_profile(arc,speeds,3.0).first;
          } catch(const std::exception &) { /* Safe fallback remains available. */ }
          if(!first_safe_loss) {
            first_safe_loss=loss;first_safe_progress=progress;
            first_safe_prepare=candidate_path.info.duration.prepare;
          }
          if (best_safe_prepare.consider(candidate_paths.size()-1,true,loss,progress,
                candidate_path.info.duration.prepare)) {
            audit_safe_index=candidate_paths.size()-1;
            selected_safe_loss=loss;
            selected_safe_progress=progress;
            selected_safe_prepare=candidate_path.info.duration.prepare;
            best_safe_debug=lane_change_debug_.collision_check_objects;
          }
          const double score_budget=lane_change::full_envelope::scoring_budget_ms(lane_change_parameters_->time_limit);
          if(envelope_preference&&envelope_ms<score_budget&&stop_watch_.toc(__func__)<lane_change_parameters_->time_limit) {
            const auto score_started=ros::WallTime::now();++envelope_attempts;
            try {
              namespace lc=utils::lane_change;
              bool rising=false,immutable=false;
              ros::param::getCached("/planning/interpolate_rising_velocity",rising);
              ros::param::getCached("/planning/lc_anchor_invariant_curvature",immutable);
              const auto anchored=lc::anchor_maneuver_at_ego(candidate_path.path,
                planner_data_->findEgoSegmentIndex(candidate_path.path.points),getEgoPosition(),rising);
              const auto full_arc=lc::safety_arc_lengths(anchored.first);
              if(full_arc.back()-full_arc[anchored.second]<150.0)
                throw std::logic_error("score common distance not covered");
              ++envelope_covered;
              const auto end=autoware::motion_utils::findNearestIndex(
                anchored.first.points,candidate_path.info.lane_changing_end.position);
              if(autoware_utils::calc_distance2d(anchored.first.points[end].point.pose,
                  candidate_path.info.lane_changing_end)>1.0)throw std::logic_error("score endpoint coverage");
              const auto &target=get_target_lanes();
              if(target.empty()||!common_data_ptr_->current_acceleration||!planner_data_->dynamic_object||
                 !common_data_ptr_->acceleration_limits.enabled())
                throw std::logic_error("score missing context");
              const auto target_reference=getRouteHandler()->getCenterLinePath(target,0.0,
                lanelet::geometry::length3d(lanelet::LaneletSequence(target)));
              const auto &lat=lane_change_parameters_->trajectory.lat_acc_map.base_max_acc;
              if(lat.empty())throw std::logic_error("score missing lateral limits");
              const auto certificate=[&](){return currentExitCertificate(anchored.first,get_current_lanes(),get_target_lanes(),
                getCommonParam().vehicle_info,anchored.second,end,getModuleType()==LaneChangeModuleType::NORMAL);};
              const auto caps=certifiedEnvelope(certificate,anchored.first,
                common_data_ptr_->current_lanes_path,target_reference,prev_module_output_.path,
                anchored.second,end,getEgoPosition(),getCommonParam().max_vel,
                *std::min_element(lat.begin(),lat.end()),autoware::motion_utils::findNearestIndex(
                  anchored.first.points,candidate_path.info.lane_changing_start.position),true,
                immutable?&candidate_path.path:nullptr,anchored.second,nullptr);
              std::vector<double> arc;
              for(size_t i=anchored.second;i<full_arc.size();++i)arc.push_back(full_arc[i]-full_arc[anchored.second]);
              double amin=0,jmin=0,jmax=0;
              if(!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",amin)||
                 !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jmin)||
                 !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/max_jerk",jmax))
                throw std::logic_error("score missing physical limits");
              amin=std::max({amin,getCommonParam().min_acc,lane_change_parameters_->trajectory.min_longitudinal_acc});
              const auto &raw=*planner_data_->dynamic_object;
              const double age=(planner_data_->self_odometry->header.stamp-raw.header.stamp).toSec();
              if(!std::isfinite(age)||age<-.010||age>.5)throw std::logic_error("score stale input");
              double horizon=3;
              for(const auto&object:raw.objects) {
                const auto observed=lc::observed_maneuver_object(object,lane_change_parameters_->safety.collision_check.prediction_time_resolution);
                if(observed.predicted_paths.empty())throw std::logic_error("score missing predictions");
                for(const auto&prediction:observed.predicted_paths) {
                  if(prediction.path.empty())throw std::logic_error("score empty prediction");
                  horizon=std::min(horizon,prediction.path.back().time-std::max(0.,age));
                }
              }
              if(horizon<2.5)throw std::logic_error("score insufficient horizon");
              horizon+=std::max(0.,-age);
              const auto limit=[&](double velocity){return common_data_ptr_->acceleration_limits.limit(velocity,
                std::min(getCommonParam().max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc));};
              bool score_positive=false,score_adaptive=false,score_continuous=false,score_partial=false;
              ros::param::getCached("/planning/lc_rss_positive_grid",score_positive);
              ros::param::getCached("/planning/lc_adaptive_positive_grid",score_adaptive);
              ros::param::getCached("/planning/lc_continuous_bounded_output",score_continuous);
              ros::param::getCached("/planning/lc_partial_acceleration_candidates",score_partial);
              const auto score_targets=lane_change::rss_positive_grid::augment(
                bounded_speed_grid::acceleration_targets(limit(getEgoVelocity()),amin),
                limit(getEgoVelocity()),score_positive&&score_adaptive&&score_continuous&&score_partial&&getModuleType()==LaneChangeModuleType::NORMAL);
              const auto score=lane_change::full_envelope::evaluate(arc,caps,getEgoVelocity(),
                common_data_ptr_->current_acceleration->accel.accel.linear.x,horizon,amin,jmin,jmax,
                150.0,limit,[&](){return envelope_ms+(ros::WallTime::now()-score_started).toSec()*1000<score_budget&&
                  stop_watch_.toc("get_path_using_path_shifter")<lane_change_parameters_->time_limit;},score_targets);
              if(score.feasible)++envelope_feasible;
              if(audit_reason)*audit_reason += score.feasible?";score_feasible":";score_infeasible_or_budget";
              if(best_envelope.consider(candidate_paths.size()-1,true,score)) {
                audit_envelope_index=candidate_paths.size()-1;
                best_envelope_debug=lane_change_debug_.collision_check_objects;
              }
            }catch(const std::exception&) { /* Score failure is not candidate rejection. */ }
            envelope_ms+=(ros::WallTime::now()-score_started).toSec()*1000;
          }
          // A zero-loss coast is not an upper bound on achievable progress.
          // Keep exploring within the existing budget, retaining the safe best.
          continue;
        }
      } catch (const std::exception & e) {
        debug_print_lat(std::string("Reject: ") + e.what());
        if(audit_reason)*audit_reason += std::string(";exception=")+e.what();
        // A footprint/boundary rejection belongs to this sample, not to the
        // entire search. Tighter lateral or different longitudinal samples can
        // still be feasible at the current speed (as in the Frenet branch).
        continue;
      }

      debug_print_lat("Reject: sampled path is not safe.");
  }

  if (finish_best_safe()) return true;
  RCLCPP_DEBUG(logger_, "No safety path found.");
  return false;
}

bool NormalLaneChange::check_candidate_path_safety(
  const LaneChangePath & candidate_path, const lane_change::TargetObjects & target_objects) const
{
  bool future_anticipation=false;
  ros::param::getCached("/planning/lc_future_mandatory_anticipation",future_anticipation);
  if(future_anticipation && getModuleType()==LaneChangeModuleType::NORMAL) {
    try {
      const auto&current=get_current_lanes();const auto&target=get_target_lanes();
      const auto native=utils::lane_change::get_lane_change_target_lane(common_data_ptr_,current);
      // Native candidates keep their exact old policy. Only a locally proved
      // prefix preceding the native future target needs this handoff gate.
      if(native && !target.empty() && target.front().id()!=native->id()) {
        bool strict=false,bounded=false,grid=false;
        ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
        ros::param::getCached("/planning/lc_continuous_bounded_output",bounded);
        ros::param::getCached("/planning/lc_partial_acceleration_candidates",grid);
        if(!strict||!bounded||!grid)return false;
        const auto handler=getRouteHandler();const auto graph=handler->getRoutingGraphPtr();
        if(!graph)return false;
        const auto route_uuid=handler->getRouteUuid();const auto route_stamp=handler->getRouteHeader().stamp;
        const auto map=handler->getLaneletMapPtr();
        lanelet::ConstLanelets containing;
        for(const auto&lane:current)
          if(utils::lane_change::contains_direction_compatible_pose(
               {lane},getEgoPose(),getCommonParam().ego_nearest_yaw_threshold))containing.push_back(lane);
        if(containing.size()!=1)return false;
        if(getDirection()!=Direction::RIGHT&&getDirection()!=Direction::LEFT)return false;
        const auto neighbor=getDirection()==Direction::RIGHT ? graph->right(containing.front()) :
          graph->left(containing.front());
        if(!neighbor||neighbor->id()!=target.front().id())return false;
        bool reached_future=false;std::set<lanelet::Id> seen;
        for(size_t i=0;i<target.size();++i) {
          if(!seen.insert(target[i].id()).second || !handler->isRouteLanelet(target[i]))return false;
          if(i) {
            const auto next=graph->following(target[i-1]);
            if(std::none_of(next.begin(),next.end(),[&](const auto&lane){return lane.id()==target[i].id();}))return false;
          }
          if(target[i].id()==native->id()){reached_future=true;break;}
        }
        if(!reached_future)return false;
        const auto admission=utils::lane_change::future_handoff::evaluate(
          candidate_path.path,candidate_path.info.shift_line.end,current,target,native->id(),*handler);
        if(!admission.allowed) {
          ROS_INFO_THROTTLE(1.0,"LC future handoff candidate rejected: %s end=(%.3f,%.3f)",
            admission.reason.c_str(),candidate_path.info.shift_line.end.position.x,
            candidate_path.info.shift_line.end.position.y);
          return false;
        }
        if(handler->getRouteUuid()!=route_uuid||handler->getRouteHeader().stamp!=route_stamp||
           handler->getLaneletMapPtr()!=map||handler->getRoutingGraphPtr()!=graph)return false;
      }
    } catch(const std::exception&error) {
      ROS_DEBUG_THROTTLE(1.0,"LC future handoff candidate unavailable: %s",error.what());
      return false;
    }
  }
  candidateReason("native_overtaking_or_delay_guard");
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
  candidateReason("native_target_footprint_guard");
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

  bool strict = false;
  ros::param::getCached("/planning/lc_strict_maneuver_safety", strict);
  if (strict) {
    candidateReason("strict_raw_exception");
    std::string reason;
    bool continuous=false,partial=false,reachable=false;
    ros::param::getCached("/planning/lc_continuous_bounded_output",continuous);
    ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial);
    ros::param::getCached("/planning/lc_reachable_admission",reachable);
    auto provisional=candidate_path;
    if(lane_change::reachable_admission::use_actual_start_provisional_phase(
         getModuleType()==LaneChangeModuleType::NORMAL,
         candidate_path.type==lane_change::PathType::SpatialSpeedProfile,
         strict,continuous,partial,reachable)) {
      // Screen unchanged geometry/ceilings using native phase prediction from
      // actual speed. NEVER publish this model or overwrite candidate metadata.
      // Both search callers still require the actual-v/a bounded admission
      // witness after all native checks, within the original shared deadline.
      provisional.type=lane_change::PathType::ConstantJerk;
    }
    if (!check_strict_maneuver_safety(provisional, candidate_path.path, reason, false)) {
      candidateReason(std::string("strict_raw_rejected:")+reason);
      ROS_INFO_THROTTLE(1.0, "Strict LC candidate rejected: %s", reason.c_str());
      return false;
    }
  }

  if ((target_objects.empty()) || candidate_path.path.points.empty()) {
    candidateReason("native_no_target_objects_pass");
    RCLCPP_DEBUG(logger_, "There is nothing to check.");
    return true;
  }

  constexpr size_t decel_sampling_num = 1;
  candidateReason("native_prediction_or_RSS_exception");
  const auto ego_predicted_paths = utils::lane_change::convert_to_predicted_paths(
    common_data_ptr_, candidate_path, decel_sampling_num);

  const auto safety_check_with_normal_rss = isLaneChangePathSafe(
    candidate_path, ego_predicted_paths, target_objects,
    common_data_ptr_->lc_param_ptr->safety.rss_params, lane_change_debug_.collision_check_objects);

  if (!safety_check_with_normal_rss.is_safe && is_stuck) {
    const auto safety_check_with_stuck_rss = isLaneChangePathSafe(
      candidate_path, ego_predicted_paths, target_objects,
      common_data_ptr_->lc_param_ptr->safety.rss_params_for_stuck,
      lane_change_debug_.collision_check_objects);
    candidateReason(safety_check_with_stuck_rss.is_safe?"native_stuck_RSS_pass":"native_stuck_RSS_rejected");
    return safety_check_with_stuck_rss.is_safe;
  }

  candidateReason(safety_check_with_normal_rss.is_safe?"native_RSS_pass":"native_RSS_rejected");
  return safety_check_with_normal_rss.is_safe;
}

bool NormalLaneChange::check_reachable_candidate_admission(
  const LaneChangePath & candidate,const ros::WallTime & deadline) const
{
  bool enabled=false,strict=false,bounded=false,partial=false;
  ros::param::getCached("/planning/lc_reachable_admission",enabled);
  if(!enabled || getModuleType()!=LaneChangeModuleType::NORMAL)return true;
  ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
  ros::param::getCached("/planning/lc_continuous_bounded_output",bounded);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial);
  if(!strict||!bounded||!partial)return false;
  if(ros::WallTime::now()>=deadline){candidateReason("reachable_deadline_before_attempt");return false;}
  namespace lc=utils::lane_change;
  const auto started=ros::WallTime::now();
  lane_change::reachable_admission::Result result;
  try {
    if(!planner_data_->dynamic_object || !common_data_ptr_->current_acceleration ||
       !common_data_ptr_->acceleration_limits.enabled())throw std::logic_error("missing_admission_inputs");
    double amin=std::max(getCommonParam().min_acc,lane_change_parameters_->trajectory.min_longitudinal_acc);
    double smoother_acc=0,jmin=0,jmax=0;
    if(!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother_acc)||
       !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",jmin)||
       !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/max_jerk",jmax))
      throw std::logic_error("missing_braking_limits");
    amin=std::max(amin,smoother_acc);
    bool rising=false,invariant=false;
    ros::param::getCached("/planning/interpolate_rising_velocity",rising);
    ros::param::getCached("/planning/lc_anchor_invariant_curvature",invariant);
    const auto anchored=lc::anchor_maneuver_at_ego(candidate.path,
      planner_data_->findEgoSegmentIndex(candidate.path.points),getEgoPosition(),rising);
    const auto &raw=*planner_data_->dynamic_object;
    const double age=(planner_data_->self_odometry->header.stamp-raw.header.stamp).toSec();
    if(!std::isfinite(age)||age<-.010||age>.5)throw std::logic_error("stale_grid_objects");
    // Stack-local immutable world only; never reused across admission calls.
    const auto world=prepare_admission_world();
    const double horizon=world->horizon+std::max(0.,-age);
    const auto full_arc=lc::safety_arc_lengths(anchored.first);
    const auto&lanes=get_target_lanes();
    if(lanes.empty())throw std::logic_error("missing partial target lanes");
    const auto target=getRouteHandler()->getCenterLinePath(lanes,0.,
      lanelet::geometry::length3d(lanelet::LaneletSequence(lanes)));
    const auto&lat=lane_change_parameters_->trajectory.lat_acc_map.base_max_acc;
    if(lat.empty())throw std::logic_error("missing partial lateral limits");
    const auto end=autoware::motion_utils::findNearestIndex(anchored.first.points,candidate.info.lane_changing_end.position);
    if(autoware_utils::calc_distance2d(anchored.first.points[end].point.pose,candidate.info.lane_changing_end)>1.)
      throw std::logic_error("partial LC endpoint not covered");
    const auto certificate=[&](){return currentExitCertificate(anchored.first,get_current_lanes(),get_target_lanes(),
      getCommonParam().vehicle_info,anchored.second,end,getModuleType()==LaneChangeModuleType::NORMAL);};
    const auto caps=certifiedEnvelope(certificate,anchored.first,common_data_ptr_->current_lanes_path,
      target,prev_module_output_.path,anchored.second,end,getEgoPosition(),getCommonParam().max_vel,
      *std::min_element(lat.begin(),lat.end()),autoware::motion_utils::findNearestIndex(
        anchored.first.points,candidate.info.lane_changing_start.position),true,
      invariant?&candidate.path:nullptr,anchored.second,nullptr);
    std::vector<double> arc;
    for(size_t i=anchored.second;i<full_arc.size();++i)arc.push_back(full_arc[i]-full_arc[anchored.second]);
    const auto limit=[&](double v){return common_data_ptr_->acceleration_limits.limit(v,
      std::min(getCommonParam().max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc));};
    auto model=candidate;model.type=lane_change::PathType::SpatialSpeedProfile;
    result=lane_change::reachable_admission::evaluate(true,arc,caps,getEgoVelocity(),
      common_data_ptr_->current_acceleration->accel.accel.linear.x,horizon,amin,jmin,jmax,limit,
      [&](const auto&proposal,std::string&reason){
        auto path=anchored.first;
        for(size_t i=0;i<proposal.speeds.size();++i)
          path.points[anchored.second+i].point.longitudinal_velocity_mps=proposal.speeds[i];
        return check_strict_maneuver_safety(model,path,reason,false,false,world.get());
      },[&](){return ros::WallTime::now()<deadline;},[&](){
        bool positive=false,adaptive=false;
        ros::param::getCached("/planning/lc_rss_positive_grid",positive);
        ros::param::getCached("/planning/lc_adaptive_positive_grid",adaptive);
        return positive&&adaptive;
      }());
  }catch(const std::exception&error){result.reason=error.what();}
  if(!result.accepted && result.reason.find("speed reference does not cover candidate")==0) {
    // An unapproved candidate diagnostic, never a runtime failure or stop event.
    // Serialization is exception-contained by the existing snapshot publisher.
    publishStrictSafetySnapshot("reachable_admission_rejected",candidate,candidate.path,nullptr,
      ros::Time::now(),result.reason,"not_an_active_maneuver","not_evaluated");
  }
  ROS_INFO_THROTTLE(1.0,"LC reachable admission: accepted=%d attempted=%u target=%.3f cost_ms=%.3f reason=%s",
    result.accepted,result.attempted,result.target,(ros::WallTime::now()-started).toSec()*1000.,result.reason.c_str());
  candidateReason(std::string(result.accepted?"reachable_pass:":"reachable_rejected:")+result.reason);
  return result.accepted;
}

bool NormalLaneChange::check_strict_maneuver_safety(
  const LaneChangePath & candidate, const PathWithLaneId & published_path,
  std::string & reason, const bool approved, const bool current_lane_following) const
{
  return check_strict_maneuver_safety(candidate,published_path,reason,approved,
    current_lane_following,nullptr);
}

std::shared_ptr<const utils::lane_change::PreparedSafetyWorld>
NormalLaneChange::prepare_admission_world() const
{
  namespace lc=utils::lane_change;
  if(!planner_data_->dynamic_object || !planner_data_->self_odometry ||
     !common_data_ptr_->current_acceleration)throw std::logic_error("missing_motion_or_objects");
  auto world=std::make_shared<lc::PreparedSafetyWorld>();
  world->planner=planner_data_.get();world->common=common_data_ptr_.get();
  world->raw=planner_data_->dynamic_object.get();world->odom=planner_data_->self_odometry.get();
  world->acceleration=common_data_ptr_->current_acceleration.get();
  world->route=getRouteHandler().get();world->map=getRouteHandler()->getLaneletMapPtr().get();
  world->route_uuid=getRouteHandler()->getRouteUuid();world->route_header=getRouteHandler()->getRouteHeader();
  world->raw_stamp=planner_data_->dynamic_object->header.stamp;
  world->odom_stamp=planner_data_->self_odometry->header.stamp;
  world->raw_frame=planner_data_->dynamic_object->header.frame_id;
  world->odom_frame=planner_data_->self_odometry->header.frame_id;
  world->raw_age=(world->odom_stamp-world->raw_stamp).toSec();
  world->resolution=lane_change_parameters_->safety.collision_check.prediction_time_resolution;
  auto observations=lc::prepare_safety_observations(*planner_data_->dynamic_object,
    *planner_data_->self_odometry,world->resolution);
  world->objects=std::move(observations.first);world->horizon=observations.second;
  const auto add=[&](const lanelet::ConstLanelets&lanes,std::vector<lanelet::Id>&ids){
    for(const auto&lane:lanes){
      ids.push_back(lane.id());const auto polygon=utils::toPolygon2d(lane);
      if(world->corridor.empty()){world->corridor.push_back(polygon);continue;}
      lc::PreparedSafetyWorld::Corridor merged;
      boost::geometry::union_(world->corridor,polygon,merged);world->corridor=std::move(merged);
    }
  };
  add(get_current_lanes(),world->current_ids);add(get_target_lanes(),world->target_ids);
  if(world->corridor.empty())throw std::logic_error("missing_lane_corridor");
  return world;
}

bool NormalLaneChange::check_strict_maneuver_safety(
  const LaneChangePath & candidate, const PathWithLaneId & published_path,
  std::string & reason, const bool approved, const bool current_lane_following,
  const utils::lane_change::PreparedSafetyWorld * world) const
{
  namespace lc = utils::lane_change;
  try {
    bool measured_diagnostic_only=false;
    ros::param::getCached("/planning/lc_measured_motion_diagnostic_only",measured_diagnostic_only);
    std::string measured_diagnostic;
    const auto record_measured=[&](const std::string & detail){
      if(measured_diagnostic.empty())measured_diagnostic=detail;
      else if(detail.find("critical_predicted_physical:")==0 &&
          measured_diagnostic.find("critical_predicted_physical:")==std::string::npos)
        measured_diagnostic+="; "+detail;
      ROS_WARN_THROTTLE(1.0,"Strict LC measured future diagnostic only: %s",detail.c_str());
    };
    const auto & data = common_data_ptr_;
    if (!data->current_acceleration || !planner_data_->dynamic_object) {
      throw std::logic_error("missing_motion_or_objects");
    }
    const auto & raw = *planner_data_->dynamic_object;
    const double raw_age = (planner_data_->self_odometry->header.stamp - raw.header.stamp).toSec();
    if(world){
      std::vector<lanelet::Id> current,target;
      for(const auto&lane:get_current_lanes())current.push_back(lane.id());
      for(const auto&lane:get_target_lanes())target.push_back(lane.id());
      if(current_lane_following || world->planner!=planner_data_.get() ||
         world->common!=common_data_ptr_.get() || world->raw!=&raw ||
         world->odom!=planner_data_->self_odometry.get() ||
         world->acceleration!=data->current_acceleration.get() ||
         world->route!=getRouteHandler().get() || world->map!=getRouteHandler()->getLaneletMapPtr().get() ||
         world->route_uuid!=getRouteHandler()->getRouteUuid() || world->route_header!=getRouteHandler()->getRouteHeader() ||
         world->raw_stamp!=raw.header.stamp || world->odom_stamp!=planner_data_->self_odometry->header.stamp ||
         world->raw_frame!=raw.header.frame_id || world->odom_frame!=planner_data_->self_odometry->header.frame_id ||
         world->resolution!=lane_change_parameters_->safety.collision_check.prediction_time_resolution ||
         world->current_ids!=current || world->target_ids!=target)
        throw std::logic_error("prepared_safety_world_epoch_mismatch");
    }
    if (raw.header.frame_id != planner_data_->self_odometry->header.frame_id ||
        !std::isfinite(raw_age) || raw_age < -0.010 || raw_age > 0.5) {
      throw std::logic_error("stale_or_wrong_frame_objects");
    }
    const double age=std::max(0.0,raw_age);
    const double ego_time_advance=std::max(0.0,-raw_age);
    bool rising_interpolation=false;
    ros::param::getCached("/planning/interpolate_rising_velocity",rising_interpolation);
    const auto anchored = lc::anchor_maneuver_at_ego(published_path,
      planner_data_->findEgoSegmentIndex(published_path.points),getEgoPosition(),rising_interpolation);
    const auto & final_path = anchored.first;
    const auto arc = lc::safety_arc_lengths(final_path);
    const size_t begin = anchored.second;
    auto check_vehicle=getCommonParam().vehicle_info;
    if(current_lane_following){
      const double envelope=lc::current_lane_cancel_envelope(final_path,candidate.path,
        getEgoPose(),candidate.info.shift_line.start,check_vehicle);
      check_vehicle=lc::cancel_envelope_vehicle(check_vehicle,envelope);
      ROS_INFO_THROTTLE(5.0,"Strict current-lane shared-prefix accepted: measured pose envelope=%.4f m",envelope);
    }
    const size_t physical_end = current_lane_following ? final_path.points.size()-1 :
      autoware::motion_utils::findNearestIndex(final_path.points,candidate.info.lane_changing_end.position);
    const size_t end=std::max(begin,physical_end);
    if (!current_lane_following && autoware_utils::calc_distance2d(
          final_path.points[physical_end].point.pose, candidate.info.lane_changing_end) > 2.0) {
      throw std::logic_error("maneuver_endpoint_not_covered");
    }
    std::pair<ExtendedPredictedObjects,double> local_observations;
    if(!world)local_observations=lc::prepare_safety_observations(raw,*planner_data_->self_odometry,
      lane_change_parameters_->safety.collision_check.prediction_time_resolution);
    const auto & checked_objects=world?world->objects:local_observations.first;
    const double horizon=world?world->horizon:local_observations.second;
    // Receding-horizon contract: all static maneuver geometry, but dynamic
    // collision checks only within the input's shared observed future. This
    // is NOT a guarantee of collision freedom after that window; recheck each
    // cycle, including approved/abort/braking outputs.
    const size_t prepare_end=current_lane_following ? begin :
      autoware::motion_utils::findNearestIndex(final_path.points,candidate.info.lane_changing_start.position);
    const double tail_acc=std::min(getCommonParam().max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc);
    const double decel=-std::max(getCommonParam().min_acc,lane_change_parameters_->trajectory.min_longitudinal_acc);
    auto modeled=final_path;
    const bool frenet=!current_lane_following && candidate.type==lane_change::PathType::FrenetPlanner;
    if(current_lane_following) {
      // Exact upstream publication remains unchanged. Its speeds are normal
      // lane-following ceilings; this is only the reachable prediction model.
      modeled=lc::current_lane_following_prediction(final_path,getEgoPose(),getEgoVelocity(),
        tail_acc,decel,data->acceleration_limits);
    } else if(frenet) {
      // Only use this temporary constant profile to construct the independent
      // measured-motion hypothesis; replace its planned member with native
      // Frenet time/pose/velocity below, never interpret road caps as speed.
      for(auto & point:modeled.points)point.point.longitudinal_velocity_mps=getEgoVelocity();
    } else if(!current_lane_following) {
      modeled = lc::maneuver_prediction_profile(final_path,begin,end,prepare_end,
        candidate.type==lane_change::PathType::SpatialSpeedProfile,getEgoVelocity(),
        candidate.info.longitudinal_acceleration.prepare,
        candidate.info.longitudinal_acceleration.lane_changing,tail_acc,decel,data->acceleration_limits);
    }
    auto hypotheses = lc::maneuver_motion_hypotheses(
      modeled, begin, end, getEgoVelocity(), data->current_acceleration->accel.accel.linear.x,
      getCommonParam().max_vel, 2.5,
      lane_change_parameters_->safety.collision_check.prediction_time_resolution,horizon+ego_time_advance);
    if(frenet) {
      const auto native=lc::restore_frenet_transition(
        lc::convert_to_predicted_path(data,candidate.frenet_path,1),candidate.frenet_path);
      hypotheses[0]=lc::frenet_observation_hypothesis(final_path,native,getEgoPose(),
        getEgoVelocity(),horizon+ego_time_advance,lane_change_parameters_->safety.collision_check.prediction_time_resolution,
        tail_acc,decel,data->acceleration_limits);
    }
    if(ego_time_advance>0)
      for(auto & hypothesis:hypotheses)
        hypothesis=lc::rebase_observation_prediction(hypothesis,ego_time_advance,horizon);
    ROS_INFO_THROTTLE(5.0,"Strict LC receding observation: horizon=%.3f raw_age=%.6f ego_advance=%.6f reference=%.9f objects=%zu",horizon,raw_age,ego_time_advance,
      std::max(planner_data_->self_odometry->header.stamp.toSec(),raw.header.stamp.toSec()),checked_objects.size());

    // Only actual current/target lane polygons form the admissible corridor.
    // An overlapping unrelated lane's ID or an intersection expansion cannot
    // silently legalize the maneuver here. Check complete vehicle footprints.
    using Polygon = decltype(lc::get_ego_footprint(getEgoPose(), getCommonParam().vehicle_info));
    boost::geometry::model::multi_polygon<Polygon> local_corridor;
    const auto add_lanes = [&](const lanelet::ConstLanelets & lanes) {
      for (const auto & lane : lanes) {
        const auto polygon = utils::toPolygon2d(lane);
        if (local_corridor.empty()) { local_corridor.push_back(polygon); continue; }
        decltype(local_corridor) merged;
        boost::geometry::union_(local_corridor, polygon, merged);
        local_corridor = std::move(merged);
      }
    };
    if(!world){
      add_lanes(get_current_lanes());
      if(!current_lane_following)add_lanes(get_target_lanes());
    }
    const auto & corridor=world?world->corridor:local_corridor;
    if (corridor.empty()) throw std::logic_error("missing_lane_corridor");
    if(measured_diagnostic_only && !boost::geometry::covered_by(
        lc::get_ego_footprint(getEgoPose(),getCommonParam().vehicle_info),corridor))
      throw std::logic_error("actual_t0_footprint_outside_corridor");
    if(current_lane_following && !boost::geometry::covered_by(
        lc::get_ego_footprint(getEgoPose(),getCommonParam().vehicle_info),corridor))
      throw std::logic_error("current_lane_cancel_ego_outside_corridor");
    double geometry_end=arc[end];
    if(current_lane_following) {
      geometry_end=arc[begin];
      for(size_t h=0;h<hypotheses.size();++h)
        for(const auto & point:hypotheses[h]) {
          if(lc::measured_failure_is_diagnostic(measured_diagnostic_only,h,point.time))continue;
          geometry_end=std::max(geometry_end,lc::project_safety_pose(final_path,arc,point.pose,.1).arc);
        }
    }
    for (double s = arc[begin]; s <= geometry_end + 1e-6; s += 0.5) {
      const auto pose = lc::safety_pose_at_arc(final_path, arc, std::min(s,geometry_end));
      if (!boost::geometry::covered_by(lc::get_ego_footprint(pose, check_vehicle), corridor))
        throw std::logic_error("maneuver_footprint_outside_corridor");
    }
    if (!boost::geometry::covered_by(lc::get_ego_footprint(
          lc::safety_pose_at_arc(final_path,arc,geometry_end),check_vehicle),corridor))
      throw std::logic_error("maneuver_endpoint_footprint_outside_corridor");
    for (size_t h=0;h<hypotheses.size();++h) {
      for (const auto & point : hypotheses[h]) {
        if (!boost::geometry::covered_by(lc::get_ego_footprint(point.pose,check_vehicle),corridor)) {
          if(lc::measured_failure_is_diagnostic(measured_diagnostic_only,h,point.time)) {
            record_measured("measured_future_footprint_outside_corridor");continue;
          }
          throw std::logic_error("actual_or_planned_footprint_outside_corridor");
        }
      }
    }

    auto checked = candidate;
    checked.path = final_path;
    checked.type = lane_change::PathType::SpatialSpeedProfile;
    // Geometry only, local to this strict call. Every object's policy,
    // velocity-dependent extension, polygon and RSS verdict is recomputed.
    const utils::path_safety_checker::PreparedPathProjection projection(checked.path);
    std::vector<double> remaining_prepare_times;
    for(const auto & hypothesis:hypotheses)
      remaining_prepare_times.push_back(current_lane_following ? 0.0 : lc::remaining_prepare_time(
        final_path,prepare_end,hypothesis,candidate.info.duration.prepare<=1e-6));
    CollisionCheckDebugMap debug;
    // Do not omit a rear vehicle in the current lane during an abort. The
    // normal category filter intentionally keeps current-lane leaders only.
    for (const auto & transformed : checked_objects) {
      for (size_t hypothesis_index=0;hypothesis_index<hypotheses.size();++hypothesis_index) {
        const auto & hypothesis=hypotheses[hypothesis_index];
        checked.info.duration.prepare=remaining_prepare_times[hypothesis_index];
        // Check the actual present sample separately before classifying any
        // later RSS failure as a hypothetical non-response diagnostic.
        if(measured_diagnostic_only && hypothesis_index==1) {
          auto present=transformed;
          for(auto & prediction:present.predicted_paths)prediction.path.resize(1);
          auto actual_present=hypothesis;
          auto actual_pose=getEgoPose();
          // Same max(odom,objects) timestamp as the future predictions. At most
          // 10ms advance along measured heading, never project onto candidate XY.
          const auto & q=actual_pose.orientation;
          const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          const double distance=getEgoVelocity()*ego_time_advance+
            .5*data->current_acceleration->accel.accel.linear.x*ego_time_advance*ego_time_advance;
          actual_pose.position.x+=distance*std::cos(yaw);
          actual_pose.position.y+=distance*std::sin(yaw);
          actual_present.front().pose=actual_pose;
          actual_present.front().velocity=std::max(0.0,getEgoVelocity()+
            data->current_acceleration->accel.accel.linear.x*ego_time_advance);
          if(!boost::geometry::covered_by(lc::get_ego_footprint(actual_pose,check_vehicle),corridor))
            throw std::logic_error("actual_aligned_t0_footprint_outside_corridor");
          if(lc::strict_physical_overlap(actual_present,present,check_vehicle))
            throw std::logic_error("actual_t0_physical_overlap");
          for(bool cancel:{false,true}) {
            debug.clear();
            const auto & rss=cancel?lane_change_parameters_->safety.rss_params_for_abort:
              lane_change_parameters_->safety.rss_params;
            const bool collision=current_lane_following ?
              !utils::path_safety_checker::check_safety_with_rss(checked.path,actual_present,
                check_vehicle,{present},debug,rss,true,1.0,
                lane_change_parameters_->safety.collision_check.th_yaw_diff,&projection):
              is_colliding(checked,present,actual_present,rss,debug,approved,&projection);
            if(collision)throw std::logic_error("measured_t0_rss_collision");
          }
        }
        // RSS intentionally skips large relative headings; physical overlap
        // must not. This opt-in guard keeps all raw prediction hypotheses.
        if(const auto hit=lc::strict_physical_overlap(hypothesis,transformed,check_vehicle)) {
          std::ostringstream detail;detail<<"strict_physical_overlap uuid=";
          for(const auto byte:transformed.uuid.uuid)
            detail<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
          detail<<std::dec<<" hypothesis="<<(hypothesis_index==0?"planned":"measured_longitudinal")
                <<" unsafe_time="<<*hit;
          if(lc::measured_failure_is_diagnostic(measured_diagnostic_only,hypothesis_index,*hit))
            record_measured("critical_predicted_physical: "+detail.str());
          else throw std::logic_error(detail.str());
        }
        for(bool cancel:{false,true}) {
          debug.clear();
          const auto & rss=cancel ? lane_change_parameters_->safety.rss_params_for_abort :
                                   lane_change_parameters_->safety.rss_params;
          const bool collision=current_lane_following ?
            !utils::path_safety_checker::check_safety_with_rss(
              checked.path,hypothesis,check_vehicle,{transformed},debug,rss,
              true,1.0,lane_change_parameters_->safety.collision_check.th_yaw_diff,&projection) :
            is_colliding(checked,transformed,hypothesis,rss,debug,approved,&projection);
          if(!collision)continue;
          // First failed object/hypothesis/policy only. No polygon dumps or
          // successful-cycle logging; the enclosing failure JSON escapes this.
          std::ostringstream detail;
          detail<<std::setprecision(9)<<"planned_or_actual_rss_collision uuid=";
          for(const auto byte:transformed.uuid.uuid)
            detail<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
          detail<<std::dec<<" hypothesis="<<(hypothesis_index==0?"planned":"measured_longitudinal")
                <<" policy="<<(cancel?"cancel":"execution")<<" horizon="<<horizon
                <<" object_age="<<age<<" prepare_duration="<<checked.info.duration.prepare;
          const auto found=debug.find(transformed.uuid.uuid);
          if(found!=debug.end()) {
            const auto & failure=found->second;
            detail<<" unsafe_reason="<<failure.unsafe_reason
                  <<" ego_xy="<<failure.expected_ego_pose.position.x<<","<<failure.expected_ego_pose.position.y
                  <<" object_xy="<<failure.expected_object_pose.position.x<<","<<failure.expected_object_pose.position.y;
            // Report only an exact reconstructed RSS sample match, not an
            // invented nearest sample timestamp.
            for(const auto & object_point:failure.object_predicted_path) {
              const auto ego_point=utils::path_safety_checker::calc_interpolated_pose_with_velocity(
                hypothesis,object_point.time);
              if(ego_point && autoware_utils::calc_distance2d(
                    ego_point->pose,failure.expected_ego_pose)<1e-5) {
                detail<<" unsafe_time="<<object_point.time;break;
              }
            }
          }
          if(measured_diagnostic_only && hypothesis_index==1)
            record_measured("measured_future_RSS: "+detail.str());
          else throw std::logic_error(detail.str());
        }
      }
    }
    reason = measured_diagnostic.empty()?"safe":"safe_with_measured_diagnostic: "+measured_diagnostic;
    if(approved && !measured_diagnostic.empty())
      publishStrictSafetySnapshot("checked_measured_diagnostic",candidate,published_path,
        &published_path,ros::Time::now(),reason,"","");
    return true;
  } catch (const std::exception & e) {
    reason = e.what();
    return false;
  }
}

void NormalLaneChange::publishStrictSafetySnapshot(
  const std::string & stage,const LaneChangePath & candidate,const PathWithLaneId & original_path,
  const PathWithLaneId * selected_path,const ros::Time & failure_stamp,
  const std::string & original_reason,const std::string & cancel_reason,
  const std::string & brake_reason,const std::vector<double> * attempted_targets) const
{
  try {
    static ros::WallTime last_failure,last_selected,last_measured_diagnostic,last_cancel,last_braking,last_admission,last_unapproved;
    auto & last_capture=stage=="unapproved_request_candidate"?last_unapproved:stage=="no_safe_maneuver"?last_failure:
      (stage=="checked_measured_diagnostic"?last_measured_diagnostic:
        (stage=="checked_current_lane_cancel"?last_cancel:
          (stage=="checked_braking_selected"?last_braking:
            (stage=="reachable_admission_rejected"?last_admission:last_selected))));
    const auto now=ros::WallTime::now();
    if((now-last_capture).toSec()>=(stage=="checked_current_lane_cancel"?0.0:
        (stage=="no_safe_maneuver"?.5:1.0))){
      last_capture=now;
      namespace lc=utils::lane_change;
      using Tree=boost::property_tree::ptree;
      Tree packet;
      packet.put("schema","lc_safety_snapshot_v1");
      packet.put("snapshot_rate_revision",2);
      packet.put("stage",stage);
      if(stage=="unapproved_request_candidate") {
        packet.put("candidate_complete",true);
        packet.put("approved",false);
        packet.put("request_source","NORMAL_preference");
        packet.put("request_direction",getDirection()==Direction::LEFT?"LEFT":
          (getDirection()==Direction::RIGHT?"RIGHT":"UNKNOWN"));
        packet.put("decision_epoch","final_candidate_generation_before_approval");
        packet.put("route_context","externalroute_join_required");
        packet.put("optional_route_proof","unknown_not_computed_by_diagnostic");
        packet.put("native_candidate_safe",true);
        packet.put("path_valid_source","previous_module_status_not_candidate");
        const auto & native=common_data_ptr_->transient_data;
        packet.put("native_reserve.has_valid_lc_distance",native.has_valid_lc_distance);
        packet.put("native_reserve.current_min",native.current_dist_buffer.min);
        packet.put("native_reserve.current_max",native.current_dist_buffer.max);
        packet.put("native_reserve.dist_to_terminal_end",native.dist_to_terminal_end);
        packet.put("native_reserve.near_terminal",is_near_terminal_end());
        packet.put("native_reserve.ego_lane",common_data_ptr_->lanes_ptr->ego_lane.id());
        packet.add_child("route_header",lc::snapshot_ros_message(getRouteHeader()));
        Tree topology;
        const auto handler=getRouteHandler();const auto graph=handler->getRoutingGraphPtr();
        for(const auto & lanes:{get_current_lanes(),get_target_lanes()})for(const auto & lane:lanes) {
          Tree row,following;row.put("id",lane.id());
          if(graph)for(const auto & next:graph->following(lane))if(handler->isRouteLanelet(next)) {
            Tree id;id.put_value(next.id());following.push_back({"",id});
          }
          row.put("graph_available",bool(graph));row.add_child("route_following",following);
          topology.push_back({"",row});
        }
        packet.add_child("route_prefix_topology",topology);
      }
      bool rising_interpolation=false,partial_acceleration=false;
      ros::param::getCached("/planning/interpolate_rising_velocity",rising_interpolation);
      ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial_acceleration);
      packet.put("interpolate_rising_velocity",rising_interpolation);
      packet.put("partial_acceleration_candidates",partial_acceleration);
      bool fresh_commit_input=false;
      ros::param::getCached("/planning/lc_fresh_commit_input",fresh_commit_input);
      packet.put("fresh_commit_input",fresh_commit_input);
      bool measured_diagnostic_only=false;
      ros::param::getCached("/planning/lc_measured_motion_diagnostic_only",measured_diagnostic_only);
      packet.put("measured_motion_diagnostic_only",measured_diagnostic_only);
      packet.put("measured_diagnostic_reason",stage=="checked_measured_diagnostic"?original_reason:"");
      packet.put("measured_predicted_physical_critical",
        original_reason.find("critical_predicted_physical:")!=std::string::npos);
      packet.put("partial_speed_cap_semantics","fresh_hard_envelope_v1");
      packet.put("partial_tail_connection_revision",2);
      packet.put("partial_hard_phase_revision",3);
      packet.put("partial_hard_reference_revision",2);
      bool adaptive_positive=false;
      ros::param::getCached("/planning/lc_adaptive_positive_grid",adaptive_positive);
      // An explicitly supplied empty list means construction failed BEFORE
      // sampling. Do not rewrite that evidence as nine attempted candidates.
      const bool recorded_attempts=attempted_targets!=nullptr;
      packet.put("adaptive_positive_grid",adaptive_positive);
      bool reachable_admission=false;
      ros::param::getCached("/planning/lc_reachable_admission",reachable_admission);
      packet.put("reachable_admission",reachable_admission);
      packet.put("partial_acceleration_targets_attempted",recorded_attempts);
      packet.put("partial_acceleration_family_revision",adaptive_positive&&recorded_attempts?3:2);
      try {
        double smoother_min=0;
        if(!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother_min))
          throw std::logic_error("missing target-family braking limit");
        const double minimum_acceleration=std::max({getCommonParam().min_acc,
          lane_change_parameters_->trajectory.min_longitudinal_acc,smoother_min});
        const double active_max=common_data_ptr_->acceleration_limits.limit(getEgoVelocity(),
          std::min(getCommonParam().max_acc,lane_change_parameters_->trajectory.max_longitudinal_acc));
        Tree targets;
        const auto targets_used=recorded_attempts?*attempted_targets:
          bounded_speed_grid::acceleration_targets(active_max,minimum_acceleration);
        for(double target:targets_used) {
          Tree value;value.put_value(target);targets.push_back({"",value});
        }
        packet.add_child("partial_acceleration_targets",targets);
        packet.put("partial_acceleration_targets_valid",true);
      } catch(const std::exception & error) {
        packet.put("partial_acceleration_targets_valid",false);
        packet.put("partial_acceleration_targets_error",error.what());
      }
      bool continuous_bounded_output=false;
      ros::param::getCached("/planning/lc_continuous_bounded_output",continuous_bounded_output);
      packet.put("continuous_bounded_output",continuous_bounded_output);
      bool anchor_invariant=false,partial_grid=false,strict_mode=false;
      ros::param::getCached("/planning/lc_anchor_invariant_curvature",anchor_invariant);
      ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial_grid);
      ros::param::getCached("/planning/lc_strict_maneuver_safety",strict_mode);
      anchor_invariant=anchor_invariant&&strict_mode&&
        lc::owns_bounded_longitudinal_output(continuous_bounded_output,partial_grid,isAbortState(),isStopState());
      packet.put("anchor_invariant_curvature",anchor_invariant);
      packet.put("partial_curvature_geometry_revision",anchor_invariant?2:1);
      bool post_end_tail=false;
      ros::param::getCached("/planning/lc_post_end_bounded_tail",post_end_tail);
      post_end_tail=post_end_tail&&strict_mode&&getModuleType()==LaneChangeModuleType::NORMAL&&
        lc::owns_bounded_longitudinal_output(continuous_bounded_output,partial_grid,isAbortState(),isStopState());
      packet.put("post_end_bounded_tail",post_end_tail);
      packet.put("partial_hard_envelope_applicable",!isAbortState()&&!isStopState());
      if(selected_path)packet.add_child("selected_path",lc::snapshot_ros_message(*selected_path));
      packet.put("failure_stamp",failure_stamp.toSec());
      std::ostringstream uuid;
      for(const auto byte:common_data_ptr_->route_handler_ptr->getRouteUuid().uuid)
        uuid<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(byte);
      packet.put("route_uuid",uuid.str());packet.put("module",getModuleTypeStr());
      packet.put("abort",isAbortState());packet.put("path_valid",status_.is_valid_path);
      packet.put("continue_reason",original_reason);packet.put("cancel_reason",cancel_reason);
      packet.put("brake_reason",brake_reason);
      packet.add_child("output_path",lc::snapshot_ros_message(original_path));
      if(original_path.points.size()>=2)
        packet.put("output_ego_segment",planner_data_->findEgoSegmentIndex(original_path.points));
      packet.add_child("upstream_path",lc::snapshot_ros_message(prev_module_output_.path));
      packet.add_child("current_speed_reference",lc::snapshot_ros_message(common_data_ptr_->current_lanes_path));
      // Snapshot the same fresh hard envelope, including exact referenced
      // paths, so later replay need not reconstruct map state from a newer map.
      try {
        const auto & target_lanes=get_target_lanes();
        if(target_lanes.empty())throw std::logic_error("missing snapshot target lanes");
        const auto target_reference=getRouteHandler()->getCenterLinePath(
          target_lanes,0.0,lanelet::geometry::length3d(lanelet::LaneletSequence(target_lanes)));
        packet.add_child("target_speed_reference",lc::snapshot_ros_message(target_reference));
        const auto & lateral_map=lane_change_parameters_->trajectory.lat_acc_map.base_max_acc;
        if(lateral_map.empty())throw std::logic_error("missing snapshot lateral limits");
        const double lateral_limit=*std::min_element(lateral_map.begin(),lateral_map.end());
        packet.put("partial_hard_lateral_limit",lateral_limit);
        if(stage=="unapproved_request_candidate") {
          packet.put("preapproval_capture_revision",2);
          packet.put("hard_envelope_recomputed",false);
          bool certified=false,positive=false;
          ros::param::getCached("/planning/lc_certified_target_only_speed_reference",certified);
          ros::param::getCached("/planning/lc_rss_positive_grid",positive);
          packet.put("certified_target_only_speed_reference_enabled",certified);
          packet.put("rss_positive_grid",positive);
          // Exact sources above permit offline reconstruction. Do not repeat
          // projection/certificate work on the critical preapproval callback.
          throw std::logic_error("not_recomputed_in_preapproval_diagnostic");
        }
        const auto anchored=lc::anchor_maneuver_at_ego(original_path,
          planner_data_->findEgoSegmentIndex(original_path.points),getEgoPosition(),rising_interpolation);
        const size_t end=autoware::motion_utils::findNearestIndex(
          anchored.first.points,candidate.info.lane_changing_end.position);
        if(autoware_utils::calc_distance2d(anchored.first.points[end].point.pose,
            candidate.info.lane_changing_end)>1.0)throw std::logic_error("snapshot LC endpoint not covered");
        const auto start=autoware::motion_utils::findNearestIndex(
          anchored.first.points,candidate.info.lane_changing_start.position);
        packet.put("partial_hard_lc_start",start);
        bool certificate_applied=false;
        const auto certificate=[&](){auto result=currentExitCertificate(anchored.first,get_current_lanes(),get_target_lanes(),
          getCommonParam().vehicle_info,anchored.second,end,getModuleType()==LaneChangeModuleType::NORMAL);
          certificate_applied=static_cast<bool>(result);return result;};
        bool rss_positive_grid=false;ros::param::getCached("/planning/lc_rss_positive_grid",rss_positive_grid);
        packet.put("rss_positive_grid",rss_positive_grid);
        const auto caps=certifiedEnvelope(certificate,anchored.first,
          common_data_ptr_->current_lanes_path,target_reference,prev_module_output_.path,
          anchored.second,end,getEgoPosition(),getCommonParam().max_vel,lateral_limit,start,true,
          anchor_invariant?&original_path:nullptr,anchored.second,
          post_end_tail&&anchored.second>end?&candidate.info.lane_changing_end.position:nullptr);
        packet.put("certified_target_only_speed_reference",certificate_applied);
        packet.put("partial_post_end_tail_applied",post_end_tail&&anchored.second>end);
        packet.add_child("partial_hard_envelope_path",lc::snapshot_ros_message(anchored.first));
        packet.put("partial_hard_begin",anchored.second);packet.put("partial_hard_lc_end",end);
        Tree values;for(const double cap:caps){Tree value;value.put_value(cap);values.push_back({"",value});}
        packet.add_child("partial_hard_caps",values);packet.put("partial_hard_envelope_valid",true);
      } catch(const std::exception & error) {
        packet.put("partial_hard_envelope_valid",false);
        packet.put("partial_hard_envelope_error",error.what());
      }
      packet.add_child("candidate_path",lc::snapshot_ros_message(candidate.path));
      if(planner_data_->self_odometry)
        packet.add_child("odometry",lc::snapshot_ros_message(*planner_data_->self_odometry));
      if(planner_data_->dynamic_object)
        packet.add_child("objects",lc::snapshot_ros_message(*planner_data_->dynamic_object));
      if(common_data_ptr_->current_acceleration)
        packet.add_child("acceleration",lc::snapshot_ros_message(*common_data_ptr_->current_acceleration));
      const auto phase=[&](const char * name,const auto & value){
        packet.put(std::string("candidate.")+name+".prepare",value.prepare);
        packet.put(std::string("candidate.")+name+".changing",value.lane_changing);};
      packet.put("candidate.type",static_cast<int>(candidate.type));
      phase("acceleration",candidate.info.longitudinal_acceleration);
      phase("velocity",candidate.info.velocity);phase("duration",candidate.info.duration);
      phase("length",candidate.info.length);
      packet.put("candidate.lateral_acceleration",candidate.info.lateral_acceleration);
      packet.put("candidate.terminal_velocity",candidate.info.terminal_lane_changing_velocity);
      packet.add_child("candidate.start",lc::snapshot_ros_message(candidate.info.lane_changing_start));
      packet.add_child("candidate.end",lc::snapshot_ros_message(candidate.info.lane_changing_end));
      packet.add_child("candidate.shift_start",lc::snapshot_ros_message(candidate.info.shift_line.start));
      packet.add_child("candidate.shift_end",lc::snapshot_ros_message(candidate.info.shift_line.end));
      packet.put("candidate.shift_start_idx",candidate.info.shift_line.start_idx);
      packet.put("candidate.shift_end_idx",candidate.info.shift_line.end_idx);
      packet.put("candidate.shift_start_length",candidate.info.shift_line.start_shift_length);
      packet.put("candidate.shift_end_length",candidate.info.shift_line.end_shift_length);
      const auto lanes=[&](const char * key,const auto & list){
        Tree records;for(const auto & lane:list){Tree record,ring;record.put("id",lane.id());
          for(const auto & point:lane.polygon2d()){Tree vertex;vertex.put("x",point.x());vertex.put("y",point.y());ring.push_back({"",vertex});}
          record.add_child("polygon",ring);records.push_back({"",record});}
        packet.add_child(key,records);};
      lanes("current_lanes",get_current_lanes());lanes("target_lanes",get_target_lanes());
      const auto & vehicle=getCommonParam().vehicle_info;
#define SNAP_V(field) packet.put("vehicle." #field,vehicle.field)
      SNAP_V(wheel_radius_m);SNAP_V(wheel_width_m);SNAP_V(wheel_base_m);SNAP_V(wheel_tread_m);
      SNAP_V(front_overhang_m);SNAP_V(rear_overhang_m);SNAP_V(left_overhang_m);SNAP_V(right_overhang_m);
      SNAP_V(vehicle_height_m);SNAP_V(max_steer_angle_rad);
      SNAP_V(min_longitudinal_offset_m);SNAP_V(max_longitudinal_offset_m);
      SNAP_V(min_lateral_offset_m);SNAP_V(max_lateral_offset_m);
#undef SNAP_V
      const auto rss=[&](const char * key,const auto & value){Tree r;
#define SNAP_R(field) r.put(#field,value.field)
        SNAP_R(extended_polygon_policy);SNAP_R(rear_vehicle_reaction_time);SNAP_R(rear_vehicle_safety_time_margin);
        SNAP_R(lateral_distance_max_threshold);SNAP_R(longitudinal_distance_min_threshold);
        SNAP_R(longitudinal_velocity_delta_time);SNAP_R(front_vehicle_deceleration);SNAP_R(rear_vehicle_deceleration);
#undef SNAP_R
        packet.add_child(key,r);};
      rss("rss_execution",lane_change_parameters_->safety.rss_params);
      rss("rss_prepare",lane_change_parameters_->safety.rss_params_for_prepare);
      rss("rss_cancel",lane_change_parameters_->safety.rss_params_for_abort);
      packet.put("limits.max_vel",getCommonParam().max_vel);
      packet.put("limits.common_max_acc",getCommonParam().max_acc);
      packet.put("limits.common_min_acc",getCommonParam().min_acc);
      packet.put("limits.lc_max_acc",lane_change_parameters_->trajectory.max_longitudinal_acc);
      packet.put("limits.lc_min_acc",lane_change_parameters_->trajectory.min_longitudinal_acc);
      packet.put("limits.prediction_resolution",lane_change_parameters_->safety.collision_check.prediction_time_resolution);
      packet.put("limits.th_yaw_diff",lane_change_parameters_->safety.collision_check.th_yaw_diff);
      packet.put("limits.ego_nearest_dist_threshold",getCommonParam().ego_nearest_dist_threshold);
      packet.put("limits.ego_nearest_yaw_threshold",getCommonParam().ego_nearest_yaw_threshold);
      packet.put("limits.min_prepare_duration",lane_change_parameters_->trajectory.min_prepare_duration);
      Tree acceleration_map;
      for(const auto & entry:common_data_ptr_->acceleration_limits.map()){
        Tree row;row.put("speed",entry.first);row.put("acceleration",entry.second);acceleration_map.push_back({"",row});}
      packet.add_child("acceleration_map",acceleration_map);
      for(const auto name:{"min_acc","min_jerk","max_jerk"}){
        double value;if(ros::param::getCached(std::string("/planning/scenario_planning/velocity_smoother/normal/")+name,value))
          packet.put(std::string("limits.smoother_")+name,value);}
      // Exact native converter inputs, not a full Frenet optimizer recreation.
      packet.put("frenet.capture_scope","prediction_converter_inputs_only");
      const auto & native=candidate.frenet_path;
      packet.add_child("frenet.prepare",lc::snapshot_ros_message(native.prepare));
      packet.put("frenet.prepare_duration",native.prepare_metric.duration);
      packet.put("frenet.prepare_velocity",native.prepare_metric.velocity);
      packet.put("frenet.prepare_acceleration",native.prepare_metric.actual_lon_accel);
      packet.put("frenet.prepare_sampled_acceleration",native.prepare_metric.sampled_lon_accel);
      Tree native_samples;
      const auto & trajectory=native.lane_changing;
      if(trajectory.times.size()!=trajectory.poses.size() || trajectory.times.size()!=trajectory.longitudinal_velocities.size())
        packet.put("frenet.supported",false);
      else {packet.put("frenet.supported",true);
        for(size_t i=0;i<trajectory.times.size();++i){Tree sample;sample.put("time",trajectory.times[i]);
          sample.put("velocity",trajectory.longitudinal_velocities[i]);sample.add_child("pose",lc::snapshot_ros_message(trajectory.poses[i]));
          native_samples.push_back({"",sample});}}
      packet.add_child("frenet.samples",native_samples);
      std::ostringstream encoded;boost::property_tree::write_json(encoded,packet,false);
      std_msgs::String snapshot;snapshot.data=encoded.str();strictSnapshotPublisher().publish(snapshot);
    }
  } catch(const std::exception & error){
    ROS_WARN_THROTTLE(1.0,"LC safety snapshot capture failed: %s",error.what());
  }
  catch(...){ROS_WARN_THROTTLE(1.0,"LC safety snapshot capture failed: unknown exception");}
}

void NormalLaneChange::enforce_strict_maneuver_safety(BehaviorModuleOutput & output)
{
  bool enabled = false;
  ros::param::getCached("/planning/lc_strict_maneuver_safety", enabled);
  // generateOutput is reached only by interface::plan(), after SceneModuleCore
  // dispatches approved execution. Waiting approval has its separate output
  // path. LaneChangeBase::is_activated_ is not maintained by that dispatcher;
  // the real generateOutput early-return behavior is covered by integration tests.
  if (!enabled) return;
  auto candidate = isAbortState() && abort_path_ ? *abort_path_ : status_.lane_change_path;
  if (isAbortState() && abort_path_)
    candidate = utils::lane_change::abort_prediction_metadata(candidate);
  bool bounded_output_mode=false,partial_grid_enabled=false;
  ros::param::getCached("/planning/lc_continuous_bounded_output",bounded_output_mode);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial_grid_enabled);
  const bool bounded_owner=utils::lane_change::owns_bounded_longitudinal_output(
    bounded_output_mode,partial_grid_enabled,isAbortState(),isStopState());
  bool adaptive_positive=false;
  ros::param::getCached("/planning/lc_adaptive_positive_grid",adaptive_positive);
  bool rss_positive=false;
  ros::param::getCached("/planning/lc_rss_positive_grid",rss_positive);
  rss_positive=rss_positive&&adaptive_positive&&bounded_owner;
  bool anchor_invariant=false;
  ros::param::getCached("/planning/lc_anchor_invariant_curvature",anchor_invariant);
  anchor_invariant=anchor_invariant&&bounded_owner;
  bool post_end_tail=false;
  ros::param::getCached("/planning/lc_post_end_bounded_tail",post_end_tail);
  post_end_tail=post_end_tail&&bounded_owner&&getModuleType()==LaneChangeModuleType::NORMAL;
  std::vector<double> attempted_targets;
  std::string original_reason="invalid_active_candidate";
  const bool original_safe=status_.is_valid_path &&
    check_strict_maneuver_safety(candidate,output.path,original_reason,true);
  if(utils::lane_change::may_publish_original_longitudinal(original_safe,bounded_owner))return;
  // No timed hold and no metadata rewrite: once this opt-in owns normal LC
  // longitudinal output, every cycle must publish a freshly checked bounded
  // proposal. A safe old approval is not permission to bypass that owner.
  std::string cancel_reason="current_lane_cancel_ineligible";
  utils::lane_change::ValidatedCancelOrder cancel_order;
  const auto try_validated_cancel=[&](bool after_grid) {
    if(!cancel_order.claim(bounded_owner,original_safe,after_grid))return false;
    if(status_.is_valid_path && status_.lane_change_path.path.points.size()>=2 &&
     !isAbortState() && isCancelEnabled() &&
     isEgoOnPreparePhase() && isAbleToReturnCurrentLane()) {
    try {
      // FAILURE/Cancel makes the manager discard this module's result and
      // publish its upstream result. Check exactly that immutable input, not
      // a modified fallback that the manager would subsequently discard.
      if(check_strict_maneuver_safety(candidate,prev_module_output_.path,cancel_reason,true,true)) {
        publishStrictSafetySnapshot("checked_current_lane_cancel",candidate,output.path,
          &prev_module_output_.path,ros::Time::now(),original_reason,cancel_reason,"",&attempted_targets);
        output=prev_module_output_;
        toCancelState();
        ROS_WARN_THROTTLE(1.0,"Strict LC selected validated current-lane cancellation");
        return true;
      }
    } catch(const std::exception & error){cancel_reason=error.what();}
    }
    return false;
  };
  if(try_validated_cancel(false))return;
  std::string brake_reason = "braking_not_constructed";
  try {
    if(!status_.is_valid_path)throw std::logic_error("invalid_active_candidate");
    double minimum_acceleration = std::max(
      getCommonParam().min_acc, lane_change_parameters_->trajectory.min_longitudinal_acc);
    double min_jerk = 0.0;
    double max_jerk = 0.0;
    double smoother_acceleration = 0.0;
    if (!ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc", smoother_acceleration) ||
        !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk", min_jerk) ||
        !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/max_jerk", max_jerk))
      throw std::logic_error("missing_braking_limits");
    minimum_acceleration = std::max(minimum_acceleration, smoother_acceleration);
    if (!common_data_ptr_->current_acceleration) throw std::logic_error("missing_braking_acceleration");
    bool rising_interpolation=false,partial_acceleration=false;
    ros::param::getCached("/planning/interpolate_rising_velocity",rising_interpolation);
    ros::param::getCached("/planning/lc_partial_acceleration_candidates",partial_acceleration);
    const auto anchored = utils::lane_change::anchor_maneuver_at_ego(output.path,
      planner_data_->findEgoSegmentIndex(output.path.points),getEgoPosition(),rising_interpolation);
    std::optional<PathWithLaneId> bounded_braking_caps;
    if(partial_acceleration && !isAbortState() && !isStopState() &&
        common_data_ptr_->acceleration_limits.enabled()) {
      // Same immutable geometry and fresh hard speed envelope. These are
      // additional proposals, never permission to bypass either safety model.
      // A failed grid construction must not suppress the full-brake fallback.
      try {
        namespace lc=utils::lane_change;
        const auto grid_started=ros::WallTime::now();
        if(!planner_data_->dynamic_object)throw std::logic_error("missing_grid_objects");
        const auto & raw=*planner_data_->dynamic_object;
        const double age=(planner_data_->self_odometry->header.stamp-raw.header.stamp).toSec();
        if(!std::isfinite(age)||age<-.010||age>.5)throw std::logic_error("stale_grid_objects");
        double horizon=3.0;
        for(const auto & object:raw.objects) {
          const auto observed=lc::observed_maneuver_object(object,
            lane_change_parameters_->safety.collision_check.prediction_time_resolution);
          if(observed.predicted_paths.empty())throw std::logic_error("missing_grid_prediction");
          for(const auto & prediction:observed.predicted_paths) {
            if(prediction.path.empty())throw std::logic_error("empty_grid_prediction");
            horizon=std::min(horizon,prediction.path.back().time-std::max(0.0,age));
          }
        }
        if(horizon<2.5)throw std::logic_error("insufficient_grid_horizon");
        // Negative object age advances the ego prediction to the newer object
        // timestamp; cover that same additional prefix in proposal generation.
        horizon+=std::max(0.0,-age);
        const auto full_arc=lc::safety_arc_lengths(anchored.first);
        const auto & target_lanes=get_target_lanes();
        if(target_lanes.empty())throw std::logic_error("missing partial target lanes");
        const auto target_reference=getRouteHandler()->getCenterLinePath(
          target_lanes,0.0,lanelet::geometry::length3d(lanelet::LaneletSequence(target_lanes)));
        const auto & lateral_map=lane_change_parameters_->trajectory.lat_acc_map.base_max_acc;
        if(lateral_map.empty())throw std::logic_error("missing partial lateral limits");
        const auto end=autoware::motion_utils::findNearestIndex(
          anchored.first.points,candidate.info.lane_changing_end.position);
        if(autoware_utils::calc_distance2d(anchored.first.points[end].point.pose,
              candidate.info.lane_changing_end)>1.0)
          throw std::logic_error("partial LC endpoint not covered");
        const auto certificate=[&](){return currentExitCertificate(anchored.first,get_current_lanes(),get_target_lanes(),
          getCommonParam().vehicle_info,anchored.second,end,getModuleType()==LaneChangeModuleType::NORMAL);};
        const auto caps=certifiedEnvelope(certificate,anchored.first,
          common_data_ptr_->current_lanes_path,target_reference,prev_module_output_.path,
          anchored.second,end,getEgoPosition(),getCommonParam().max_vel,
          *std::min_element(lateral_map.begin(),lateral_map.end()),
          autoware::motion_utils::findNearestIndex(
            anchored.first.points,candidate.info.lane_changing_start.position),true,
          anchor_invariant?&output.path:nullptr,anchored.second,
          post_end_tail&&anchored.second>end?&candidate.info.lane_changing_end.position:nullptr);
        if(bounded_owner)bounded_braking_caps=lc::maneuver_braking_hard_caps(
          anchored.first,anchored.second,caps);
        std::vector<double> arc;
        for(size_t i=anchored.second;i<full_arc.size();++i) {
          arc.push_back(full_arc[i]-full_arc[anchored.second]);
        }
        const auto acceleration_limit=[&](double speed) {
          return common_data_ptr_->acceleration_limits.limit(speed,
            std::min(getCommonParam().max_acc,
              lane_change_parameters_->trajectory.max_longitudinal_acc));
        };
        const double active_max=acceleration_limit(getEgoVelocity());
        std::vector<bounded_speed_grid::Proposal> proposals;
        std::ostringstream rejected_proposals;
        const auto base_targets=lane_change::rss_positive_grid::augment(
          bounded_speed_grid::acceleration_targets(active_max,minimum_acceleration),active_max,rss_positive);
        double smallest_positive=std::numeric_limits<double>::infinity();
        for(double target:base_targets)if(target>0)smallest_positive=std::min(smallest_positive,target);
        bool coast_generated=false,smallest_positive_generated=false;
        std::set<double> generated_targets;
        const auto generate=[&](double target) {
          if(std::find(attempted_targets.begin(),attempted_targets.end(),target)!=attempted_targets.end())
            return generated_targets.count(target)!=0;
          if(attempted_targets.size()>=15)return false;
          attempted_targets.push_back(target);
          try {
            auto proposal=[&]() {
              try {
                return bounded_speed_grid::make(arc,caps,getEgoVelocity(),
                  common_data_ptr_->current_acceleration->accel.accel.linear.x,target,horizon,
                  minimum_acceleration,min_jerk,max_jerk,acceleration_limit);
              } catch(const std::logic_error &) {
                // Additional physically reachable family: release positive
                // acceleration before a uniform existing cap, never raise it.
                // Variable caps, stops and unsupported initial states fail
                // closed in the helper; every final safety check stays below.
                if(!(rss_positive&&adaptive_positive&&bounded_owner&&target>0))throw;
                return bounded_speed_grid::make_uniform_cap_coast(arc,caps,getEgoVelocity(),
                  common_data_ptr_->current_acceleration->accel.accel.linear.x,target,horizon,
                  min_jerk,max_jerk,acceleration_limit,[&]() {
                    return (ros::WallTime::now()-grid_started).toSec()*1000.0 <
                      lane_change_parameters_->time_limit;
                  });
              }
            }();
            // Rank the final sparse v(s), not its pre-resampling integration.
            proposal.progress=lc::sample_speed_profile(arc,proposal.speeds,horizon).first;
            proposals.push_back(std::move(proposal));
            generated_targets.insert(target);
            return true;
          } catch(const std::logic_error & error) {
            rejected_proposals<<" generation target="<<target<<": "<<error.what()<<";";
            return false;
          }
        };
        for(double target:base_targets) {
          const bool generated=generate(target);
          if(target==0)coast_generated=generated;
          if(target==smallest_positive)smallest_positive_generated=generated;
        }
        if(adaptive_positive&&bounded_owner) {
          const auto probes=bounded_speed_grid::probe_small_positive(
            coast_generated,!smallest_positive_generated,smallest_positive,generate,
            rss_positive?15-attempted_targets.size():6);
          if(!probes.empty())ROS_INFO_THROTTLE(1.0,
            "LC adaptive positive grid: probes=%zu total_targets=%zu smallest_base_positive=%.6f",
            probes.size(),attempted_targets.size(),smallest_positive);
        }
        std::stable_sort(proposals.begin(),proposals.end(),[](const auto & a,const auto & b) {
          return a.progress>b.progress;
        });
        auto model=candidate;
        model.type=lane_change::PathType::SpatialSpeedProfile;
        const double generation_ms=(ros::WallTime::now()-grid_started).toSec()*1000.0;
        size_t checked_proposals=0;
        for(const auto & speeds:proposals) {
          auto proposed=anchored.first;
          for(size_t i=0;i<speeds.speeds.size();++i)
            proposed.points[anchored.second+i].point.longitudinal_velocity_mps=speeds.speeds[i];
          std::string partial_reason;
          ++checked_proposals;
          if(!check_strict_maneuver_safety(model,proposed,partial_reason,true)) {
            rejected_proposals<<" validation progress="<<speeds.progress<<": "<<partial_reason<<";";
            continue;
          }
          publishStrictSafetySnapshot(bounded_owner?"checked_continuous_acceleration_selected":
            "checked_partial_acceleration_selected",candidate,
            output.path,&proposed,ros::Time::now(),original_reason,cancel_reason,partial_reason,&attempted_targets);
          output.path=std::move(proposed);  // Atomic acceptance after the full gate only.
          if(bounded_owner)remember_selected_output(model,output.path);
          ROS_INFO_THROTTLE(1.0,"Strict LC selected checked partial acceleration, progress=%.3f tail_factor=%.3f tail_attempts=%u",
            speeds.progress,speeds.tail_factor,speeds.tail_attempts);
          if(bounded_owner)ROS_INFO_THROTTLE(1.0,
            "Continuous LC bounded output: original_safe=%d generated=%zu checked=%zu generation_ms=%.3f total_ms=%.3f progress=%.3f",
            original_safe,proposals.size(),checked_proposals,generation_ms,
            (ros::WallTime::now()-grid_started).toSec()*1000.0,speeds.progress);
          return;
        }
        if(bounded_owner)ROS_WARN_THROTTLE(1.0,
          "Continuous LC bounded output unavailable: original_safe=%d generated=%zu checked=%zu generation_ms=%.3f total_ms=%.3f reasons=%s",
          original_safe,proposals.size(),checked_proposals,generation_ms,
          (ros::WallTime::now()-grid_started).toSec()*1000.0,rejected_proposals.str().c_str());
      } catch(const std::exception & error) {
        if(bounded_owner)ROS_WARN_THROTTLE(1.0,
          "Continuous LC bounded output construction failed: original_safe=%d reason=%s",
          original_safe,error.what());
        ROS_DEBUG_THROTTLE(1.0,"Strict LC partial candidates unavailable: %s",error.what());
      }
    }
    // Only bounded ownership defers cancellation until all bounded proposals
    // have failed. The legacy non-owner already tried it before grid search.
    if(try_validated_cancel(true))return;
    // The bounded owner already separates dynamic v(s) from physical limits.
    // Preserve that same contract for full braking, including all fresh stops.
    // If its fresh envelope could not be built, do not fall back to stale caps.
    if(bounded_owner&&!bounded_braking_caps)
      throw std::logic_error("missing fresh braking hard envelope");
    const auto proposal = utils::lane_change::make_maneuver_braking_proposal(
      bounded_owner?*bounded_braking_caps:anchored.first, anchored.second, getEgoVelocity(),
      common_data_ptr_->current_acceleration->accel.accel.linear.x, minimum_acceleration, min_jerk,max_jerk);
    auto braking_model=candidate;
    // This separately generated jerk-limited output is an actual v(s), not
    // the old Frenet/legacy longitudinal plan. Keep its checked XY unchanged.
    braking_model.type=lane_change::PathType::SpatialSpeedProfile;
    if (utils::lane_change::accept_checked_maneuver(output.path, proposal, [&](const auto & path) {
          const bool accepted=check_strict_maneuver_safety(braking_model,path,brake_reason,true);
          if(accepted)publishStrictSafetySnapshot("checked_braking_selected",candidate,
            output.path,&path,ros::Time::now(),original_reason,cancel_reason,brake_reason,&attempted_targets);
          return accepted;
        })) {
      // Per-cycle checked output override only. Persisting these stop points
      // into the old Spatial/Frenet plan would corrupt its native prediction
      // contract next cycle. Every active output is gated again on fresh data.
      ROS_WARN_THROTTLE(1.0, "Strict LC selected checked braking output");
      if(bounded_owner)remember_selected_output(braking_model,output.path);
      return;
    }
  } catch (const std::exception & e) { brake_reason = e.what(); }
  // Missing/invalid construction inputs may throw before the normal deferred
  // point. Preserve one checked cancellation opportunity, never duplicate it.
  if(try_validated_cancel(true))return;

  // No safe maneuver was found. Zero speed is a minimum-risk STOP REQUEST,
  // not a claim of lateral collision avoidance or achievable instantaneous
  // braking. The harness consumes this explicit failure and ends the trial.
  // Publish the small failure signal BEFORE diagnostic allocation/serialization.
  // Snapshot the original path inside the guarded block, then always zero it.
  const auto failure_stamp=ros::Time::now();
  auto & failure_pub = strictFailurePublisher();
  std::ostringstream uuid;
  for (const auto byte : common_data_ptr_->route_handler_ptr->getRouteUuid().uuid)
    uuid << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
  std::ostringstream event;
  event << std::setprecision(17) << "{\"stamp\":" << failure_stamp.toSec()
        << ",\"module\":" << utils::lane_change::safety_json_string(getModuleTypeStr())
        << ",\"reason\":\"no_safe_maneuver\",\"continue_reason\":"
        << utils::lane_change::safety_json_string(original_reason)
        << ",\"cancel_reason\":" << utils::lane_change::safety_json_string(cancel_reason)
        << ",\"brake_reason\":" << utils::lane_change::safety_json_string(brake_reason) << ","
        << "\"route_uuid\":\"" << uuid.str() << "\",\"ego_x\":";
  if(std::isfinite(getEgoPosition().x))event<<getEgoPosition().x;else event<<"null";
  event<<",\"ego_y\":";
  if(std::isfinite(getEgoPosition().y))event<<getEgoPosition().y;else event<<"null";
  if(planner_data_->self_odometry)
    event<<",\"odom_header_stamp\":"<<planner_data_->self_odometry->header.stamp.toSec()
      <<",\"odom_frame\":"<<utils::lane_change::safety_json_string(planner_data_->self_odometry->header.frame_id);
  if(planner_data_->dynamic_object)
    event<<",\"object_header_stamp\":"<<planner_data_->dynamic_object->header.stamp.toSec()
      <<",\"object_frame\":"<<utils::lane_change::safety_json_string(planner_data_->dynamic_object->header.frame_id);
  event<<"}";
  std_msgs::String message;
  message.data = event.str();
  failure_pub.publish(message);
  publishStrictSafetySnapshot("no_safe_maneuver",candidate,output.path,nullptr,
    failure_stamp,original_reason,cancel_reason,brake_reason,&attempted_targets);
  for (auto & point : output.path.points) point.point.longitudinal_velocity_mps = 0.0;
  ROS_ERROR_THROTTLE(0.5, "LC no safe maneuver: continue=%s braking=%s", original_reason.c_str(), brake_reason.c_str());
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

std::optional<utils::lane_change::SelectedOutputIdentity>
NormalLaneChange::selected_output_identity() const
{
  bool enabled=false,grid=false,strict=false,diagnostic=false;
  ros::param::getCached("/planning/lc_continuous_bounded_output",enabled);
  ros::param::getCached("/planning/lc_partial_acceleration_candidates",grid);
  ros::param::getCached("/planning/lc_strict_maneuver_safety",strict);
  ros::param::getCached("/planning/lc_measured_motion_diagnostic_only",diagnostic);
  if(!strict || !utils::lane_change::owns_bounded_longitudinal_output(
      enabled,grid,isAbortState(),isStopState()) || !status_.is_valid_path ||
      !planner_data_->self_odometry || !planner_data_->dynamic_object ||
      !common_data_ptr_->current_acceleration || !getRouteHandler())return std::nullopt;
  utils::lane_change::SelectedOutputIdentity key;
  key.cycle=selected_output_cycle_;
  key.odom_stamp=planner_data_->self_odometry->header.stamp.toNSec();
  key.object_stamp=planner_data_->dynamic_object->header.stamp.toNSec();
  key.route_stamp=getRouteHeader().stamp.toNSec();
  key.acceleration_stamp=common_data_ptr_->current_acceleration->header.stamp.toNSec();
  key.odom=reinterpret_cast<uintptr_t>(planner_data_->self_odometry.get());
  key.objects=reinterpret_cast<uintptr_t>(planner_data_->dynamic_object.get());
  key.acceleration=reinterpret_cast<uintptr_t>(common_data_ptr_->current_acceleration.get());
  key.route=reinterpret_cast<uintptr_t>(getRouteHandler().get());
  const auto uuid=getRouteHandler()->getRouteUuid();
  key.route_uuid.assign(uuid.uuid.begin(),uuid.uuid.end());
  key.measured_acceleration=common_data_ptr_->current_acceleration->accel.accel.linear.x;
  key.diagnostic_only=diagnostic;
  if(!key.matches(key))return std::nullopt;
  return key;
}

void NormalLaneChange::remember_selected_output(
  const LaneChangePath & model,const PathWithLaneId & path)
{
  const auto identity=selected_output_identity();
  if(!identity)return;
  auto selected=model;
  selected.path=path;
  selected.type=lane_change::PathType::SpatialSpeedProfile;
  selected_output_model_=std::move(selected);
  selected_output_identity_=identity;
}

PathSafetyStatus NormalLaneChange::isApprovedPathSafe() const
{
  bool fresh_commit_input=false;
  ros::param::getCached("/planning/lc_fresh_commit_input",fresh_commit_input);
  if(fresh_commit_input && getModuleType()==LaneChangeModuleType::NORMAL &&
     (!planner_data_ || !planner_data_->dynamic_object)) return {false,false};
  const auto identity=selected_output_identity();
  const bool use_selected=selected_output_model_ && selected_output_identity_ && identity &&
    selected_output_identity_->matches(*identity);
  const auto & path = use_selected ? *selected_output_model_ : status_.lane_change_path;
  const auto & current_lanes = get_current_lanes();
  const auto & target_lanes = get_target_lanes();

  if (current_lanes.empty() || target_lanes.empty()) {
    if(use_selected)return {false,false};
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

  if(use_selected) {
    std::string reason;
    if(!check_strict_maneuver_safety(path,path.path,reason,true))return {false,false};
  }

  const auto decel_sampling_num =
    static_cast<size_t>(lane_change_parameters_->cancel.deceleration_sampling_num);
  const auto safety_status = [&]() -> PathSafetyStatus {
    if(!use_selected) {
      const auto predictions=utils::lane_change::convert_to_predicted_paths(
        common_data_ptr_,path,decel_sampling_num,true);
      return isLaneChangePathSafe(path,predictions,target_objects,
        lane_change_parameters_->safety.rss_params_for_abort,debug_data,true);
    }
    try {
      namespace lc=utils::lane_change;
      bool rising=false;
      ros::param::getCached("/planning/interpolate_rising_velocity",rising);
      const auto anchored=lc::anchor_maneuver_at_ego(path.path,
        planner_data_->findEgoSegmentIndex(path.path.points),getEgoPosition(),rising);
      const auto end=std::max(anchored.second,autoware::motion_utils::findNearestIndex(
        anchored.first.points,path.info.lane_changing_end.position));
      const auto prepare=autoware::motion_utils::findNearestIndex(
        anchored.first.points,path.info.lane_changing_start.position);
      double horizon=3.0;
      const double age=std::max(0.0,(planner_data_->self_odometry->header.stamp-
        planner_data_->dynamic_object->header.stamp).toSec());
      for(const auto & object:planner_data_->dynamic_object->objects) {
        const auto observed=lc::observed_maneuver_object(object,
          lane_change_parameters_->safety.collision_check.prediction_time_resolution);
        for(const auto & prediction:observed.predicted_paths) {
          if(prediction.path.empty())throw std::logic_error("missing selected prediction");
          horizon=std::min(horizon,prediction.path.back().time-age);
        }
      }
      const auto predictions=lc::selected_output_predictions(anchored.first,anchored.second,
        end,prepare,getEgoVelocity(),identity->measured_acceleration,getCommonParam().max_vel,
        lane_change_parameters_->safety.collision_check.prediction_time_resolution,horizon,
        path.info.duration.prepare<=1e-6,identity->diagnostic_only);
      auto selected=path;
      selected.path=anchored.first;
      for(const auto & hypothesis:predictions) {
        selected.info.duration.prepare=hypothesis.prepare_time;
        const auto result=isLaneChangePathSafe(selected,{hypothesis.prediction},target_objects,
          lane_change_parameters_->safety.rss_params_for_abort,debug_data,true);
        if(!result.is_safe)return result;
      }
      return {true,true};
    } catch(const std::exception & error) {
      ROS_WARN_THROTTLE(1.0,"LC selected postProcess prediction invalid: %s",error.what());
      return {false,false};
    }
  }();
  if(use_selected)ROS_INFO_THROTTLE(1.0,
    "LC postProcess checked selected output: cycle=%llu safe=%d diagnostic=%d",
    static_cast<unsigned long long>(selected_output_cycle_),safety_status.is_safe,
    identity->diagnostic_only);
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

  return safety_status;
}

PathSafetyStatus NormalLaneChange::evaluateApprovedPathWithUnsafeHysteresis(
  PathSafetyStatus approved_path_safety_status)
{
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
    get_abort_idx_and_distance(lane_change_parameters_->cancel.delta_time);
  const auto [abort_return_idx, abort_return_dist] = get_abort_idx_and_distance(
    lane_change_parameters_->cancel.delta_time + lane_change_parameters_->cancel.duration);

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
    shift_line.end_shift_length, abort_start_dist, current_velocity);
  path_shifter.setVelocity(current_velocity);
  const auto lateral_acc_range =
    lane_change_parameters_->trajectory.lat_acc_map.find(current_velocity);
  const double & max_lateral_acc = lateral_acc_range.second;
  path_shifter.setLateralAccelerationLimit(max_lateral_acc);

  if (lateral_jerk > lane_change_parameters_->cancel.max_lateral_jerk) {
    RCLCPP_ERROR(logger_, "Aborting jerk is too strong. lateral_jerk = %f", lateral_jerk);
    return false;
  }

  ShiftedPath shifted_path;
  // offset front side
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(logger_, "failed to generate abort shifted path.");
    bool strict_generation = false;
    ros::param::getCached("/planning/lc_strict_maneuver_safety", strict_generation);
    if (strict_generation) return false;
  }

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
  abort_path.shifted_path = shifted_path;
  abort_path.info.shift_line = shift_line;

  {
    PathWithLaneId aborting_path;
    aborting_path.points.insert(
      aborting_path.points.begin(), shifted_path.path.points.begin(),
      shifted_path.path.points.begin() + static_cast<int>(abort_return_idx));

    if (!reference_lane_segment.points.empty()) {
      abort_path.path = utils::combinePath(aborting_path, reference_lane_segment);
    } else {
      abort_path.path = aborting_path;
    }
  }

  bool strict = false;
  ros::param::getCached("/planning/lc_strict_maneuver_safety", strict);
  if (strict) {
    // The physical abort endpoint is the shifted return point, not the old
    // approved lane-change endpoint inherited by the local copy.
    abort_path.info.lane_changing_end = shifted_path.path.points.at(abort_return_idx).point.pose;
    auto final_output = prev_module_output_;
    final_output.path = abort_path.path;
    insert_stop_point(get_current_lanes(), final_output.path);
    extendOutputDrivableArea(final_output);
    std::string reason;
    const auto abort_model=utils::lane_change::abort_prediction_metadata(abort_path);
    if (!check_strict_maneuver_safety(abort_model, final_output.path, reason, true)) {
      ROS_WARN_THROTTLE(1.0, "Strict LC abort rejected: %s", reason.c_str());
      return false;
    }
    abort_path.path = final_output.path;
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
      if (objects_idx.find(idx) == objects_idx.end()) {
        current_colliding_objects.push_back(object);
        objects_idx.insert(idx);
      }
    }

    if (current_colliding_objects.empty()) {
      return false;
    }

    std::move(
      current_colliding_objects.begin(), current_colliding_objects.end(),
      std::back_inserter(colliding_objects));

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

  if (lane_change_path.type == lane_change::PathType::SpatialSpeedProfile) {
    // These are simultaneous planned/actual-motion hypotheses, not alternative
    // deceleration maneuvers. EVERY hypothesis must pass for both front/rear gaps.
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

  if (
    const auto found_colliding_objects_opt = check_for_colliding_objects(
      {ego_predicted_paths.front()}, collision_check_objects.trailing)) {
    const auto is_moving_object =
      ranges::any_of(*found_colliding_objects_opt, check_for_moving_objects);
    return {!is_safe, is_moving_object};
  }

  if (check_for_colliding_objects(ego_predicted_paths, collision_check_objects.leading)) {
    return {!is_safe, !is_moving_object_behind_ego};
  }

  return {is_safe, !is_moving_object_behind_ego};
}

bool NormalLaneChange::is_colliding(
  const LaneChangePath & lane_change_path, const ExtendedPredictedObject & obj,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const RSSparams & rss_param,
  CollisionCheckDebugMap & debug_data, const bool is_approved) const
{
  return is_colliding(lane_change_path,obj,ego_predicted_path,rss_param,debug_data,is_approved,nullptr);
}
bool NormalLaneChange::is_colliding(
  const LaneChangePath & lane_change_path, const ExtendedPredictedObject & obj,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path, const RSSparams & rss_param,
  CollisionCheckDebugMap & debug_data, const bool is_approved,
  const utils::path_safety_checker::PreparedPathProjection * projection) const
{
  if (lane_change_path.path.points.empty() || ego_predicted_path.empty()) return false;

  // Restore the upstream timestamp selector without changing RSS geometry:
  // each original object sample uses prepare RSS strictly before the prepare
  // endpoint and lane-changing RSS at/after it. The core's conservative
  // collision handling is retained (upstream lane-polygon filtering is not
  // introduced here).
  constexpr double hysteresis_factor{1.0};
  const auto yaw_threshold = common_data_ptr_->lc_param_ptr->safety.collision_check.th_yaw_diff;
  const auto & vehicle_info = common_data_ptr_->bpp_param_ptr->vehicle_info;
  const auto & parked_threshold = lane_change_parameters_->safety.th_stopped_object_velocity;
  const auto & lane_changing_params =
    (!is_approved && obj.initial_twist.linear.x <= parked_threshold)
      ? lane_change_parameters_->safety.rss_params_for_parked
      : rss_param;
  std::string phase_reason;
  const bool safe = utils::lane_change::check_rss_phases(
    obj, ego_predicted_path, lane_change_path.info.duration.prepare,
    [&](const auto & phase_object, const bool prepare, const auto & original_prediction) {
      const bool phase_safe = utils::path_safety_checker::check_safety_with_rss(
        lane_change_path.path, ego_predicted_path, vehicle_info, {phase_object}, debug_data,
        prepare ? common_data_ptr_->lc_param_ptr->safety.rss_params_for_prepare
                : lane_changing_params,
        true, hysteresis_factor, yaw_threshold,projection);
      if (!phase_safe) {
        // Keep the failing geometry/reason, plus the complete original object
        // hypothesis for replay rather than only its partitioned phase.
        debug_data[obj.uuid.uuid].object_predicted_path = original_prediction.path;
      }
      return phase_safe;
    }, phase_reason);
  if (!safe && phase_reason.find("rss_phase_") == 0) {
    auto & debug = debug_data[obj.uuid.uuid];
    debug.is_safe = false;
    debug.unsafe_reason = phase_reason;
    debug.ego_predicted_path = ego_predicted_path;
    debug.expected_object_pose = obj.initial_pose;
    debug.extended_object_polygon = obj.initial_polygon;
  }
  return !safe;
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
  const auto & pose = common_data_ptr_->get_ego_pose();
  const double yaw_threshold = getCommonParam().ego_nearest_yaw_threshold;
  if (utils::lane_change::contains_direction_compatible_pose(
        get_current_lanes(), pose, yaw_threshold) ||
      utils::lane_change::contains_direction_compatible_pose(
        get_target_lanes(), pose, yaw_threshold)) {
    return true;
  }
  // Preserve the existing nearest-lane fallback outside these polygons. Do not
  // change route-wide tie breaking or expand the intended corridor.
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

  // Check lateral deviation from planned path
  const auto lateral_deviation =
    autoware::motion_utils::calcLateralOffset(path_points, current_pose.position);

  const auto dynamic_lateral_threshold = [this]() -> double {
    const double ego_velocity = getEgoVelocity();

    const std::vector<double> velocity_points =
      lane_change_parameters_->path_miss_velocity_points;  // m/s
    const std::vector<double> lateral_thresholds =
      lane_change_parameters_->path_miss_lateral_thresholds;

    const double lateral_threshold = autoware::interpolation::lerp(
      velocity_points, lateral_thresholds,
      std::clamp(ego_velocity, velocity_points.front(), velocity_points.back()));

    return lateral_threshold;
  }();

  // Check if lateral deviation exceeds threshold
  if (std::abs(lateral_deviation) > dynamic_lateral_threshold) {
    RCLCPP_WARN(
      logger_, "Path miss detected: lateral deviation %.2f > threshold %.2f",
      std::abs(lateral_deviation), dynamic_lateral_threshold);
    return true;
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
