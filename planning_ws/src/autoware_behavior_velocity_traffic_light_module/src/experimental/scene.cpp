// Copyright 2025 Tier IV, Inc.
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

#include "scene.hpp"
#include <autoware/behavior_velocity_traffic_light_module/signal_decision_video.hpp>

#include "../utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
TrafficLightModule::TrafficLightModule(
  const lanelet::Id module_id,
  const lanelet::TrafficLight & traffic_light_reg_elem,  //
  lanelet::ConstLanelet lane,                            //
  const lanelet::ConstLineString3d & initial_stop_line,  //
  const bool is_turn_lane, const bool has_static_arrow, const PlannerParam & planner_param,
  ros::NodeHandle & node, const std::string logger, const std::shared_ptr<TrafficLightRos1Clock> clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, node, time_keeper, planning_factor_interface),
  logger_(logger), clock_(clock),
  yellow_transition_state_(YellowState::kNotYellow),
  looking_tl_state_{},
  traffic_signal_stamp_(std::nullopt),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  stop_line_(initial_stop_line),
  is_turn_lane_(is_turn_lane),
  has_static_arrow_(has_static_arrow),
  state_(State::APPROACH),
  debug_data_(),
  is_prev_state_stop_(false)
{
  planner_param_ = planner_param;
  signal_compliance_guard_.setRequiredGreenObservations(planner_param_.signal_compliance_required_green_observations);
  signal_compliance_guard_.setUnknownGraceSec(planner_param_.signal_compliance_unknown_grace_sec);
  prev_looking_tl_state_.traffic_light_group_id = 0;
}

void TrafficLightModule::updateDecisionLane(
  const lanelet::ConstLanelet & lane, const bool is_turn_lane)
{
  lane_ = lane;
  is_turn_lane_ = is_turn_lane;
}

bool TrafficLightModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  first_stop_point_s_ = path.length();

  const auto & self_pose = planner_data.current_odometry;
  const bool pending_stop = stop_requested_on_approach_ || decision_line_latch_.pendingStop();
  if (pending_stop) state_ = State::APPROACH;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(path, self_pose->pose.position);
  std::optional<double> official_crossing_distance;
  std::optional<double> virtual_front_distance;
  const auto * official_line = signal_compliance::findSignalJudgingLine(
    planner_param_.signal_judging_lines, traffic_light_reg_elem_.id());
  signal_compliance::JudgingPoint official_front;
  if (planner_param_.enable_signal_compliance_guard && planner_param_.use_official_judging_lines && official_line) {
    const double yaw = tf2::getYaw(self_pose->pose.orientation);
    official_front = {
      self_pose->pose.position.x + planner_param_.signal_compliance_front_axle_offset_m * std::cos(yaw),
      self_pose->pose.position.y + planner_param_.signal_compliance_front_axle_offset_m * std::sin(yaw)};
    official_crossing_distance = signal_compliance::finiteJudgingLineDistance(*official_line, official_front);
    if (planner_param_.enable_yellow_passthrough) {
      // Keep terminal-unreadability geometry independent of a cropped path's
      // synthetic current-ego stop target. Use the configured approach, not yaw.
      signal_compliance::SignalJudgingLine virtual_line;
      virtual_line.regulatory_id = traffic_light_reg_elem_.id();
      virtual_line.approach_direction = official_line->approach_direction;
      for (const auto & point : stop_line_) {
        virtual_line.points.push_back({point.x(), point.y()});
      }
      virtual_front_distance = signal_compliance::finiteJudgingLineDistance(virtual_line, official_front);
    }
    if (!pending_stop && planner_param_.enable_yellow_passthrough && official_crossing_distance && *official_crossing_distance < 0.0) {
      state_ = State::GO_OUT;
      seen_signal_on_approach_ = false;
      stop_signal_received_time_ptr_.reset();
      is_prev_state_stop_ = false;
      setSafe(true);
      return true;
    }
  }
  decision_line_official_distance_ = official_crossing_distance;

  // Calculate stop point
  auto stop_point_s = calcStopPoint(
    path, left_bound, right_bound, stop_line_,
    planner_param_.stop_margin + planner_data.vehicle_info_.max_longitudinal_offset_m);
  const bool genuine_path_stop_point = stop_point_s.has_value();
  if (!stop_point_s && planner_param_.enable_signal_compliance_guard &&
      (pending_stop || (planner_param_.use_official_judging_lines &&
       planner_param_.enable_yellow_passthrough && official_crossing_distance &&
       *official_crossing_distance >= 0.0))) {
    stop_point_s = ego_s;
  }

  if (!stop_point_s) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_,
      "Failed to calculate stop point for regulatory element id " << traffic_light_reg_elem_.id());
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  // Calculate dist to stop point
  const auto signed_arc_length_to_stop_point = *stop_point_s - ego_s;
  setDistance(signed_arc_length_to_stop_point);

  const auto front_axle_distance = signal_compliance::frontAxleDistanceFromBumperStop(
    signed_arc_length_to_stop_point, planner_param_.stop_margin,
    planner_data.vehicle_info_.max_longitudinal_offset_m,
    planner_param_.signal_compliance_front_axle_offset_m);
  // Original-MGeo/front-axle geometry is only for GO_OUT crossing. Yellow uses
  // the earlier Lanelet bumper stopping target without requiring original ETA.
  const auto crossing_distance = planner_param_.use_official_judging_lines && planner_param_.enable_yellow_passthrough ?
    official_crossing_distance : front_axle_distance;
  if (planner_param_.enable_signal_compliance_guard && !front_axle_distance) {
    setSafe(false);
    path = insertStopVelocity(path, *stop_point_s, planner_data);
    is_prev_state_stop_ = true;
    return true;
  }

  // Check state
  if (state_ == State::APPROACH) {
    // Move to go out state if ego vehicle over deadline.
    constexpr double signed_deadline_length = -2.0;
    if (!pending_stop && (planner_param_.enable_signal_compliance_guard ? (crossing_distance && *crossing_distance < 0.0) :
        signed_arc_length_to_stop_point < signed_deadline_length)) {
      RCLCPP_DEBUG(logger_, "APPROACH -> GO_OUT");
      state_ = State::GO_OUT;
      seen_signal_on_approach_ = false;
      stop_signal_received_time_ptr_.reset();
      if (planner_param_.enable_signal_compliance_guard) {
        setSafe(true);
        is_prev_state_stop_ = false;
        RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 200,
          "[signal_compliance] reg=%ld state=GO_OUT front_line_m=%.3f decision=clear_crossed_line",
          traffic_light_reg_elem_.id(), *crossing_distance);
      }
      return true;
    }

    first_stop_point_s_ = stop_point_s;

    // Check if stop is coming.
    const bool is_stop_signal = isStopSignal(
      planner_data, signed_arc_length_to_stop_point, virtual_front_distance, genuine_path_stop_point);
    stop_requested_on_approach_ = is_stop_signal;

    // Update stop signal received time
    if (!is_stop_signal) {
      stop_signal_received_time_ptr_.reset();
    } else if (!stop_signal_received_time_ptr_) {
      stop_signal_received_time_ptr_ = std::make_unique<Time>(clock_->now());
    }

    // Check hysteresis
    const double time_diff =
      stop_signal_received_time_ptr_
        ? std::max((clock_->now() - *stop_signal_received_time_ptr_).toSec(), 0.0)
        : 0.0;
    bool to_be_stopped = is_stop_signal && (planner_param_.enable_signal_compliance_guard ||
      (!planner_param_.enable_yellow_passthrough && isTrafficSignalYellow()) ||
      is_prev_state_stop_ || time_diff > planner_param_.stop_time_hysteresis);

    debug_data_.is_remaining_time_used = false;
    if (planner_param_.v2i_use_remaining_time) {
      const bool will_traffic_light_turn_red_before_reaching_stop_line =
        willTrafficLightTurnRedBeforeReachingStopLine(
          signed_arc_length_to_stop_point, planner_data);
      if (will_traffic_light_turn_red_before_reaching_stop_line && !is_stop_signal) {
        debug_data_.is_remaining_time_used = true;
      }

      to_be_stopped = to_be_stopped || will_traffic_light_turn_red_before_reaching_stop_line;
    }

    // Check if the vehicle is stopped and within a certain distance to the stop line
    if (
      planner_data.isVehicleStopped() &&
      planner_param_.min_behind_dist_to_stop_for_restart_suppression <
        signed_arc_length_to_stop_point &&
      signed_arc_length_to_stop_point <
        planner_param_.max_behind_dist_to_stop_for_restart_suppression &&
      is_stop_signal) {
      // Suppress restart
      RCLCPP_DEBUG(logger_, "Suppressing restart due to proximity to stop line.");
      path = insertStopVelocity(path, ego_s, planner_data);
      return true;
    }

    setSafe(!to_be_stopped);
    if (isActivated()) {
      is_prev_state_stop_ = false;
      return true;
    }

    // Decide whether to stop or pass even if a stop signal is received.
    if (!isPassthrough(signed_arc_length_to_stop_point, planner_data)) {
      const double target = planner_param_.enable_signal_compliance_guard && signed_arc_length_to_stop_point < 0.0 ?
        ego_s : *stop_point_s;
      path = insertStopVelocity(path, target, planner_data);
      is_prev_state_stop_ = true;
    }
    return true;
  } else if (state_ == State::GO_OUT) {
    // Initialize if vehicle is far from stop_line
    constexpr bool use_initialization_after_start = true;
    constexpr double restart_length = 1.0;
    if (use_initialization_after_start && (planner_param_.enable_signal_compliance_guard ?
        (crossing_distance && *crossing_distance > restart_length) : signed_arc_length_to_stop_point > restart_length)) {
      RCLCPP_DEBUG(logger_, "GO_OUT(RESTART) -> APPROACH");
      state_ = State::APPROACH;
      signal_compliance_guard_.reset();
      decision_line_latch_.reset();
      stop_requested_on_approach_ = false;
      seen_signal_on_approach_ = false;
      if (planner_param_.enable_signal_compliance_guard) {
        stop_signal_received_time_ptr_.reset();
        return modifyPathVelocity(path,left_bound,right_bound,planner_data);
      }
    }
    stop_signal_received_time_ptr_.reset();
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopSignal(
  const PlannerData & planner_data, const double distance_to_stop_point,
  std::optional<double> virtual_front_distance, const bool genuine_path_stop_point)
{
  // Store previous state before updating
  prev_looking_tl_state_ = looking_tl_state_;

  updateTrafficSignal(planner_data);

  // Keep the existing no-traffic-stack competition behavior. This policy also
  // applies to a lost publisher; use the existing fail-open=false for fail-closed.
  // Fresh RED/UNKNOWN and all current yellow rules remain guarded below.
  if (planner_param_.competition_fail_open &&
      (!traffic_signal_stamp_ || isTrafficSignalTimedOut())) {
    signal_compliance_guard_.reset();
    signal_compliance::publishVideoDecision(
      traffic_light_reg_elem_.id(), clock_->now(), false, false,
      planner_data.current_velocity->twist.linear.x,
      genuine_path_stop_point ? distance_to_stop_point : std::numeric_limits<double>::quiet_NaN(),
      decision_line_official_distance_.value_or(std::numeric_limits<double>::quiet_NaN()),
      std::numeric_limits<double>::quiet_NaN(), "competition_fail_open");
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[traffic_light] missing/stale signal: competition fail-open GO");
    return false;
  }

  if (!planner_param_.enable_yellow_passthrough && isTrafficSignalYellow() &&
      !planner_param_.enable_signal_compliance_guard) {
    return true;
  }

  if (planner_param_.enable_signal_compliance_guard) {
    using signal_compliance::Observation;
    auto observation = Observation::Missing;
    if (traffic_signal_stamp_) {
      const auto & elements = looking_tl_state_.elements;
      const double age = (clock_->now() - *traffic_signal_stamp_).toSec();
      if (age < 0.0 || age > planner_param_.tl_state_timeout) {
        observation = Observation::Stale;
      } else if (elements.empty() || std::any_of(elements.begin(), elements.end(), [](const auto & element) {
          return element.color == TrafficSignalElement::UNKNOWN ||
                 element.shape == TrafficSignalElement::UNKNOWN ||
                 element.status != TrafficSignalElement::SOLID_ON;
        })) {
        observation = Observation::Unknown;
      } else if (isTrafficSignalYellow()) {
        const bool green = std::any_of(elements.begin(), elements.end(), [](const auto & element) {
          return element.color == TrafficSignalElement::GREEN;
        });
        observation = green ? Observation::Unknown : Observation::Yellow;
      } else {
        observation = autoware::traffic_light_utils::isTrafficSignalStop(lane_, looking_tl_state_) ?
          Observation::Stop : Observation::AuthorizedGreen;
      }
    }
    const bool has_color_evidence = observation == Observation::Unknown &&
      std::any_of(looking_tl_state_.elements.begin(), looking_tl_state_.elements.end(),
        [](const auto & element) { return element.color != TrafficSignalElement::UNKNOWN; });
    if (observation == Observation::Stop || observation == Observation::AuthorizedGreen ||
        observation == Observation::Yellow || has_color_evidence) {
      seen_signal_on_approach_ = true;
    }
    const bool require_stop = signal_compliance_guard_.observe(
      observation, traffic_signal_stamp_ ? traffic_signal_stamp_->toSec() : 0.0,
      clock_->now().toSec(), planner_param_.tl_state_timeout, has_color_evidence);
    const bool pending_stop = stop_requested_on_approach_ || decision_line_latch_.pendingStop();
    const auto apply_decision_line = [&](bool guarded_stop, bool defer_initial_unseen = false) {
      // Never turn a cropped path's synthetic current-ego target into proof
      // that the dynamic decision line has been reached. A real finite virtual
      // line may supply the bumper target when the path no longer contains it.
      const double target_distance = genuine_path_stop_point ? distance_to_stop_point :
        virtual_front_distance ? *virtual_front_distance -
          (planner_data.vehicle_info_.max_longitudinal_offset_m -
           planner_param_.signal_compliance_front_axle_offset_m) - planner_param_.stop_margin :
        std::numeric_limits<double>::quiet_NaN();
      const double velocity = planner_data.current_velocity->twist.linear.x;
      const auto braking = signal_compliance::jerkStoppingDistance(
        velocity, planner_param_.signal_compliance_yellow_deceleration_mps2,
        planner_param_.signal_compliance_yellow_jerk_mps3, planner_data.delay_response_time);
      const double age = traffic_signal_stamp_ ?
        (clock_->now()-*traffic_signal_stamp_).toSec() : -1.;
      if (!planner_param_.enable_yellow_passthrough) {
        signal_compliance::publishVideoDecision(
          traffic_light_reg_elem_.id(), clock_->now(), guarded_stop, false, velocity,
          target_distance, decision_line_official_distance_.value_or(
            std::numeric_limits<double>::quiet_NaN()),
          braking.value_or(std::numeric_limits<double>::quiet_NaN()), "yellow_master_off");
        return guarded_stop;
      }
      const auto decision = decision_line_latch_.update(
        observation, age >= 0.0 && age <= planner_param_.tl_state_timeout,
        guarded_stop, velocity, planner_data.isVehicleStopped(), target_distance,
        braking.value_or(std::numeric_limits<double>::quiet_NaN()),
        traffic_signal_stamp_ ? traffic_signal_stamp_->toSec() : 0.0, clock_->now().toSec(),
        defer_initial_unseen);
      signal_compliance::publishVideoDecision(
        traffic_light_reg_elem_.id(), clock_->now(), decision.require_stop,
        decision.committed_go, velocity, target_distance,
        decision_line_official_distance_.value_or(std::numeric_limits<double>::quiet_NaN()),
        decision.stopping_distance_m, decision.reason);
      return decision.require_stop;
    };
    const auto terminal_front_distance = virtual_front_distance ? virtual_front_distance :
      genuine_path_stop_point ? signal_compliance::frontAxleDistanceFromBumperStop(
        distance_to_stop_point, planner_param_.stop_margin,
        planner_data.vehicle_info_.max_longitudinal_offset_m,
        planner_param_.signal_compliance_front_axle_offset_m) : std::nullopt;
    const double terminal_age = traffic_signal_stamp_ ?
      (clock_->now()-*traffic_signal_stamp_).toSec() : -1.;
    if (!pending_stop && signal_compliance::deferTerminalUnknownAfterVirtualLine(
        planner_param_.enable_yellow_passthrough && planner_param_.use_official_judging_lines,
        seen_signal_on_approach_, observation, has_color_evidence, terminal_front_distance,
        terminal_age >= 0.0 && terminal_age <= planner_param_.tl_state_timeout)) {
      RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 200,
        "[signal_lanelet_exit] reg=%ld pure_unknown_past_virtual_line keep_path_open=1 virtual_front_m=%.3f path_stop_target_m=%.3f geometry_basis=%s",
        traffic_light_reg_elem_.id(), *terminal_front_distance, distance_to_stop_point,
        virtual_front_distance ? "finite_lanelet_line" : "genuine_path_stop_point");
      return apply_decision_line(false);
    }
    if (!pending_stop && signal_compliance::deferUnseenUntilObserved(
          planner_param_.defer_unseen_until_observed, seen_signal_on_approach_, observation)) {
      RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000,
        "[signal_compliance] reg=%ld unreadable before first lamp: defer stop point",
        traffic_light_reg_elem_.id());
      return apply_decision_line(false, true);
    }
    if (observation == Observation::Unknown && traffic_signal_stamp_ &&
        !planner_data.current_acceleration) {
      return apply_decision_line(require_stop);
    }
    if (observation == Observation::Unknown && traffic_signal_stamp_) {
      const double speed = planner_data.current_velocity->twist.linear.x;
      const double stopping_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
        speed, planner_data.current_acceleration->accel.accel.linear.x,
        planner_data.max_stop_acceleration_threshold,
        planner_data.max_stop_jerk_threshold, planner_data.delay_response_time);
      if (!pending_stop && signal_compliance::deferFarUnseenUnknown(
            planner_param_.defer_far_unseen_unknown, seen_signal_on_approach_,
            looking_tl_state_.elements.empty(), distance_to_stop_point,
            stopping_distance, speed, planner_param_.far_unseen_min_stop_distance_m,
            planner_param_.far_unseen_response_margin_sec)) {
        RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000,
          "[signal_compliance] reg=%ld empty/unseen far ahead: defer stop point at %.1fm",
          traffic_light_reg_elem_.id(), distance_to_stop_point);
        return apply_decision_line(false);
      }
    }
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 200,
      "[signal_compliance] reg=%ld class=%s source_age_sec=%.3f green_count=%zu require_stop=%d unknown_grace_active=%d unknown_grace_sec=%.3f last_real_green_source_age_sec=%.3f last_real_green_processing_age_sec=%.3f",
      traffic_light_reg_elem_.id(), signal_compliance::observationName(observation),
      traffic_signal_stamp_ ? (clock_->now()-*traffic_signal_stamp_).toSec() : -1.,
      signal_compliance_guard_.greenConfirmationCount(), require_stop,
      signal_compliance_guard_.unknownGraceActive(), signal_compliance_guard_.unknownGraceSec(),
      signal_compliance_guard_.lastRealGreenSourceSec() ?
        clock_->now().toSec()-*signal_compliance_guard_.lastRealGreenSourceSec() : -1.,
      signal_compliance_guard_.lastRealGreenProcessedSec() ?
        clock_->now().toSec()-*signal_compliance_guard_.lastRealGreenProcessedSec() : -1.);
    return apply_decision_line(require_stop);
  }

  // If there is no upcoming traffic signal information,
  //   SIMULATION: it will PASS to prevent stopping on the planning simulator
  //   or scenario simulator.
  //   REAL ENVIRONMENT: it will STOP for safety in cases such that traffic light
  //   recognition is not working properly or the map is incorrect.
  if (!traffic_signal_stamp_) {
    if (planner_param_.competition_fail_open) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "[traffic_light] missing signal: competition fail-open GO");
      return false;
    }
    return !planner_data.is_simulation;
  }

  // Stop if the traffic signal information has timed out
  if (isTrafficSignalTimedOut()) {
    return !planner_param_.competition_fail_open;
  }

  const bool has_known_lamp = std::any_of(
    looking_tl_state_.elements.begin(), looking_tl_state_.elements.end(), [](const auto & element) {
      return element.color != TrafficSignalElement::UNKNOWN &&
             element.shape != TrafficSignalElement::UNKNOWN;
    });
  if (planner_param_.competition_fail_open && !has_known_lamp) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[traffic_light] unknown signal: competition fail-open GO");
    return false;
  }

  // Check if current state is yellow
  const bool is_yellow_now = isTrafficSignalYellow();
  if (is_yellow_now && planner_param_.yellow_light_is_go) {
    return false;
  }

  // Override for yellow light on turn lanes with static arrows
  if (planner_param_.enable_arrow_aware_yellow_passing) {
    updateYellowState(is_yellow_now);

    // Check if conditions are met (Green->Yellow, turn lane, static arrow)
    if (
      is_yellow_now && is_turn_lane_ && has_static_arrow_ &&
      yellow_transition_state_ == YellowState::kFromGreen) {
      // This is a "Green -> Yellow" sequence. This is the state we *do* want to override (pass).
      return false;
    }
  }

  // Check if the current traffic signal state requires stopping
  return autoware::traffic_light_utils::isTrafficSignalStop(lane_, looking_tl_state_);
}

bool TrafficLightModule::isTrafficSignalYellow() const
{
  for (const auto & element : looking_tl_state_.elements) {
    if (
      element.color == TrafficSignalElement::AMBER &&
      (element.status == TrafficSignalElement::SOLID_ON ||
       element.status == TrafficSignalElement::UNKNOWN)) {
      return true;
    }
  }
  return false;
}

void TrafficLightModule::updateYellowState(const bool is_yellow_now)
{
  if (is_yellow_now) {
    // This is a yellow light. Check if this is the *start* of the yellow sequence.
    if (yellow_transition_state_ == YellowState::kNotYellow) {
      // This is the first frame of yellow. Determine how it started.
      if (!prev_looking_tl_state_.elements.empty()) {
        const bool prev_had_green_circle = std::any_of(
          prev_looking_tl_state_.elements.begin(), prev_looking_tl_state_.elements.end(),
          [](const auto & element) {
            return element.shape == TrafficSignalElement::CIRCLE &&
                   element.color == TrafficSignalElement::GREEN;
          });

        if (prev_had_green_circle) {
          RCLCPP_DEBUG_THROTTLE(
            logger_, *clock_, 1000, "[TrafficLight Debug]   -> Detected Green->Yellow transition.");
          yellow_transition_state_ = YellowState::kFromGreen;
        } else {
          RCLCPP_DEBUG_THROTTLE(
            logger_, *clock_, 1000,
            "[TrafficLight Debug]   -> NO Green Circle found in prev state.");
          yellow_transition_state_ = YellowState::kFromNonGreen;
        }
      } else {
        // No previous traffic light state available; cannot determine transition origin.
        RCLCPP_DEBUG_THROTTLE(
          logger_, *clock_, 1000,
          "[TrafficLight Debug]   -> Previous TL state unavailable; "
          "treating Yellow transition as unsafe; defaulting to stop behavior.");
        // Leave yellow_transition_state_ as kNotYellow so that no special passing logic is
        // applied.
      }
    }
  } else {
    // Not yellow. Reset the state.
    if (yellow_transition_state_ != YellowState::kNotYellow) {
      RCLCPP_DEBUG_THROTTLE(
        logger_, *clock_, 1000, "[TrafficLight Debug] Module %ld: Yellow ended. Resetting state.",
        module_id_);
    }
    yellow_transition_state_ = YellowState::kNotYellow;
  }
}

bool TrafficLightModule::willTrafficLightTurnRedBeforeReachingStopLine(
  const double & distance_to_stop_line, const PlannerData & planner_data) const
{
  double ego_velocity = planner_data.current_velocity->twist.linear.x;
  double predicted_passing_stop_line_time = ego_velocity > planner_param_.v2i_velocity_threshold
                                              ? distance_to_stop_line / ego_velocity
                                              : planner_param_.v2i_required_time_to_departure;

  double seconds = predicted_passing_stop_line_time + planner_param_.v2i_last_time_allowed_to_pass;

  ros::Time now = clock_->now();
  // find stop signal from looking_tl_state_.predictions by using isTrafficSignalStop
  for (const auto & prediction : looking_tl_state_.predictions) {
    if (isTrafficSignalRedStop(lane_, prediction.simultaneous_elements)) {
      ros::Time predicted_time(prediction.predicted_stamp.sec, prediction.predicted_stamp.nanosec);
      ros::Duration remaining_time = predicted_time - now;
      if (remaining_time.toSec() < seconds) {
        return true;
      }
    }
  }
  return false;
}

void TrafficLightModule::updateTrafficSignal(const PlannerData & planner_data)
{
  TrafficSignalStamped signal;
  if (!findValidTrafficSignal(signal, planner_data)) {
    if (planner_param_.enable_signal_compliance_guard) {
      traffic_signal_stamp_.reset();
    }
    // Don't stop if it never receives traffic light topic.
    // Reset looking_tl_state
    looking_tl_state_.elements.clear();
    looking_tl_state_.traffic_light_group_id = 0;
    return;
  }

  traffic_signal_stamp_ = signal.stamp;

  // Found signal associated with the lanelet
  looking_tl_state_ = signal.signal;
}

bool TrafficLightModule::isPassthrough(
  const double & signed_arc_length, const PlannerData & planner_data) const
{
  if (!planner_param_.enable_yellow_passthrough) {
    return false;
  }
  if (planner_param_.enable_signal_compliance_guard) {
    // The one-shot decision-line policy already ran for every color above.
    // A STOP must not be reopened here merely because distance has shrunk.
    return false;
  }

  // Explicit guard-OFF legacy rollback; never selected by production defaults.
  if (is_prev_state_stop_) return false;
  const double reachable_distance =
    planner_data.current_velocity->twist.linear.x * planner_param_.yellow_lamp_period;
  const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data.current_velocity->twist.linear.x,
    planner_data.current_acceleration->accel.accel.linear.x,
    planner_data.max_stop_acceleration_threshold, planner_data.max_stop_jerk_threshold,
    planner_data.delay_response_time);
  const bool distance_stoppable = pass_judge_line_distance < signed_arc_length;
  const bool slow_velocity =
    planner_data.current_velocity->twist.linear.x < planner_param_.yellow_light_stop_velocity;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = signed_arc_length < reachable_distance;

  const auto & enable_pass_judge = planner_param_.enable_pass_judge;

  if (enable_pass_judge && !stoppable) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    } else {
      // pass through
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[traffic_light] can pass through intersection during yellow lamp");
      return true;
    }
  }

  return false;
}

bool TrafficLightModule::findValidTrafficSignal(
  TrafficSignalStamped & valid_traffic_signal, const PlannerData & planner_data) const
{
  // get traffic signal associated with the regulatory element id
  const auto traffic_signal_stamped_opt = planner_data.getTrafficSignal(
    traffic_light_reg_elem_.id(), false /* traffic light module does not keep last observation */);
  if (!traffic_signal_stamped_opt) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_, "the traffic signal data associated with regulatory element id "
                 << traffic_light_reg_elem_.id() << " is not received");
    return false;
  }
  valid_traffic_signal = traffic_signal_stamped_opt.value();
  return true;
}

bool TrafficLightModule::isTrafficSignalTimedOut() const
{
  if (!traffic_signal_stamp_) {
    return false;
  }

  const auto is_traffic_signal_timeout =
    (clock_->now() - *traffic_signal_stamp_).toSec() > planner_param_.tl_state_timeout;
  if (is_traffic_signal_timeout) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "the received traffic signal data is outdated");
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "time diff: " << (clock_->now() - *traffic_signal_stamp_).toSec());
    return true;
  }
  return false;
}

Trajectory TrafficLightModule::insertStopVelocity(
  const Trajectory & input, const double & stop_point_s, const PlannerData & planner_data)
{
  auto modified_path = input;

  const auto stop_pose = modified_path.compute(stop_point_s).point.pose;
  debug_data_.stop_poses.push_back(stop_pose);

  modified_path.set_stopline(stop_point_s);

  planning_factor_interface_->add(
    modified_path.restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::PlanningFactor::STOP,
    autoware_internal_planning_msgs::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "traffic_light");

  return modified_path;
}

void TrafficLightModule::updateStopLine(const lanelet::ConstLineString3d & stop_line)
{
  stop_line_ = stop_line;
}

visualization_msgs::MarkerArray TrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::MarkerArray debug_marker_array;
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls TrafficLightModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  if (debug_data_.is_remaining_time_used) {
    wall.text = "traffic_light(V2I)";
  } else {
    wall.text = "traffic_light";
  }
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = autoware::motion_utils::VirtualWallType::deadline;
  for (const auto & p : debug_data_.dead_line_poses) {
    wall.pose = autoware_utils_geometry::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware_utils_geometry::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
