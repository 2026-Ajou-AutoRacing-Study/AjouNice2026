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

#include "autoware/behavior_path_lane_change_module/interface.hpp"

#include "autoware/behavior_path_lane_change_module/utils/markers.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_lane_change_module/utils/passing_intent.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"

#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <std_msgs/String.h>
#include <mutex>
#include <iomanip>
#include <sstream>

namespace autoware::behavior_path_planner
{
using autoware_utils::append_marker_array;
using utils::lane_change::assignToCandidate;

namespace
{
bool & passingEnabled() { static bool enabled=false;return enabled; }
ros::Publisher & laneChangeTransitionPublisher()
{
  // Advertise at module construction so rosbag's planning regex sees this
  // topic before the first short-lived failure is emitted.
  static ros::Publisher publisher=ros::NodeHandle().advertise<std_msgs::String>(
    "/planning/debug/lane_change_transition",100,false);
  return publisher;
}
template<class UUID> std::string passingHex(const UUID & uuid)
{
  std::ostringstream stream;
  for (auto byte : uuid.uuid) stream << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte);
  return stream.str();
}

// Process-lifetime observer survives normal candidate destruction during an
// external maneuver. No commands/services/control publishers are owned here.
struct PassingIntentObserver
{
  std::mutex mutex;
  passing_intent::Intent intent;
  bool valid{false}, outbound_seen{false}, snapshot_valid{false};
  std::string boot, route;
  double route_stamp{0}, snapshot_stamp{0};
  ros::WallTime received, snapshot_received;
  std::set<int64_t> route_ids;
  std::string message_route;
  double message_route_stamp{0};
  ros::Subscriber intent_sub, ownership_sub, route_sub;
  ros::Publisher ack;
  PassingIntentObserver()
  {
    ros::NodeHandle node;
    route_sub=node.subscribe<autoware_planning_msgs::LaneletRoute>("/planning/mission_planning/route",1,
      [this](const autoware_planning_msgs::LaneletRoute::ConstPtr & message) {
        std::lock_guard<std::mutex> lock(mutex);
        message_route=passingHex(message->uuid);message_route_stamp=message->header.stamp.toSec();
        route_ids.clear();
        for (const auto & segment:message->segments)
          for (const auto & primitive:segment.primitives) route_ids.insert(primitive.id);
      });
    ack=node.advertise<std_msgs::String>("/evaluation/overtaking_policy/intent_ack",10,false);
    intent_sub=node.subscribe<std_msgs::String>("/evaluation/overtaking_policy/passing_intent",1,
      [this](const std_msgs::String::ConstPtr & message) {
        std::lock_guard<std::mutex> lock(mutex);
        try {
          boost::property_tree::ptree p;std::istringstream input(message->data);read_json(input,p);
          passing_intent::Intent next;
          next.id=p.get<std::string>("intent_uuid");next.outbound_uuid=p.get<std::string>("outbound_uuid");
          next.lead_uuid=p.get<std::string>("lead_uuid");next.route=p.get<std::string>("route_uuid");
          next.boot=p.get<std::string>("boot_id");next.map=p.get<std::string>("map_sha");
          next.direction=p.get<std::string>("direction");next.phase=p.get<std::string>("phase");
          next.route_stamp=p.get<double>("route_stamp");next.stamp=p.get<double>("stamp");next.ttl=p.get<double>("ttl");
          for (const auto & lane:p.get_child("source_chain")) next.source.insert(lane.second.get_value<int64_t>());
          for (const auto & lane:p.get_child("target_chain")) next.target.insert(lane.second.get_value<int64_t>());
          if (next.id != intent.id || !valid || (ros::WallTime::now()-received).toSec()>intent.ttl)
            outbound_seen=false;
          else if (next.outbound_uuid!=intent.outbound_uuid || next.lead_uuid!=intent.lead_uuid ||
                   next.route!=intent.route || next.route_stamp!=intent.route_stamp || next.boot!=intent.boot ||
                   next.direction!=intent.direction || next.source!=intent.source || next.target!=intent.target) {
            valid=false;outbound_seen=false;return;
          }
          intent=std::move(next);received=ros::WallTime::now();valid=true;
        } catch (...) {valid=false;outbound_seen=false;}
      });
    ownership_sub=node.subscribe<std_msgs::String>("/planning/debug/behavior_path_planner/module_ownership",1,
      [this](const std_msgs::String::ConstPtr & message) {
        std::lock_guard<std::mutex> lock(mutex);
        try {
          boost::property_tree::ptree p;std::istringstream input(message->data);read_json(input,p);
          const auto next_boot=p.get<std::string>("boot_id");
          const auto next_route=p.get<std::string>("route_uuid");
          const auto next_route_stamp=p.get<double>("route_stamp");
          if (boot!=next_boot || route!=next_route || route_stamp!=next_route_stamp) outbound_seen=false;
          boot=next_boot;route=next_route;route_stamp=next_route_stamp;
          snapshot_stamp=p.get<double>("stamp");snapshot_received=ros::WallTime::now();
          snapshot_valid=p.get<bool>("valid") && p.get<bool>("complete_snapshot");
          const auto now=ros::Time::now().toSec();
          if (!snapshot_valid || !valid || now-snapshot_stamp<0 || now-snapshot_stamp>.4 ||
              now-intent.stamp<0 || now-intent.stamp>intent.ttl || intent.route!=route ||
              intent.route_stamp!=route_stamp || intent.boot!=boot) return;
          for (const auto & item:p.get_child("modules")) {
            const auto & module=item.second;
            if (module.get<std::string>("name")!="external_request_lane_change_"+intent.direction ||
                !module.get<bool>("approved") || !module.get<bool>("active")) continue;
            for (const auto & rtc:module.get_child("rtc_uuids"))
              if (rtc.second.get<std::string>("uuid")==intent.outbound_uuid) outbound_seen=true;
          }
        } catch (...) {snapshot_valid=false;outbound_seen=false;}
      });
  }
};
PassingIntentObserver & passingObserver()
{
  static PassingIntentObserver observer;
  return observer;
}
}  // namespace

LaneChangeInterface::LaneChangeInterface(
  const std::string & name, ros::NodeHandle & node, std::shared_ptr<LaneChangeParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface,
  std::unique_ptr<LaneChangeBase> && module_type)
: SceneModuleInterface{name, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface, ModuleStatus::WAITING_APPROVAL},  // NOLINT
  parameters_{std::move(parameters)},
  module_type_{std::move(module_type)}
{
  module_type_->setTimeKeeper(getTimeKeeper());
  (void)laneChangeTransitionPublisher();
  node.param("lane_change/passing_intent/enable_simulator_experiment",passingEnabled(),false);
  if (passingEnabled()) (void)passingObserver();
}

void LaneChangeInterface::processOnExit()
{
  module_type_->resetParameters();
  debug_marker_.markers.clear();
  post_process_safety_status_ = {};
  interface_debug_ = {};
  resetPathCandidate();
}

bool LaneChangeInterface::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  if (auto err = module_type_->isLaneChangeRequired()) {
    interface_debug_.request_info = err.value();
    return false;
  }

  if (passingEnabled() && deferPassingConvenienceReturn()) {
    interface_debug_.request_info = "Experiment passing intent: defer convenience return.";
    return false;
  }

  interface_debug_.request_info = {};

  const auto & current_lanes = module_type_->get_current_lanes();
  const auto & target_lanes = module_type_->get_target_lanes();

  auto get_ids = [](const auto & lanes) {
    std::vector<decltype(lanes[0].id())> ids;
    ids.reserve(lanes.size());
    for (const auto & lane : lanes) ids.push_back(lane.id());
    return ids;
  };

  interface_debug_.request_info = fmt::format(
    "Current lanes: {} | Target lanes: {}", fmt::join(get_ids(current_lanes), ", "),
    fmt::join(get_ids(target_lanes), ", "));

  return true;
}

bool LaneChangeInterface::deferPassingConvenienceReturn() const
{
  auto & observer=passingObserver();
  std::lock_guard<std::mutex> lock(observer.mutex);
  if (!observer.valid || !observer.snapshot_valid ||
      (ros::WallTime::now()-observer.snapshot_received).toSec()>.4) return false;
  passing_intent::Context c;
  c.enabled=passingEnabled();c.now=ros::Time::now().toSec();
  c.intent_wall_age=(ros::WallTime::now()-observer.received).toSec();
  c.boot=observer.boot;c.ownership_stamp=observer.snapshot_stamp;
  c.normal=module_type_->getModuleType()==LaneChangeModuleType::NORMAL;
  c.running=getCurrentStatus()==ModuleStatus::RUNNING;
  c.aborting=module_type_->isAbortState();
  c.direction=module_type_->getDirection()==Direction::LEFT ? "left" : "right";
  try {
    const auto handler=module_type_->getRouteHandler();
    c.route=passingHex(handler->getRouteUuid());c.route_stamp=handler->getRouteHeader().stamp.toSec();
    c.route_verified=c.route==observer.route && std::abs(c.route_stamp-observer.route_stamp)<1e-6 &&
      c.route==observer.message_route && std::abs(c.route_stamp-observer.message_route_stamp)<1e-6;
    const std::vector<int64_t> expected={201459,201436,201465,200138,201444,201469,200127,201473,201439,200185,201453};
    const auto preferred=handler->getPreferredLanelets();
    lanelet::ConstLanelets highway;
    for (auto lane=preferred.begin();lane!=preferred.end();++lane) {
      if (highway.empty() && lane->id()!=expected.front()) continue;
      highway.push_back(*lane);if (lane->id()==expected.back()) break;
    }
    c.corridor_verified=highway.size()==expected.size();
    for (size_t n=0;c.corridor_verified && n<expected.size();++n)
      c.corridor_verified=highway[n].id()==expected[n];
    if (!c.corridor_verified) return false;
    const auto & route_ids=observer.route_ids;
    const auto graph=handler->getRoutingGraphPtr();
    for (size_t n=0;n+1<highway.size();++n) {
      c.corridor.insert(highway[n].id());
      for (const bool left:{false,true}) {
        auto lane=highway[n];
        for (int k=0;k<4;++k) {
          const auto next=left ? graph->left(lane):graph->right(lane);
          if (!next || !route_ids.count(next->id())) break;
          c.corridor.insert(next->id());lane=*next;
        }
      }
    }
    lanelet::ConstLanelet ego_lane;
    if (!handler->getClosestLaneletWithinRoute(module_type_->getEgoPose(),&ego_lane)) return false;
    c.current_lane=ego_lane.id();
    if (!c.corridor.count(c.current_lane)) return false;
    if (ego_lane.attributeOr("speed_limit",0.0)<80.0) return false;
    // Validate lateral relationship against this live routing graph, not just XY.
    for (const auto source:observer.intent.source) {
      if (!route_ids.count(source)) return false;
      const auto source_lane=handler->getLaneletsFromId(source);
      const auto adjacent=observer.intent.direction=="left" ? graph->left(source_lane):graph->right(source_lane);
      if (!adjacent || !observer.intent.target.count(adjacent->id())) return false;
    }
    const auto next=c.direction=="left" ? graph->left(ego_lane):graph->right(ego_lane);
    if (next) c.target_lane=next->id();
    const auto path=handler->getCenterLinePath(highway,0.0,std::numeric_limits<double>::max());
    geometry_msgs::Point end;
    const auto point=highway.back().centerline().front();end.x=point.x();end.y=point.y();end.z=point.z();
    c.remaining=autoware::motion_utils::calcSignedArcLength(path.points,module_type_->getEgoPosition(),end);
    // First loss of the legal lateral connection along the actual ego chain,
    // not the canonical preferred-lane end alone. Branches/gaps terminate it.
    const bool on_pass_lane=observer.intent.target.count(c.current_lane);
    const auto & own_chain=on_pass_lane ? observer.intent.target:observer.intent.source;
    const auto & other_chain=on_pass_lane ? observer.intent.source:observer.intent.target;
    const bool connect_left=on_pass_lane ? observer.intent.direction=="right":observer.intent.direction=="left";
    lanelet::ConstLanelets connected;
    auto cursor=ego_lane;
    std::set<int64_t> visited;
    while (own_chain.count(cursor.id()) && visited.insert(cursor.id()).second) {
      const auto side=connect_left ? graph->left(cursor):graph->right(cursor);
      if (!side || !other_chain.count(side->id())) break;
      connected.push_back(cursor);
      lanelet::ConstLanelets following;
      for (const auto & next_lane:graph->following(cursor))
        if (own_chain.count(next_lane.id())) following.push_back(next_lane);
      if (following.size()!=1) break;
      cursor=following.front();
    }
    if (connected.empty()) return false;
    const auto connected_path=handler->getCenterLinePath(connected,0.0,std::numeric_limits<double>::max());
    if (connected_path.points.size()<2) return false;
    c.lateral_distance=autoware::motion_utils::calcSignedArcLength(
      connected_path.points,module_type_->getEgoPosition(),connected_path.points.back().point.pose.position);
    const auto transient=module_type_->passingIntentTransientData();
    c.distances_valid=transient.has_valid_lc_distance;
    c.return_buffer=transient.current_dist_buffer.max;
    c.terminal_distance=transient.dist_to_terminal_end;
    c.speed=std::abs(module_type_->getEgoVelocity());c.near_terminal=module_type_->is_near_terminal_end();
    const auto acceleration=module_type_->passingIntentAcceleration();
    if (!acceleration) return false;
    c.acceleration=*acceleration;c.front_margin=module_type_->passingIntentFrontMargin();
    double validator_acceleration=0,smoother_acceleration=0,smoother_jerk=0;
    // Read existing active contracts. Unknown/moved parameters fail closed;
    // there is deliberately no duplicated -3/-6/-10 default here.
    if (!ros::param::getCached("/planning_validator/trajectory_checker/forward_trajectory_length/acceleration",validator_acceleration) ||
        !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_acc",smoother_acceleration) ||
        !ros::param::getCached("/planning/scenario_planning/velocity_smoother/normal/min_jerk",smoother_jerk) ||
        !std::isfinite(validator_acceleration) || !std::isfinite(smoother_acceleration) || !std::isfinite(smoother_jerk) ||
        validator_acceleration>=0 || smoother_acceleration>=0 || smoother_jerk>=0 ||
        !std::isfinite(parameters_->trajectory.min_longitudinal_acc) || parameters_->trajectory.min_longitudinal_acc>=0)
      return false;
    c.deceleration=std::min({-validator_acceleration,-smoother_acceleration,-parameters_->trajectory.min_longitudinal_acc});
    c.jerk=-smoother_jerk;
  } catch (...) {return false;}
  const auto decision=passing_intent::decide(observer.intent,c,observer.outbound_seen);
  boost::property_tree::ptree ack;
  ack.put("intent_uuid",observer.intent.id);ack.put("outbound_uuid",observer.intent.outbound_uuid);
  ack.put("route_uuid",c.route);ack.put("route_stamp",c.route_stamp);ack.put("boot_id",c.boot);
  ack.put("stamp",c.now);ack.put("module",name());ack.put("acknowledged",decision.acknowledged);
  ack.put("suppress",decision.suppress);ack.put("outbound_seen",observer.outbound_seen);
  ack.put("reserve",decision.reserve);ack.put("remaining",c.remaining);ack.put("reason",decision.reason);
  std::ostringstream stream;write_json(stream,ack,false);
  std_msgs::String message;message.data=stream.str();observer.ack.publish(message);
  return decision.suppress;
}

bool LaneChangeInterface::isExecutionReady() const
{
  return module_type_->isSafe() && !module_type_->isAbortState();
}

void LaneChangeInterface::updateData()
{
  autoware_utils::ScopedTimeTrack st(__func__, *getTimeKeeper());
  module_type_->setPreviousModuleOutput(getPreviousModuleOutput());
  module_type_->update_lanes(getCurrentStatus() == ModuleStatus::RUNNING);
  module_type_->update_filtered_objects();
  module_type_->update_transient_data(getCurrentStatus() == ModuleStatus::RUNNING);
  module_type_->updateSpecialData();

  if (isWaitingApproval() || module_type_->isAbortState()) {
    module_type_->updateLaneChangeStatus();
  }

  // A late ARM intent can arrive after this candidate was registered. The
  // manager then reuses it without invoking isExecutionRequested(). Re-evaluate
  // the existing ACK contract against the freshly updated candidate each cycle;
  // deliberately ignore suppression here (normal return arbitration is unchanged).
  if (passing_intent::refresh_waiting_candidate_ack(
        passingEnabled(), module_type_->getModuleType()==LaneChangeModuleType::EXTERNAL_REQUEST,
        isWaitingApproval(), isActivated(), module_type_->isAbortState())) {
    (void)deferPassingConvenienceReturn();
  }

  module_type_->resetStopPose();
  updateDebugMarker();
}

void LaneChangeInterface::postProcess()
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    const auto safety_status = module_type_->isApprovedPathSafe();
    post_process_safety_status_ =
      module_type_->evaluateApprovedPathWithUnsafeHysteresis(safety_status);
  }
  updateDebugMarker();
}

BehaviorModuleOutput LaneChangeInterface::plan()
{
  autoware_utils::ScopedTimeTrack st(__func__, *getTimeKeeper());
  resetPathCandidate();
  resetPathReference();

  auto output = module_type_->generateOutput();
  path_reference_ = output.reference_path;

  if(module_type_->isCancelState()) {
    updateRTCStatus(std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(),true,State::FAILED);
    return output;
  }

  stop_pose_ = module_type_->getStopPose();

  // The ROS1 safety core retains predicted poses/polygons for collision and
  // planning-factor output, but not the original Shape message required by
  // the ROS2 object-marker publisher.  Keep safety behavior intact and omit
  // only this optional object-visualization side effect.

  updateSteeringFactorPtr(output);
  if (module_type_->isAbortState()) {
    const auto candidate = planCandidate();
    path_candidate_ = candidate.path_candidate;
    updateRTCStatus(
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), true,
      State::ABORTING);
  } else {
    const auto path =
      assignToCandidate(module_type_->getLaneChangePath(), module_type_->getEgoPosition());
    const auto is_registered = std::any_of(
      rtc_interfaces_.begin(), rtc_interfaces_.end(),
      [&](const auto & rtc) { return rtc.second->isRegistered(rtcUuid(rtc.first)); });

    if (!is_registered) {
      updateRTCStatus(
        path.start_distance_to_path_change, path.finish_distance_to_path_change, true,
        State::WAITING_FOR_EXECUTION);
    } else {
      updateRTCStatus(
        path.start_distance_to_path_change, path.finish_distance_to_path_change,
        !is_rtc_force_activated(), State::RUNNING);
    }
  }

  set_longitudinal_planning_factor(output.path);

  return output;
}

BehaviorModuleOutput LaneChangeInterface::planWaitingApproval()
{
  BehaviorModuleOutput out = module_type_->getTerminalLaneChangePath();

  module_type_->insert_stop_point(module_type_->get_current_lanes(), out.path, true);
  out.turn_signal_info = module_type_->get_current_turn_signal_info();

  // See plan(): ROS1 collision diagnostics do not carry marker Shape data.

  path_reference_ = out.reference_path;
  stop_pose_ = module_type_->getStopPose();

  if (!module_type_->isValidPath()) {
    path_candidate_ = PathWithLaneId{};
    updateRTCStatus(
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), false,
      State::WAITING_FOR_EXECUTION);
    // Keep the pending request's signal/preparation clock. updateLaneChangeStatus
    // clears invalid geometry; a full reset here restarts preparation every cycle.
    return out;
  }

  const auto candidate = planCandidate();
  path_candidate_ = candidate.path_candidate;
  const bool execution_ready = isExecutionReady();

  updateRTCStatus(
    candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change,
    execution_ready, State::WAITING_FOR_EXECUTION);
  is_abort_path_approved_ = false;

  set_longitudinal_planning_factor(out.path);

  return out;
}

CandidateOutput LaneChangeInterface::planCandidate() const
{
  autoware_utils::ScopedTimeTrack st(__func__, *getTimeKeeper());
  const auto selected_path = module_type_->getLaneChangePath();

  if (selected_path.path.points.empty()) {
    return {};
  }

  CandidateOutput output = assignToCandidate(selected_path, module_type_->getEgoPosition());

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

void LaneChangeInterface::updateModuleParams(const std::any & parameters)
{
  parameters_ = std::any_cast<std::shared_ptr<LaneChangeParameters>>(parameters);
}

void LaneChangeInterface::setData(const std::shared_ptr<const PlannerData> & data)
{
  planner_data_ = data;
  module_type_->setData(data);
}

bool LaneChangeInterface::canTransitSuccessState()
{
  auto log_debug_throttled = [&](std::string_view message) -> void {
    RCLCPP_DEBUG(getLogger(), "%s", message.data());
  };
  updateDebugMarker();

  // A pending candidate can expire or become unnecessary at a merge, but it
  // has never executed. Do not report it as a successfully executed maneuver.
  if (isWaitingApproval()) {
    return false;
  }

  if (module_type_->hasFinishedLaneChange()) {
    module_type_->resetParameters();
    log_debug_throttled("Lane change process has completed.");
    return true;
  }

  if (
    planner_data_ && planner_data_->operation_mode &&
    planner_data_->operation_mode->mode != OperationModeState::AUTONOMOUS &&
    module_type_->is_near_terminal_end()) {
    return true;
  }

  log_debug_throttled("Lane changing process is ongoing");
  return false;
}

bool LaneChangeInterface::canTransitFailureState()
{
  const auto force_activated = std::invoke([&]() {
    if (!is_rtc_force_activated()) {
      return false;
    }

    if (!module_type_->isValidPath()) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000, "Force activated, but no valid path. Ignore force activation.");
      return false;
    }

    return true;
  });

  if (force_activated) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "unsafe but force executed");
    return false;
  }

  const auto [state, reason] = check_transit_failure();

  interface_debug_.failing_reason = reason;
  interface_debug_.lc_state = state;

  updateDebugMarker();

  // Preserve transition evidence BEFORE onExit/reset clears debug and the
  // manager discards this module's output. No planning decision is recomputed.
  if (state == LaneChangeStates::Cancel ||
      (state == LaneChangeStates::Abort && !module_type_->isAbortState())) {
    try {
      boost::property_tree::ptree event,rtcs,current_lanes,target_lanes,collisions;
      event.put("stamp",ros::Time::now().toSec());event.put("module",name());
      event.put("route_uuid",passingHex(module_type_->getRouteHandler()->getRouteUuid()));
      event.put("route_stamp",module_type_->getRouteHeader().stamp.toSec());
      event.put("reason",std::string(reason));
      event.put("transition",state==LaneChangeStates::Cancel ? "CANCEL":"ABORT");
      event.put("module_state_before",static_cast<int>(getCurrentStatus()));
      event.put("waiting_approval",isWaitingApproval());
      event.put("postprocess_safe",post_process_safety_status_.is_safe);
      event.put("unsafe_trailing",post_process_safety_status_.is_trailing_object);
      event.put("force_deactivated",is_rtc_force_deactivated());
      event.put("ego.speed",module_type_->getEgoVelocity());
      const auto & pose=module_type_->getEgoPose();
      event.put("ego.x",pose.position.x);event.put("ego.y",pose.position.y);
      if (planner_data_ && planner_data_->self_acceleration)
        event.put("ego.acceleration",planner_data_->self_acceleration->accel.accel.linear.x);
      for (const auto & rtc:getRtcUuids()) {
        boost::property_tree::ptree item;
        item.put("interface",rtc.first);item.put("uuid",passingHex(rtc.second));
        rtcs.push_back({"",item});
      }
      for (const auto & lane:module_type_->get_current_lanes()) {
        boost::property_tree::ptree item;item.put("",lane.id());current_lanes.push_back({"",item});
      }
      for (const auto & lane:module_type_->get_target_lanes()) {
        boost::property_tree::ptree item;item.put("",lane.id());target_lanes.push_back({"",item});
      }
      event.add_child("rtc_uuids",rtcs);event.add_child("current_lanes",current_lanes);
      event.add_child("target_lanes",target_lanes);
      const auto & status=module_type_->getLaneChangeStatus();
      event.put("path_valid",status.is_valid_path);event.put("candidate_safe",status.is_safe);
      event.put("path_type",static_cast<int>(status.lane_change_path.type));
      event.put("path_points",status.lane_change_path.path.points.size());
      if (!status.lane_change_path.path.points.empty()) {
        const auto & info=status.lane_change_path.info;
        event.put("lc.start_x",info.lane_changing_start.position.x);
        event.put("lc.start_y",info.lane_changing_start.position.y);
        event.put("lc.end_x",info.lane_changing_end.position.x);
        event.put("lc.end_y",info.lane_changing_end.position.y);
        event.put("lc.prepare_duration",info.duration.prepare);
        event.put("lc.prepare_acceleration",info.longitudinal_acceleration.prepare);
        event.put("lc.changing_acceleration",info.longitudinal_acceleration.lane_changing);
        event.put("lc.phase",module_type_->isEgoOnPreparePhase() ? "PREPARE":"CHANGING");
      }
      const auto & debug=module_type_->getDebugData();
      event.put("can_return_last_check",debug.is_able_to_return_to_current_lane);
      event.put("distance_to_finish_last_check",debug.distance_to_lane_change_finished);
      const auto append_collisions=[&](const auto & data,const char * stage) {
        for (const auto & entry:data) {
          const auto & value=entry.second;
          if (value.is_safe) continue;
          unique_identifier_msgs::UUID id;id.uuid=entry.first;
          boost::property_tree::ptree item;
          item.put("uuid",passingHex(id));item.put("stage",stage);
          item.put("unsafe_reason",value.unsafe_reason);
          // safety_core's overlap branches populate the expected poses/polygons,
          // not the legacy current-pose/RSS/front fields. Do not report their
          // default zeros as measured positions or distances.
          const bool overlap_evidence = value.unsafe_reason == "overlap_polygon" ||
            value.unsafe_reason == "overlap_extended_polygon";
          item.put("legacy_rss_and_current_pose_valid",false);
          item.put("expected_pose_valid",overlap_evidence);
          item.put("expected_ego_x",value.expected_ego_pose.position.x);
          item.put("expected_ego_y",value.expected_ego_pose.position.y);
          item.put("expected_object_x",value.expected_object_pose.position.x);
          item.put("expected_object_y",value.expected_object_pose.position.y);
          const auto prediction_json=[](const auto & prediction) {
            boost::property_tree::ptree samples;
            for (const auto & sample:prediction) {
              boost::property_tree::ptree point;
              point.put("time",sample.time);point.put("velocity",sample.velocity);
              point.put("x",sample.pose.position.x);point.put("y",sample.pose.position.y);
              point.put("qx",sample.pose.orientation.x);point.put("qy",sample.pose.orientation.y);
              point.put("qz",sample.pose.orientation.z);point.put("qw",sample.pose.orientation.w);
              samples.push_back({"",point});
            }
            return samples;
          };
          const auto polygon_json=[](const auto & polygon) {
            boost::property_tree::ptree ring;
            for (const auto & vertex:polygon.outer()) {
              boost::property_tree::ptree point;
              point.put("x",vertex.x());point.put("y",vertex.y());
              ring.push_back({"",point});
            }
            return ring;
          };
          // Exact stored inputs/failed polygons, not a recomputed safety check.
          // Relative prediction times belong to this retained entry's evaluation;
          // no absolute evaluation stamp is available in CollisionCheckDebug.
          item.put("evaluation_stamp_available",false);
          item.add_child("ego_prediction",prediction_json(value.ego_predicted_path));
          item.add_child("object_prediction",prediction_json(value.object_predicted_path));
          item.add_child("extended_ego_polygon",polygon_json(value.extended_ego_polygon));
          item.add_child("extended_object_polygon",polygon_json(value.extended_object_polygon));
          collisions.push_back({"",item});
        }
      };
      append_collisions(debug.collision_check_objects,"candidate");
      append_collisions(debug.collision_check_objects_after_approval,"after_approval_retained_up_to_2s");
      event.put("unsafe_debug_count",collisions.size());event.add_child("unsafe_debug",collisions);
      std::ostringstream stream;write_json(stream,event,false);
      std_msgs::String message;message.data=stream.str();laneChangeTransitionPublisher().publish(message);
      RCLCPP_WARN(getLogger(),"LC transition module=%s reason=%s state=%d postprocess_safe=%d speed=%.2f",
        name().c_str(),std::string(reason).c_str(),static_cast<int>(state),
        post_process_safety_status_.is_safe,module_type_->getEgoVelocity());
    } catch (const std::exception & error) {
      // Diagnostics must not turn a normal cancel into a planning failure.
      RCLCPP_WARN(getLogger(),"LC transition diagnostic unavailable: %s",error.what());
    }
  }

  if (state == LaneChangeStates::Cancel) {
    updateRTCStatus(
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), true,
      State::FAILED);
    module_type_->toCancelState();
    return true;
  }

  if (state == LaneChangeStates::Abort) {
    if (!module_type_->isAbortState()) {
      RCLCPP_DEBUG(getLogger(), "Transit to Abort state.");

      const auto current_pose = module_type_->getEgoPose();

      const auto planning_factor_direction = std::invoke([&]() {
        if (module_type_->getDirection() == Direction::LEFT) {
          return PlanningFactor::SHIFT_LEFT;
        }
        if (module_type_->getDirection() == Direction::RIGHT) {
          return PlanningFactor::SHIFT_RIGHT;
        }
        return PlanningFactor::NONE;
      });

      planning_factor_interface_->add(
        0.0, current_pose, planning_factor_direction, SafetyFactorArray{}, true, 0.0, 0.0, "abort");
    }
    module_type_->toAbortState();
    return false;
  }

  // Note: Ideally, if it is unsafe, but for some reason, we can't abort or cancel, then we should
  // stop. Note: This feature is not working properly for now.
  const auto [is_safe, unsafe_trailing_obj] = post_process_safety_status_;
  if (!is_safe && module_type_->isRequiredStop(unsafe_trailing_obj)) {
    module_type_->toStopState();
    return false;
  }

  module_type_->toNormalState();
  return false;
}

std::pair<LaneChangeStates, std::string_view> LaneChangeInterface::check_transit_failure()
{
  if(module_type_->isCancelState())
    return {LaneChangeStates::Cancel,"ValidatedCurrentLaneCancel"};
  if (
    planner_data_ && planner_data_->operation_mode &&
    planner_data_->operation_mode->mode != OperationModeState::AUTONOMOUS &&
    module_type_->is_near_terminal_end()) {
    return {LaneChangeStates::Cancel, "ManualModeNearTerminal"};
  }
  if (module_type_->isAbortState()) {
    if (module_type_->hasFinishedAbort()) {
      return {LaneChangeStates::Cancel, "Aborted"};
    }
    return {LaneChangeStates::Abort, "Aborting"};
  }

  if (!module_type_->is_ego_in_current_or_target_lanes()) {
    return {LaneChangeStates::Cancel, "EgoOutOfLanes"};
  }

  if (module_type_->hasMissedLaneChangePath()) {
    return {LaneChangeStates::Cancel, "MissedLaneChangePath"};
  }

  if (isWaitingApproval()) {
    if (module_type_->specialExpiredCheck()) {
      return {LaneChangeStates::Cancel, "PendingCandidateExpired"};
    }
    if (module_type_->isValidPath() && module_type_->hasFinishedLaneChange()) {
      return {LaneChangeStates::Cancel, "PendingCandidateAlreadyPassed"};
    }
    if (module_type_->is_near_regulatory_element()) {
      return {LaneChangeStates::Cancel, "CloseToRegElement"};
    }
    return {LaneChangeStates::Normal, "WaitingForApproval"};
  }

  if (!module_type_->isValidPath()) {
    return {LaneChangeStates::Cancel, "InvalidPath"};
  }

  if (module_type_->is_near_terminal_end()) {
    return {LaneChangeStates::Warning, "TooNearTerminal"};
  }

  const auto is_preparing = module_type_->isEgoOnPreparePhase();
  const auto can_return_to_current = module_type_->isAbleToReturnCurrentLane();

  // regardless of safe and unsafe, we want to cancel lane change.
  if (is_preparing && is_rtc_force_deactivated() && can_return_to_current) {
    return {LaneChangeStates::Cancel, "ForceDeactivation"};
  }

  if (post_process_safety_status_.is_safe) {
    return {LaneChangeStates::Normal, "SafeToLaneChange"};
  }

  if (!module_type_->isCancelEnabled()) {
    return {LaneChangeStates::Warning, "CancelDisabled"};
  }

  // We also check if the ego can return to the current lane, as prepare segment might be out of the
  // lane, for example, during an evasive maneuver around a static object.
  if (is_preparing && can_return_to_current) {
    return {LaneChangeStates::Cancel, "SafeToCancel"};
  }

  if (!module_type_->isAbortEnabled()) {
    return {LaneChangeStates::Warning, "AbortDisabled"};
  }

  // To prevent the lane module from having to check rear objects in the current lane, we limit the
  // abort maneuver to cases where the ego vehicle is still in the current lane.
  if (!can_return_to_current) {
    return {LaneChangeStates::Warning, "TooLateToAbort"};
  }

  const auto found_abort_path = module_type_->calcAbortPath();
  if (!found_abort_path) {
    return {LaneChangeStates::Warning, "AbortPathNotFound"};
  }

  return {LaneChangeStates::Abort, "SafeToAbort"};
}

void LaneChangeInterface::updateDebugMarker() const
{
  debug_marker_.markers.clear();
  if (!parameters_->publish_debug_marker) {
    return;
  }
  using marker_utils::lane_change_markers::createDebugMarkerArray;
  debug_marker_ = createDebugMarkerArray(
    interface_debug_, module_type_->getDebugData(), module_type_->getEgoPose());
}

MarkerArray LaneChangeInterface::getModuleVirtualWall()
{
  using marker_utils::lane_change_markers::createLaneChangingVirtualWallMarker;
  MarkerArray marker;

  if (!parameters_->publish_debug_marker) {
    return marker;
  }

  if (isWaitingApproval() || getCurrentStatus() != ModuleStatus::RUNNING) {
    return marker;
  }
  const auto & start_pose = module_type_->getLaneChangePath().info.lane_changing_start;
  const auto start_marker =
    createLaneChangingVirtualWallMarker(start_pose, name(), ros::Time::now(), "lane_change_start");

  const auto & end_pose = module_type_->getLaneChangePath().info.lane_changing_end;
  const auto end_marker =
    createLaneChangingVirtualWallMarker(end_pose, name(), ros::Time::now(), "lane_change_end");
  marker.markers.reserve(start_marker.markers.size() + end_marker.markers.size());
  append_marker_array(start_marker, &marker);
  append_marker_array(end_marker, &marker);
  return marker;
}

void LaneChangeInterface::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  autoware_utils::ScopedTimeTrack st(__func__, *getTimeKeeper());

  const auto current_position = module_type_->getEgoPosition();
  const auto status = module_type_->getLaneChangeStatus();
  const auto start_distance = autoware::motion_utils::calcSignedArcLength(
    output.path.points, current_position, status.lane_change_path.info.shift_line.start.position);
  const auto finish_distance = autoware::motion_utils::calcSignedArcLength(
    output.path.points, current_position, status.lane_change_path.info.shift_line.end.position);

  const auto start_idx = autoware::motion_utils::findNearestIndex(
    output.path.points, status.lane_change_path.info.shift_line.start.position);
  const auto finish_idx = autoware::motion_utils::findNearestIndex(
    output.path.points, status.lane_change_path.info.shift_line.end.position);
  const double start_velocity = output.path.points.at(start_idx).point.longitudinal_velocity_mps;
  const double end_velocity = output.path.points.at(finish_idx).point.longitudinal_velocity_mps;

  const auto planning_factor_direction = std::invoke([&]() {
    if (module_type_->getDirection() == Direction::LEFT) {
      return PlanningFactor::SHIFT_LEFT;
    }
    if (module_type_->getDirection() == Direction::RIGHT) {
      return PlanningFactor::SHIFT_RIGHT;
    }
    return PlanningFactor::NONE;
  });

  const auto & lane_change_debug = module_type_->getDebugData();
  planning_factor_interface_->add(
    start_distance, finish_distance, status.lane_change_path.info.shift_line.start,
    status.lane_change_path.info.shift_line.end, planning_factor_direction,
    utils::path_safety_checker::to_safety_factor_array(lane_change_debug.collision_check_objects),
    true, start_velocity, end_velocity, status.lane_change_path.info.shift_line.start_shift_length,
    status.lane_change_path.info.shift_line.end_shift_length, "");
}

void LaneChangeInterface::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t planning_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return PlanningFactor::SHIFT_LEFT;
    }
    return PlanningFactor::SHIFT_RIGHT;
  });

  const auto status = module_type_->getLaneChangeStatus();
  const auto start_idx = autoware::motion_utils::findNearestIndex(
    selected_path.path.points, status.lane_change_path.info.shift_line.start.position);
  const auto finish_idx = autoware::motion_utils::findNearestIndex(
    selected_path.path.points, status.lane_change_path.info.shift_line.end.position);
  const double start_velocity =
    selected_path.path.points.at(start_idx).point.longitudinal_velocity_mps;
  const double end_velocity =
    selected_path.path.points.at(finish_idx).point.longitudinal_velocity_mps;

  const auto & lane_change_debug = module_type_->getDebugData();
  planning_factor_interface_->add(
    output.start_distance_to_path_change, output.finish_distance_to_path_change,
    selected_path.info.shift_line.start, selected_path.info.shift_line.end,
    planning_factor_direction,
    utils::path_safety_checker::to_safety_factor_array(lane_change_debug.collision_check_objects),
    true, start_velocity, end_velocity, selected_path.info.shift_line.start_shift_length,
    selected_path.info.shift_line.end_shift_length, "");
}
}  // namespace autoware::behavior_path_planner
