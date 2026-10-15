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

#include <autoware/behavior_path_planner/source_planner_pipeline.hpp>
#include <autoware/behavior_path_planner_common/utils/root_reference_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <boost/scope_exit.hpp>
#include <iostream>
namespace autoware::behavior_path_planner {
void SourcePlannerPipeline::configureModuleSlot(
  const std::vector<std::vector<std::string>> & slot_configuration)
{
  std::unordered_map<std::string, SceneModuleManagerPtr> registered_modules;
  for (const auto & manager_ptr : manager_ptrs_) {
    registered_modules[manager_ptr->name()] = manager_ptr;
  }

  for (const auto & slot : slot_configuration) {
    SubPlannerManager sub_manager(current_route_lanelet_, processing_time_, debug_info_);
    for (const auto & module_name : slot) {
      if (const auto it = registered_modules.find(module_name); it != registered_modules.end()) {
        sub_manager.addSceneModuleManager(it->second);
      } else {
        // TODO(Mamoru Sobue): use LOG
        std::cout << module_name << " registered in slot_configuration is not registered, skipping"
                  << std::endl;
      }
    }
    if (sub_manager.getSceneModuleManager().size() != 0) {
      planner_manager_slots_.push_back(sub_manager);
      // TODO(Mamoru Sobue): use LOG
      std::cout << "added a slot with " << sub_manager.getSceneModuleManager().size() << " modules"
                << std::endl;
    }
  }
}

// This is a temporary process until motion planning can take the terminal pose into account
bool keep_input_points(const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses)
{
  const std::vector<std::string> target_modules = {"goal_planner", "avoidance"};

  const auto target_status = ModuleStatus::RUNNING;

  for (auto & status : statuses) {
    if (status->is_waiting_approval || status->status == target_status) {
      if (
        std::find(target_modules.begin(), target_modules.end(), status->module_name) !=
        target_modules.end()) {
        return true;
      }
    }
  }
  return false;
}

BehaviorModuleOutput SourcePlannerPipeline::run(const std::shared_ptr<PlannerData> & data)
{
  resetProcessingTime();
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("total_time");
  BOOST_SCOPE_EXIT((&processing_time_)(&stop_watch))
  {
    processing_time_.at("total_time") += stop_watch.toc("total_time", true);
  }
  BOOST_SCOPE_EXIT_END;

  debug_info_.scene_status.clear();
  debug_info_.slot_status.clear();

  if (!current_route_lanelet_->has_value()) resetCurrentRouteLanelet(data);

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [&data](const auto & m) { m->setData(data); });

  const bool is_any_approved_module_running = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [&](const auto & slot) {
      return slot.isAnyApprovedPred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL;
      });
    });

  // IDLE is a state in which an execution has been requested but not yet approved.
  // once approved, it basically turns to running.
  const bool is_any_candidate_module_running_or_idle = std::any_of(
    planner_manager_slots_.begin(), planner_manager_slots_.end(), [](const auto & slot) {
      return slot.isAnyCandidatePred([](const auto & m) {
        const auto status = m->getCurrentStatus();
        return status == ModuleStatus::RUNNING || status == ModuleStatus::WAITING_APPROVAL ||
               status == ModuleStatus::IDLE;
      });
    });

  const bool is_any_module_running =
    is_any_approved_module_running || is_any_candidate_module_running_or_idle;

  updateCurrentRouteLanelet(data, is_any_approved_module_running);

  const bool is_out_of_route = utils::isEgoOutOfRoute(
    data->self_odometry->pose.pose, current_route_lanelet_->value(), data->prev_modified_goal,
    data->route_handler);

  if (!is_any_module_running && is_out_of_route) {
    BehaviorModuleOutput result_output = utils::createGoalAroundPath(data);
    ROS_WARN_THROTTLE(5.0,
      "Ego is out of route, no module is running. Skip running scene modules.");
    generateCombinedDrivableArea(result_output, data);
    return result_output;
  }

  SlotOutput result_output = SlotOutput{
    getReferencePath(data),
    false,
    false,
    false,
  };

  for (auto & planner_manager_slot : planner_manager_slots_) {
    if (result_output.is_upstream_failed_approved) {
      // clear all candidate/approved modules of all subsequent slots, and keep result_output as is
      planner_manager_slot.propagateWithFailedApproved();
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_APPROVED_FAILED);
    } else if (result_output.is_upstream_waiting_approved) {
      result_output = planner_manager_slot.propagateWithWaitingApproved(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_WAITING_APPROVED);
    } else if (result_output.is_upstream_candidate_exclusive) {
      result_output = planner_manager_slot.propagateWithExclusiveCandidate(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::UPSTREAM_EXCLUSIVE_CANDIDATE);
    } else {
      result_output = planner_manager_slot.propagateFull(data, result_output);
      debug_info_.slot_status.push_back(SlotStatus::NORMAL);
    }
  }

  std::for_each(manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) {
    m->updateObserver();
    m->publishRTCStatus();
    m->publish_planning_factors();
  });
  // resample the path prior to generating the drivable area
  result_output.valid_output.path = utils::resamplePathWithSpline(
    result_output.valid_output.path, data->parameters.output_path_interval,
    keep_input_points(getSceneModuleStatus()));
  generateCombinedDrivableArea(result_output.valid_output, data);
  return result_output.valid_output;
}

// NOTE: To deal with some policies about drivable area generation, currently DrivableAreaInfo is
// quite messy. Needs to be refactored.
void SourcePlannerPipeline::generateCombinedDrivableArea(
  BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const
{
  if (output.path.points.empty()) {
    ROS_ERROR_STREAM( "[generateCombinedDrivableArea] Output path is empty!");
    return;
  }

  const auto & di = output.drivable_area_info;
  constexpr double epsilon = 1e-3;

  const auto is_driving_forward_opt = autoware::motion_utils::isDrivingForward(output.path.points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (epsilon < std::abs(di.drivable_margin)) {
    // for single free space pull over
    utils::generateDrivableArea(
      output.path, data->parameters.vehicle_length, di.drivable_margin, is_driving_forward);
  } else if (di.is_already_expanded) {
    // for single side shift
    utils::generateDrivableArea(
      output.path, di.drivable_lanes, false, false, false, data, is_driving_forward);
  } else {
    const auto length_before_overlap = autoware::motion_utils::calcArcLength(output.path.points);
    const auto shorten_lanes = utils::cutOverlappedLanes(output.path, di.drivable_lanes);
    const auto length_after_overlap = autoware::motion_utils::calcArcLength(output.path.points);
    if (length_after_overlap + 1.0 < length_before_overlap) {
      ROS_ERROR_STREAM("[generateCombinedDrivableArea] overlap shortened final path: "
        << length_before_overlap << " -> " << length_after_overlap << " m");
    }

    const auto & dp = data->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::expandLanelets(
      shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);

    // for other modules where multiple modules may be launched
    utils::generateDrivableArea(
      output.path, expanded_lanes, di.enable_expanding_hatched_road_markings,
      di.enable_expanding_intersection_areas || use_intersection_areas_,
      di.enable_expanding_freespace_areas, data,
      is_driving_forward);
  }

  // extract obstacles from drivable area
  utils::extractObstaclesFromDrivableArea(output.path, di.obstacles);
}

void SourcePlannerPipeline::updateCurrentRouteLanelet(
  const std::shared_ptr<PlannerData> & data, const bool is_any_approved_module_running)
{
  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  const auto p = data->parameters;

  constexpr double extra_margin = 10.0;
  const auto backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);

  lanelet::ConstLanelet closest_lane{};

  if (route_handler->getClosestRouteLaneletFromLanelet(
        pose, current_route_lanelet_->value(), &closest_lane, p.ego_nearest_dist_threshold,
        p.ego_nearest_yaw_threshold)) {
    *current_route_lanelet_ = closest_lane;
    return;
  }

  const auto lanelet_sequence = route_handler->getLaneletSequence(
    current_route_lanelet_->value(), pose, backward_length, p.forward_path_length);

  auto opt = autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint(
    lanelet_sequence, pose, p.ego_nearest_dist_threshold, p.ego_nearest_yaw_threshold);
  if (opt.has_value()) {
    closest_lane = *opt;
    *current_route_lanelet_ = closest_lane;
  } else if (const auto opt_constraint =
               experimental::lanelet2_utils::get_closest_lanelet(lanelet_sequence, pose);
             opt_constraint) {
    *current_route_lanelet_ = opt_constraint.value();
  } else if (!is_any_approved_module_running) {
    resetCurrentRouteLanelet(data);
  }
}

BehaviorModuleOutput SourcePlannerPipeline::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  const auto reference_path = utils::getReferencePath(current_route_lanelet_->value(), data);

  if (reference_path.path.points.empty()) {
    ROS_WARN_THROTTLE(5.0,
      "Empty reference path detected. Using last valid reference path if available.");

    if (last_valid_reference_path_.has_value()) {
      return last_valid_reference_path_.value();
    }

    ROS_WARN_THROTTLE(5.0, "No valid previous reference path available. Creating empty path.");
    return BehaviorModuleOutput{};
  }

  last_valid_reference_path_ = reference_path;

  publishDebugRootReferencePath(reference_path);
  return reference_path;
}

void SourcePlannerPipeline::publishDebugRootReferencePath(
  const BehaviorModuleOutput & reference_path) const
{
  using visualization_msgs::Marker;
  visualization_msgs::MarkerArray array;
  Marker m = autoware_utils::create_default_marker(
    "map", ros::Time::now(), "root_reference_path", 0UL, Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(1.0, 1.0, 1.0),
    autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0));
  for (const auto & p : reference_path.path.points) m.points.push_back(p.point.pose.position);
  array.markers.push_back(m);
  m.points.clear();
  m.id = 1UL;
  for (const auto & p : current_route_lanelet_->value().polygon3d().basicPolygon()) {
    geometry_msgs::Point point; point.x = p.x(); point.y = p.y(); point.z = p.z();
    m.points.push_back(point);
  }
  array.markers.push_back(m);
  root_reference_publisher_.publish(array);
}


}
