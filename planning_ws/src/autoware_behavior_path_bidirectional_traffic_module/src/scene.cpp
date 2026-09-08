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
#include "autoware/behavior_path_bidirectional_traffic_module/scene.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/keep_left.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"
#include "autoware/route_handler/route_handler.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <ros/ros.h>

#include <autoware_internal_planning_msgs/PathPointWithLaneId.h>
#include <autoware_perception_msgs/PredictedObjects.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
namespace autoware::behavior_path_planner
{

using autoware_internal_planning_msgs::PathWithLaneId;

BidirectionalTrafficModule::BidirectionalTrafficModule(
  std::string_view name,
  const std::shared_ptr<BidirectionalTrafficModuleParameters> & parameters,
  const SceneModuleInterface::RTCInterfaceMap & rtc_interface_ptr_map,
  SceneModuleInterface::MarkerInterfaceMap &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{name.data(), rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_(parameters),
  has_trajectory_bidirectional_lane_overlap_(false)
{
}

void BidirectionalTrafficModule::initialize_all_member_variables()
{
  bidirectional_lane_where_ego_was_ = std::nullopt;
  has_trajectory_bidirectional_lane_overlap_ = false;
  give_way_ = std::nullopt;
  oncoming_cars_ = std::nullopt;
  stop_pose_ = std::nullopt;
}

CandidateOutput BidirectionalTrafficModule::planCandidate() const
{
  return CandidateOutput{};
}

BehaviorModuleOutput BidirectionalTrafficModule::plan()
{
  using autoware_internal_planning_msgs::PathPointWithLaneId;

  BehaviorModuleOutput module_output = getPreviousModuleOutput();

  PathWithLaneId previous_path = module_output.path;

  auto trajectory = experimental::trajectory::Trajectory<
                      autoware_internal_planning_msgs::PathPointWithLaneId>::Builder{}
                      .build(previous_path.points);

  if (!trajectory) {
    ROS_ERROR("Failed to build trajectory in BidirectionalTrafficModule::plan");
    return module_output;
  }

  *trajectory = shift_trajectory_for_keep_left(
    *trajectory, get_all_bidirectional_lanes_in_map(), *parameters_,
    EgoParameters::from_planner_data(*planner_data_));

  if (give_way_ && oncoming_cars_) {
    const double speed = planner_data_->self_odometry->twist.twist.linear.x;
    *trajectory = give_way_->modify_trajectory(*trajectory, *oncoming_cars_, getEgoPose(), speed);
  }

  module_output.path.points = trajectory->restore();

  return module_output;
}

BehaviorModuleOutput BidirectionalTrafficModule::planWaitingApproval()
{
  return plan();
}

bool BidirectionalTrafficModule::isExecutionRequested() const
{
  return has_trajectory_bidirectional_lane_overlap_;
}

bool BidirectionalTrafficModule::isExecutionReady() const
{
  return true;
}

void BidirectionalTrafficModule::processOnEntry()
{
}

void BidirectionalTrafficModule::processOnExit()
{
}

void BidirectionalTrafficModule::updateData()
{
  PathWithLaneId previous_path = getPreviousModuleOutput().path;

  auto trajectory = experimental::trajectory::Trajectory<
                      autoware_internal_planning_msgs::PathPointWithLaneId>::Builder{}
                      .build(previous_path.points);

  if (!trajectory) {
    ROS_WARN_THROTTLE(5.0, "Failed to build trajectory in BidirectionalTrafficModule::updateData");
    return;
  }

  // Check if the trajectory has bidirectional lanelet's id
  has_trajectory_bidirectional_lane_overlap_ =
    has_common_part(get_all_bidirectional_lane_ids_in_map(), trajectory->get_contained_lane_ids());

  if (!has_trajectory_bidirectional_lane_overlap_) {
    initialize_all_member_variables();
    return;
  }

  // Check if the ego vehicle is running on the bidirectional lane
  auto bidirectional_lane_where_ego_is = get_bidirectional_lanelets_where_ego_is(
    getEgoPose(), EgoParameters::from_planner_data(*planner_data_),
    get_all_bidirectional_lanes_in_map());

  if (
    !bidirectional_lane_where_ego_was_.has_value() && bidirectional_lane_where_ego_is.has_value()) {
    give_way_ = GiveWay(
      *bidirectional_lane_where_ego_is, EgoParameters::from_planner_data(*planner_data_),
      *parameters_, [this](geometry_msgs::Pose pose) { stop_pose_ = PoseWithDetail(pose); });

    oncoming_cars_ = OncomingCars(
      *bidirectional_lane_where_ego_is, 0.1,
      [this](std::string_view msg) { ROS_INFO_STREAM(msg); });

    ROS_INFO("Entered Bidirectional Lane");
  }
  if (
    bidirectional_lane_where_ego_was_.has_value() && !bidirectional_lane_where_ego_is.has_value()) {
    give_way_ = std::nullopt;
    ROS_INFO("Exited Bidirectional Lane");
  }
  bidirectional_lane_where_ego_was_ = bidirectional_lane_where_ego_is;

  if (!bidirectional_lane_where_ego_is.has_value()) {
    return;
  }
  if (oncoming_cars_.has_value() && planner_data_->dynamic_object) {
    oncoming_cars_->update(*planner_data_->dynamic_object, getEgoPose(), ros::Time::now());
  } else if (oncoming_cars_.has_value()) {
    ROS_WARN_THROTTLE(
      5.0, "Bidirectional traffic requires PredictedObjects; oncoming-car update is held.");
  }
}

std::vector<ConnectedBidirectionalLanelets::SharedConstPtr>
BidirectionalTrafficModule::get_all_bidirectional_lanes_in_map() const
{
  if (!all_bidirectional_lanes_in_map_.has_value()) {
    const lanelet::LaneletMap & map = *planner_data_->route_handler->getLaneletMapPtr();
    auto get_next_lanelets = [this](const lanelet::ConstLanelet & lanelet) {
      return planner_data_->route_handler->getNextLanelets(lanelet);
    };
    auto get_prev_lanelets = [this](const lanelet::ConstLanelet & lanelet) {
      return planner_data_->route_handler->getPreviousLanelets(lanelet);
    };
    all_bidirectional_lanes_in_map_ =
      ConnectedBidirectionalLanelets::search_bidirectional_lanes_on_map(
        map, get_next_lanelets, get_prev_lanelets);
    ROS_INFO("Found %zu bidirectional lanes", all_bidirectional_lanes_in_map_->size() / 2);
  }
  return all_bidirectional_lanes_in_map_.value();
}

lanelet::Ids BidirectionalTrafficModule::get_all_bidirectional_lane_ids_in_map() const
{
  lanelet::Ids all_bidirectional_lane_ids;
  for (const auto & lane : get_all_bidirectional_lanes_in_map()) {
    for (const auto & lanelet : lane->get_lanelets()) {
      all_bidirectional_lane_ids.emplace_back(lanelet.id());
    }
  }
  return all_bidirectional_lane_ids;
}

bool BidirectionalTrafficModule::canTransitSuccessState()
{
  return !has_trajectory_bidirectional_lane_overlap_;
}

bool BidirectionalTrafficModule::canTransitFailureState()
{
  return false;
}

}  // namespace autoware::behavior_path_planner
