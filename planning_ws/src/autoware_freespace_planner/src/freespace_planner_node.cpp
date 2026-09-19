// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware_freespace_planner/freespace_planner_node.hpp"

#include "autoware_freespace_planner/utils.hpp"
#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
: nh_(nh), pnh_(pnh)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.th_obstacle_time_sec = declare_parameter<double>("th_obstacle_time_sec");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
  }

  // set vehicle_info
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(pnh_).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base_length = vehicle_info.wheel_base_m;
    vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Planning
  initializePlanningAlgorithm();

  if (node_param_.update_rate <= 0 || node_param_.th_stopped_time_sec <= 0)
    throw std::invalid_argument("Freespace rate and stopped-time window must be positive");
  route_sub_ = pnh_.subscribe("input/route", 1, &FreespacePlannerNode::onRoute, this);
  occupancy_grid_sub_ = pnh_.subscribe<OccupancyGrid>("input/occupancy_grid", 1,
    [this](const OccupancyGrid::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_); pending_grid_ = msg;
    });
  scenario_sub_ = pnh_.subscribe<Scenario>("input/scenario", 1,
    [this](const Scenario::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_); pending_scenario_ = msg;
    });
  odom_sub_ = pnh_.subscribe<Odometry>("input/odometry", 100,
    [this](const Odometry::ConstPtr & msg) {
      std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
      pending_odometry_.push_back(msg);
      if (pending_odometry_.size() > 100) pending_odometry_.pop_front();
    });
  trajectory_pub_ = pnh_.advertise<Trajectory>("output/trajectory", 1, true);
  debug_pose_array_pub_ = pnh_.advertise<PoseArray>("debug/pose_array", 1, true);
  debug_partial_pose_array_pub_ = pnh_.advertise<PoseArray>("debug/partial_pose_array", 1, true);
  parking_state_pub_ = nh_.advertise<std_msgs::Bool>("is_completed", 1, true);
  processing_time_pub_ = pnh_.advertise<autoware_internal_debug_msgs::Float64Stamped>("debug/processing_time_ms", 1);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  timer_ = nh_.createTimer(ros::Duration(1.0 / node_param_.update_rate),
    [this](const ros::TimerEvent &) { onTimer(); });

}

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.direction_change_weight = declare_parameter<double>("direction_change_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio");
  p.turning_steps = declare_parameter<int>("turning_steps");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found && checkCurrentTrajectoryCollision()) {
    ROS_DEBUG( "Found obstacle");
    return true;
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out = utils::calc_distance_2d(trajectory_, current_pose_.pose) >
                               node_param_.th_course_out_distance_m;
    if (is_course_out) {
      ROS_INFO( "Course out");
      return true;
    }
  }

  return false;
}

bool FreespacePlannerNode::checkCurrentTrajectoryCollision()
{
  algo_->setMap(*occupancy_grid_);

  const size_t nearest_index_partial = autoware::motion_utils::findNearestIndex(
    partial_trajectory_.points, current_pose_.pose.position);
  const size_t end_index_partial = partial_trajectory_.points.size() - 1;
  const auto forward_trajectory = utils::get_partial_trajectory(
    partial_trajectory_, nearest_index_partial, end_index_partial, ros::Time::now());

  const bool is_obs_found =
    algo_->hasObstacleOnTrajectory(utils::trajectory_to_pose_array(forward_trajectory));

  if (!is_obs_found) {
    obs_found_time_ = {};
    return false;
  }

  if (!obs_found_time_) obs_found_time_ = ros::Time::now();

  return (ros::Time::now() - obs_found_time_.get()).toSec() > node_param_.th_obstacle_time_sec;
}

void FreespacePlannerNode::updateTargetIndex()
{
  if (!utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return;
  }

  const auto is_near_target = utils::is_near_target(
    trajectory_.points.at(target_index_).pose, current_pose_.pose,
    node_param_.th_arrived_distance_m);

  if (!is_near_target) return;

  const auto new_target_index =
    utils::get_next_target_index(trajectory_.points.size(), reversing_indices_, target_index_);

  if (new_target_index == target_index_) {
    // Finished publishing all partial trajectories
    is_completed_ = true;
    ROS_INFO_THROTTLE(1.0, "Freespace planning completed");
    std_msgs::Bool is_completed_msg;
    is_completed_msg.data = is_completed_;
    parking_state_pub_.publish(is_completed_msg);
  } else {
    // Switch to next partial trajectory
    prev_target_index_ = target_index_;
    target_index_ = new_target_index;
  }
}

void FreespacePlannerNode::onRoute(const LaneletRoute::ConstPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  route_ = msg;

  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;

  is_new_parking_cycle_ = true;

  reset();
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      ros::Time(msg->header.stamp) - ros::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.toSec() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

void FreespacePlannerNode::updateData()
{
  occupancy_grid_ = pending_grid_;

  {
    auto msgs = std::move(pending_odometry_);
    pending_odometry_.clear();
    for (const auto & msg : msgs) {
      onOdometry(msg);
    }
  }
}

bool FreespacePlannerNode::isDataReady()
{
  bool is_ready = true;

  if (!route_) {
    ROS_INFO_THROTTLE(5.0, "Waiting for route data.");
    is_ready = false;
  }

  if (!occupancy_grid_) {
    ROS_INFO_THROTTLE(5.0, "Waiting for occupancy grid.");
    is_ready = false;
  }

  if (!odom_) {
    ROS_INFO_THROTTLE(5.0, "Waiting for odometry.");
    is_ready = false;
  }

  return is_ready;
}

void FreespacePlannerNode::onTimer()
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  scenario_ = pending_scenario_;
  if (!utils::is_active(scenario_)) {
    reset();
    return;
  }

  updateData();

  if (!isDataReady()) {
    return;
  }

  if (is_completed_) {
    partial_trajectory_.header = odom_->header;
    const auto stop_trajectory = utils::create_stop_trajectory(partial_trajectory_);
    trajectory_pub_.publish(stop_trajectory);
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return;
  }

  // Must stop before replanning any new trajectory
  const bool is_reset_required = !reset_in_progress_ && isPlanRequired();
  if (is_reset_required) {
    // Stop before planning new trajectory, except in a new parking cycle as the vehicle already
    // stops.
    if (!is_new_parking_cycle_) {
      const auto stop_trajectory = partial_trajectory_.points.empty()
                                     ? utils::create_stop_trajectory(current_pose_, ros::Time::now())
                                     : utils::create_stop_trajectory(partial_trajectory_);
      trajectory_pub_.publish(stop_trajectory);
      debug_pose_array_pub_.publish(utils::trajectory_to_pose_array(stop_trajectory));
      debug_partial_pose_array_pub_.publish(utils::trajectory_to_pose_array(stop_trajectory));
    }

    reset();

    reset_in_progress_ = true;
  }

  if (reset_in_progress_) {
    const auto is_ego_stopped =
      utils::is_stopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
    if (is_ego_stopped) {
      // Plan new trajectory
      planTrajectory();
      reset_in_progress_ = false;
    } else {
      // Will keep current stop trajectory
      ROS_WARN_THROTTLE(1.0,
        "Waiting for the vehicle to stop before generating a new trajectory.");
    }
  }

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    is_new_parking_cycle_ = false;
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ =
    utils::get_partial_trajectory(trajectory_, prev_target_index_, target_index_, ros::Time::now());

  // Publish messages
  trajectory_pub_.publish(partial_trajectory_);
  debug_pose_array_pub_.publish(utils::trajectory_to_pose_array(trajectory_));
  debug_partial_pose_array_pub_.publish(utils::trajectory_to_pose_array(partial_trajectory_));

  is_new_parking_cycle_ = false;

  // Publish ProcessingTime
  autoware_internal_debug_msgs::Float64Stamped processing_time_msg;
  const auto processing_stamp = ros::Time::now();
  processing_time_msg.stamp.sec = processing_stamp.sec;
  processing_time_msg.stamp.nanosec = processing_stamp.nsec;
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_.publish(processing_time_msg);
}

void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return;
  }

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_pose_in_costmap_frame = utils::transform_pose(
    current_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = utils::transform_pose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute planning
  const ros::Time start = ros::Time::now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  } catch (const std::exception & e) {
    error_msg = e.what();
  }
  const ros::Time end = ros::Time::now();

  ROS_DEBUG( "Freespace planning: %f [s]", (end - start).toSec());

  if (result) {
    ROS_DEBUG( "Found goal!");
    trajectory_ = utils::create_trajectory(
      current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
    reversing_indices_ = utils::get_reversing_indices(trajectory_);
    prev_target_index_ = 0;
    target_index_ = utils::get_next_target_index(
      trajectory_.points.size(), reversing_indices_, prev_target_index_);
  } else {
    ROS_INFO( "Can't find goal: %s", error_msg.c_str());
    reset();
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  std_msgs::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_.publish(is_completed_msg);
  obs_found_time_ = {};
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(from, to, ros::Time(0), ros::Duration(1.0));
  } catch (const tf2::TransformException & ex) {
    ROS_ERROR( "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, pnh_);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, pnh_);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  ROS_INFO_STREAM( "initialize planning algorithm: " << algo_name);
}
}  // namespace autoware::freespace_planner
