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
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#include <autoware_freespace_planner/utils.hpp>


#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <mutex>
#include <stdexcept>
namespace autoware::route_handler { class RouteHandler; }
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <ros/ros.h>

#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class TestFreespacePlanner;

namespace autoware::freespace_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::RRTStarParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware_internal_planning_msgs::Scenario;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Trajectory;
using geometry_msgs::PoseArray;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::Twist;
using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double th_obstacle_time_sec;
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class FreespacePlannerNode
{
public:
  explicit FreespacePlannerNode(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);

private:
  // ros
  ros::Publisher trajectory_pub_;
  ros::Publisher debug_pose_array_pub_;
  ros::Publisher debug_partial_pose_array_pub_;
  ros::Publisher parking_state_pub_;
  ros::Publisher processing_time_pub_;

  ros::Subscriber route_sub_;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber occupancy_grid_sub_, scenario_sub_, odom_sub_;
  ros::Timer timer_;
  std::recursive_mutex callback_mutex_;
  OccupancyGrid::ConstPtr pending_grid_;
  Scenario::ConstPtr pending_scenario_;
  std::deque<Odometry::ConstPtr> pending_odometry_;
  template<class T> T declare_parameter(const std::string & name) {
    T value{};
    if (!pnh_.getParam(name, value)) throw std::runtime_error("Missing freespace parameter: " + name);
    return value;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;
  bool reset_in_progress_ = false;
  bool is_new_parking_cycle_ = true;
  boost::optional<ros::Time> obs_found_time_;

  LaneletRoute::ConstPtr route_;
  OccupancyGrid::ConstPtr occupancy_grid_;
  Scenario::ConstPtr scenario_;
  Odometry::ConstPtr odom_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  std::deque<Odometry::ConstPtr> odom_buffer_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onRoute(const LaneletRoute::ConstPtr msg);
  void onOdometry(const Odometry::ConstPtr msg);

  void onTimer();
  void updateData();
  void reset();
  void planTrajectory();
  void initializePlanningAlgorithm();
  bool isDataReady();

  /**
   * @brief Checks if a new trajectory planning is required.
   * @details A new trajectory planning is required if:
   *           - Current trajectory points are empty, or
   *           - Current trajectory collides with an object, or
   *           - Ego deviates from current trajectory
   * @return true if any of the conditions are met.
   */
  bool isPlanRequired();

  /**
   * @brief Sets the target index along the current trajectory points
   * @details if Ego is stopped AND is near the current target index along the trajectory,
   *          then will get the next target index along the trajectory.
   *          If the new target index is the same as the current target index, then
   *          is_complete_ is set to true, and will publish is_completed_msg.
   *          Otherwise will update prev_target_index_ and target_index_, to continue
   *          following the trajectory.
   */
  void updateTargetIndex();

  /**
   * @brief Checks if current trajectory is colliding with an object.
   * @details Will check if an obstacle exists along the current trajectory,
   *          if there is no obstacle along the current trajectory, will reset obs_found_time_.
   *          If an obstacle exists and the variable obs_found_time_ is not initialized,
   *          will initialize with the current time.
   * @return true if there is an obstacle along current trajectory, AND duration since
   *         obs_found_time_ exceeds the parameter th_obstacle_time_sec
   */
  bool checkCurrentTrajectoryCollision();

  TransformStamped getTransform(const std::string & from, const std::string & to);

  // ROS1 node exposes standard roscpp get/set_logger_level services.

  friend class ::TestFreespacePlanner;
};
}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
