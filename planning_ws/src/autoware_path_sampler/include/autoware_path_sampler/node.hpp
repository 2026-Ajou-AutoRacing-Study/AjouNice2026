// Copyright 2023 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef AUTOWARE_PATH_SAMPLER__NODE_HPP_
#define AUTOWARE_PATH_SAMPLER__NODE_HPP_

#include "autoware_path_sampler/common_structs.hpp"
#include "autoware_path_sampler/parameters.hpp"
#include "autoware_path_sampler/type_alias.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_sampler_common/structures.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <ros/ros.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_sampler
{
class PathSampler
{
public:
  PathSampler(ros::NodeHandle node, ros::NodeHandle private_node);

private:
  template <typename T> T requiredParam(const std::string & name) const;

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};
  mutable DebugData debug_data_{};
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_{nullptr};
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};
  Parameters params_{};
  size_t debug_id_ = 0;
  std::optional<autoware::sampler_common::Path> prev_path_;

  ros::Publisher traj_pub_;
  ros::Publisher virtual_wall_pub_;
  ros::Subscriber path_sub_;
  autoware_utils::InterProcessPollingSubscriber<Odometry>::SharedPtr odom_sub_;
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects>::SharedPtr objects_sub_;
  ros::Publisher debug_markers_pub_;
  ros::Publisher debug_calculation_time_pub_;

  void onPath(const Path::ConstPtr & path);
  bool isDataReady(const Path & path, const Odometry::ConstPtr & ego_state);
  PlannerData createPlannerData(const Path & path, const Odometry & ego_state) const;
  std::vector<TrajectoryPoint> generateTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_traj_points) const;
  void resetPreviousData();
  autoware::sampler_common::State getPlanningState(
    autoware::sampler_common::State & state,
    const autoware::sampler_common::transform::Spline2D & path_spline) const;
  void copyZ(const std::vector<TrajectoryPoint> & from, std::vector<TrajectoryPoint> & to);
  void copyVelocity(
    const std::vector<TrajectoryPoint> & from, std::vector<TrajectoryPoint> & to,
    const geometry_msgs::Pose & ego_pose);
  autoware::sampler_common::Path generatePath(const PlannerData & planner_data);
  std::vector<autoware::sampler_common::Path> generateCandidatesFromPreviousPath(
    const PlannerData & planner_data,
    const autoware::sampler_common::transform::Spline2D & path_spline);
  std::vector<TrajectoryPoint> generateTrajectoryPoints(const PlannerData & planner_data);
  void publishVirtualWall(const geometry_msgs::Pose & stop_pose) const;
  void publishDebugMarker(const std::vector<TrajectoryPoint> & traj_points) const;
};
}  // namespace autoware::path_sampler

#endif
