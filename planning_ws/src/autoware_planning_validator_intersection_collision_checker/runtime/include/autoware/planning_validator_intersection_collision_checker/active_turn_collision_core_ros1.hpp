#pragma once

#include "autoware/planning_validator_intersection_collision_checker/active_target_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/object_tracking_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/parameters_ros1.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>

#include <unordered_map>
#include <vector>

namespace autoware::planning_validator::intersection_collision_checker
{
struct ActiveTurnCollisionResult
{
  bool safe{true};
  std::vector<lanelet::Id> collision_lane_ids;
};

// Stateful rendering of the active-target portion of source check_collision:
// point-in-lane, clustering, arc association, per-lane history and the
// source collision predicate. Pointcloud TF/cropping remains one layer above.
class ActiveTurnCollisionCore
{
public:
  ActiveTurnCollisionResult evaluate(
    const pcl::PointCloud<pcl::PointXYZ> & map_points,
    const std::vector<ActiveTarget> & active_targets, const ros::Time & stamp,
    double ego_base_z, double ego_velocity,
    const intersection_collision_checker_ros1::Params & params);

private:
  struct Track
  {
    explicit Track(const ros::Time & stamp, const double distance)
    : history(stamp, distance) {}

    intersection_collision_checker_ros1::ObjectTrackingCore history;
    double moving_time{0.0};
  };
  std::unordered_map<lanelet::Id, Track> tracks_;
};
}  // namespace autoware::planning_validator::intersection_collision_checker
