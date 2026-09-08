#include "autoware/planning_validator_intersection_collision_checker/active_turn_collision_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/arc_distance_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/collision_decision_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/object_tracking_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/point_in_lane_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/pointcloud_cluster_core_ros1.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::planning_validator::intersection_collision_checker
{
ActiveTurnCollisionResult ActiveTurnCollisionCore::evaluate(
  const pcl::PointCloud<pcl::PointXYZ> & map_points,
  const std::vector<ActiveTarget> & active_targets, const ros::Time & stamp,
  const double ego_base_z, const double ego_velocity,
  const intersection_collision_checker_ros1::Params & params)
{
  ActiveTurnCollisionResult result;
  const auto & p = params.pointcloud;
  const intersection_collision_checker_ros1::ClusterParameters cluster_params{
    p.clustering.tolerance, p.clustering.min_height, p.clustering.min_size, p.clustering.max_size};

  for (const auto & target : active_targets) {
    pcl::PointCloud<pcl::PointXYZ> inside;
    for (const auto & lane : target.lanelets) {
      const auto lane_points =
        intersection_collision_checker_ros1::points_within_lanelet(map_points, lane);
      inside += lane_points;
    }
    const auto clustered = intersection_collision_checker_ros1::cluster_points_above_height(
      inside, ego_base_z, cluster_params);
    const auto approaching = intersection_collision_checker_ros1::nearest_approaching_point(
      clustered, target.lanelets, target.overlap_pose);
    if (!approaching.has_value()) continue;

    auto [it, inserted] = tracks_.try_emplace(target.id, stamp, approaching->distance_to_overlap);
    if (inserted) continue;  // Source first observation only initializes history.

    auto & track = it->second;
    const double dt = (stamp - track.history.last_update_time()).toSec();
    track.history.update(
      stamp, approaching->distance_to_overlap, p.velocity_estimation.max_velocity,
      p.velocity_estimation.max_acceleration, params.close_distance_th);
    const bool reliable =
      track.history.track_duration() > p.velocity_estimation.observation_time;
    track.history.set_reliable(reliable);
    if (dt > 0.0) {
      track.moving_time = track.history.velocity() < params.filter.min_velocity
        ? 0.0
        : track.moving_time + dt;
    }
    const double delay_compensated = std::max(
      0.0, track.history.distance_to_overlap() -
      track.history.velocity() * p.latency);
    constexpr double zero_velocity_threshold = 0.1;
    const double ttc = std::invoke([&]() {
      if (track.history.velocity() < -zero_velocity_threshold) {
        return std::numeric_limits<double>::infinity();
      }
      if (delay_compensated < params.close_distance_th) return 0.0;
      return delay_compensated /
        std::max(track.history.velocity(), zero_velocity_threshold);
    });
    const intersection_collision_checker_ros1::CollisionObjectState object{
      reliable, track.moving_time > params.filter.moving_time, ttc};
    if (intersection_collision_checker_ros1::is_colliding(
          object, {target.overlap.first_time_seconds, target.overlap.last_time_seconds},
          ego_velocity, params)) {
      result.safe = false;
      result.collision_lane_ids.push_back(target.id);
    }
  }

  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if ((stamp - it->second.history.last_update_time()).toSec() >
        p.velocity_estimation.max_history_time) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
  return result;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
