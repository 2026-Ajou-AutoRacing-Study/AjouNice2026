#include "autoware/planning_validator_intersection_collision_checker/ego_trajectory_core_ros1.hpp"

#include <gtest/gtest.h>

TEST(EgoTrajectoryCore, UsesSourceFrontAndRearVehicleOffsetsForTimesAndIndices)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    trajectory.points.push_back(point);
  }
  geometry_msgs::Pose ego_pose;
  ego_pose.orientation.w = 1.0;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.wheel_base_m = 2.0;
  vehicle.front_overhang_m = 1.0;
  vehicle.rear_overhang_m = 1.0;

  const auto ego = autoware::planning_validator::intersection_collision_checker::make_ego_trajectory(
    trajectory, ego_pose, vehicle);
  ASSERT_TRUE(ego.has_value());
  // Source findNearestIndex keeps the first point on an equal-distance tie.
  EXPECT_EQ(ego->front_index, 1U);
  EXPECT_EQ(ego->back_index, 0U);
  EXPECT_EQ(ego->front_trajectory.front().time_from_start.sec, 0);
  EXPECT_GT(ego->front_trajectory.back().time_from_start.sec, 0);
  EXPECT_GT(ego->back_trajectory.back().time_from_start.sec, 0);
}

TEST(EgoTrajectoryCore, RejectsEmptyTrajectory)
{
  autoware_planning_msgs::Trajectory empty;
  geometry_msgs::Pose pose;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  EXPECT_FALSE(
    autoware::planning_validator::intersection_collision_checker::make_ego_trajectory(
      empty, pose, vehicle).has_value());
}
