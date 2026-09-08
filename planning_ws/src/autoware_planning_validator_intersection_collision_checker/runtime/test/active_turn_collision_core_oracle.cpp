#include "autoware/planning_validator_intersection_collision_checker/active_turn_collision_core_ros1.hpp"

#include <gtest/gtest.h>

namespace
{
lanelet::ConstLanelet target_lane()
{
  lanelet::Points3d left{
    lanelet::Point3d(1, 0.0, 1.0, 0.0), lanelet::Point3d(2, 10.0, 1.0, 0.0)};
  lanelet::Points3d right{
    lanelet::Point3d(3, 0.0, -1.0, 0.0), lanelet::Point3d(4, 10.0, -1.0, 0.0)};
  lanelet::Lanelet lane(10, lanelet::LineString3d(11, left), lanelet::LineString3d(12, right));
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return lane;
}

pcl::PointCloud<pcl::PointXYZ> object_at(const float x)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({x - 0.05F, -0.15F, 1.0F});
  cloud.push_back({x + 0.05F, -0.15F, 1.0F});
  cloud.push_back({x, 0.15F, 1.0F});
  return cloud;
}

autoware::planning_validator::intersection_collision_checker_ros1::Params params()
{
  using Params = autoware::planning_validator::intersection_collision_checker_ros1::Params;
  Params p{};
  p.ttc_threshold = 1.0;
  p.ego_deceleration = 1.0;
  p.close_distance_th = 0.1;
  p.close_time_th = 1.0;
  p.filter.min_velocity = 0.0;
  p.filter.moving_time = 0.0;
  p.pointcloud.latency = 0.0;
  p.pointcloud.clustering = {0.5, 0.1, 3, 30};
  p.pointcloud.velocity_estimation = {10.0, 10.0, 0.5, 10.0, 10};
  return p;
}
}  // namespace

TEST(ActiveTurnCollisionCore, RequiresHistoryThenDetectsReliableApproachingCollision)
{
  const auto lane = target_lane();
  autoware::planning_validator::intersection_collision_checker::ActiveTarget target;
  target.id = lane.id();
  target.lanelets = {lane};
  target.overlap_pose.position.x = 9.0;
  target.overlap_pose.orientation.w = 1.0;
  target.overlap = {0U, 0U, 7.0, 8.0};
  autoware::planning_validator::intersection_collision_checker::ActiveTurnCollisionCore core;

  EXPECT_TRUE(core.evaluate(object_at(1.0F), {target}, ros::Time(1.0), 0.0, 1.0, params()).safe);
  const auto result = core.evaluate(object_at(2.0F), {target}, ros::Time(2.0), 0.0, 1.0, params());
  EXPECT_FALSE(result.safe);
  ASSERT_EQ(result.collision_lane_ids.size(), 1U);
  EXPECT_EQ(result.collision_lane_ids.front(), lane.id());
}
