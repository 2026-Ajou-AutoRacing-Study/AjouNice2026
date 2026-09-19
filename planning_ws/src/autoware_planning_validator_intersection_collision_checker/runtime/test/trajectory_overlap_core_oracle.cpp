#include "autoware/planning_validator_intersection_collision_checker/trajectory_overlap_core_ros1.hpp"

#include <gtest/gtest.h>

namespace
{
lanelet::ConstLanelet vertical_target_lane()
{
  lanelet::Points3d left{
    lanelet::Point3d(1, 9.0, -2.0, 0.0), lanelet::Point3d(2, 9.0, 2.0, 0.0)};
  lanelet::Points3d right{
    lanelet::Point3d(3, 11.0, -2.0, 0.0), lanelet::Point3d(4, 11.0, 2.0, 0.0)};
  lanelet::Lanelet lane(10, lanelet::LineString3d(11, left), lanelet::LineString3d(12, right));
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return lane;
}

autoware_planning_msgs::Trajectory sampled_trajectory()
{
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto x : {0.0, 8.0, 9.5, 10.5, 12.0, 20.0}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.time_from_start.sec = static_cast<int32_t>(x / 2.0);
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(TrajectoryOverlapCore, ProducesSortedSourceShapedIndicesAndTimes)
{
  const auto result =
    autoware::planning_validator::intersection_collision_checker::trajectory_overlap_for_lanelet(
      vertical_target_lane(), sampled_trajectory(), 1U);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first_index, 2U);
  EXPECT_EQ(result->last_index, 3U);
  EXPECT_DOUBLE_EQ(result->first_time_seconds, 4.0);
  EXPECT_DOUBLE_EQ(result->last_time_seconds, 5.0);
}

TEST(TrajectoryOverlapCore, RejectsOverlapAlreadyBehindEgoFrontIndex)
{
  EXPECT_FALSE(
    autoware::planning_validator::intersection_collision_checker::trajectory_overlap_for_lanelet(
      vertical_target_lane(), sampled_trajectory(), 4U).has_value());
}
