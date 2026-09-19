#include "autoware/planning_validator_intersection_collision_checker/left_target_discovery_core_ros1.hpp"

#include <gtest/gtest.h>

namespace
{
lanelet::ConstLanelet vertical_lane(const lanelet::Id base, const double x, const std::string & turn = "")
{
  lanelet::Points3d left{
    lanelet::Point3d(base, x - 1.0, -2.0, 0.0), lanelet::Point3d(base + 1, x - 1.0, 2.0, 0.0)};
  lanelet::Points3d right{
    lanelet::Point3d(base + 2, x + 1.0, -2.0, 0.0), lanelet::Point3d(base + 3, x + 1.0, 2.0, 0.0)};
  lanelet::Lanelet lane(base + 30, lanelet::LineString3d(base + 10, left), lanelet::LineString3d(base + 20, right));
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  if (!turn.empty()) lane.attributes()["turn_direction"] = turn;
  return lane;
}

autoware_planning_msgs::Trajectory trajectory()
{
  autoware_planning_msgs::Trajectory result;
  for (const auto x : {0.0, 8.0, 9.5, 10.5, 12.0, 20.0}) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    result.points.push_back(point);
  }
  return result;
}
}  // namespace

TEST(LeftTargetDiscoveryCore, ExcludesLastTurnAndNonOverlappingPredecessors)
{
  const auto last_turn = vertical_lane(100, 2.0, "left");
  const auto overlapping = vertical_lane(200, 10.0);
  const auto non_overlapping = vertical_lane(300, 30.0);
  const auto targets =
    autoware::planning_validator::intersection_collision_checker::discover_left_target_lanelets(
      {last_turn, overlapping, non_overlapping}, last_turn.id(), trajectory(), 1U, true);
  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets.front().id(), overlapping.id());
}

TEST(LeftTargetDiscoveryCore, AppliesSourceTurningPolicy)
{
  const auto target_turn = vertical_lane(200, 10.0, "right");
  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::discover_left_target_lanelets(
      {target_turn}, 999, trajectory(), 1U, false).empty());
  EXPECT_EQ(
    autoware::planning_validator::intersection_collision_checker::discover_left_target_lanelets(
      {target_turn}, 999, trajectory(), 1U, true).front().id(),
    target_turn.id());
}
