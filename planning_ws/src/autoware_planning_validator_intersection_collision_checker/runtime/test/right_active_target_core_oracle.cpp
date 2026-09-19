#include "autoware/planning_validator_intersection_collision_checker/right_active_target_core_ros1.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>

namespace
{
lanelet::Lanelet make_lane(
  const lanelet::Id base, const lanelet::Points3d & left, const lanelet::Points3d & right,
  const std::string & turn = "")
{
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
    point.pose.position.y = x / 10.0;
    point.pose.orientation.w = 1.0;
    point.time_from_start.sec = static_cast<int32_t>(x / 2.0);
    result.points.push_back(point);
  }
  return result;
}

autoware::planning_validator::intersection_collision_checker_ros1::Params params()
{
  using Params = autoware::planning_validator::intersection_collision_checker_ros1::Params;
  Params p{};
  p.detection_range = 100.0;
  p.right_turn.check_crossing_lanes = true;
  p.right_turn.check_turning_lanes = true;
  p.right_turn.check_traffic_signal = true;
  p.right_turn.crossing_lane_angle_th = 0.78539816339;
  return p;
}
}  // namespace

TEST(RightActiveTargetCore, BuildsTargetOnlyWithinHorizonAndRemovesPriority)
{
  const auto ego_turn = make_lane(
    100,
    {lanelet::Point3d(100, 0.0, 1.0, 0.0), lanelet::Point3d(101, 20.0, 3.0, 0.0)},
    {lanelet::Point3d(102, 0.0, -1.0, 0.0), lanelet::Point3d(103, 20.0, 1.0, 0.0)}, "right");
  const auto target = make_lane(
    200,
    {lanelet::Point3d(200, 9.0, -10.0, 0.0), lanelet::Point3d(201, 9.0, 10.0, 0.0)},
    {lanelet::Point3d(202, 11.0, -10.0, 0.0), lanelet::Point3d(203, 11.0, 10.0, 0.0)});
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ego_turn);
  map->add(target);
  autoware::route_handler::RouteHandler route_handler(map);
  autoware::planning_validator::intersection_collision_checker::LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {ego_turn};
  ego_lanes.trajectory_lanelets = {ego_turn};

  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::make_right_active_targets(
      route_handler, trajectory(), ego_lanes, 0U, 3.0, params(), false).empty());
  const auto active =
    autoware::planning_validator::intersection_collision_checker::make_right_active_targets(
      route_handler, trajectory(), ego_lanes, 0U, 6.0, params(), false);
  ASSERT_EQ(active.size(), 1U);
  EXPECT_EQ(active.front().id, target.id());
  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::make_right_active_targets(
      route_handler, trajectory(), ego_lanes, 0U, 6.0, params(), true).empty());
}
