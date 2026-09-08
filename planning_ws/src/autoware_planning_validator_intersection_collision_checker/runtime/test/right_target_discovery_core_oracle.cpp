#include "autoware/planning_validator_intersection_collision_checker/right_target_discovery_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/right_target_filter_core_ros1.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <gtest/gtest.h>

namespace
{
lanelet::Lanelet make_lane(
  const lanelet::Id base, const lanelet::Points3d & left, const lanelet::Points3d & right,
  const std::string & turn_direction = "")
{
  lanelet::Lanelet lane(
    base + 30, lanelet::LineString3d(base + 10, left), lanelet::LineString3d(base + 20, right));
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  if (!turn_direction.empty()) lane.attributes()["turn_direction"] = turn_direction;
  return lane;
}

autoware_planning_msgs::Trajectory trajectory_along(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::Trajectory trajectory;
  for (const auto & point : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint item;
    item.pose.position.x = point.x();
    item.pose.position.y = point.y();
    item.pose.position.z = point.z();
    item.pose.orientation.w = 1.0;
    trajectory.points.push_back(item);
  }
  return trajectory;
}
}  // namespace

TEST(RightTargetDiscoveryCore, FindsOnlyOverlappingNonRouteRoadCandidate)
{
  const auto ego_turn = make_lane(
    100,
    {lanelet::Point3d(100, 0.0, 1.0, 0.0), lanelet::Point3d(101, 20.0, 3.0, 0.0)},
    {lanelet::Point3d(102, 0.0, -1.0, 0.0), lanelet::Point3d(103, 20.0, 1.0, 0.0)}, "right");
  const auto crossing_target = make_lane(
    200,
    {lanelet::Point3d(200, 9.0, -10.0, 0.0), lanelet::Point3d(201, 9.0, 10.0, 0.0)},
    {lanelet::Point3d(202, 11.0, -10.0, 0.0), lanelet::Point3d(203, 11.0, 10.0, 0.0)});
  const auto non_overlapping = make_lane(
    300,
    {lanelet::Point3d(300, 39.0, -10.0, 0.0), lanelet::Point3d(301, 39.0, 10.0, 0.0)},
    {lanelet::Point3d(302, 41.0, -10.0, 0.0), lanelet::Point3d(303, 41.0, 10.0, 0.0)});
  autoware::planning_validator::intersection_collision_checker::LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {ego_turn};
  ego_lanes.trajectory_lanelets = {ego_turn};
  lanelet::LaneletMap map;
  map.add(ego_turn);
  map.add(crossing_target);
  map.add(non_overlapping);
  EXPECT_EQ(
    map.laneletLayer.search(
      lanelet::BoundingBox2d(lanelet::BasicPoint2d(0.0, 0.0), lanelet::BasicPoint2d(20.0, 2.0)))
      .size(),
    2U);
  const auto searched = map.laneletLayer.search(
    lanelet::BoundingBox2d(lanelet::BasicPoint2d(0.0, 0.0), lanelet::BasicPoint2d(20.0, 2.0)));
  const auto mapped_target = std::find_if(
    searched.begin(), searched.end(),
    [&](const lanelet::ConstLanelet & lane) { return lane.id() == crossing_target.id(); });
  ASSERT_NE(mapped_target, searched.end());
  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::eligible_right_target(
      *mapped_target, ego_lanes, true, true, 0.78539816339));

  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::eligible_right_target(
      crossing_target, ego_lanes, true, true, 0.78539816339));
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : crossing_target.polygon2d()) polygon.outer().emplace_back(point.x(), point.y());
  boost::geometry::correct(polygon);
  autoware_utils_geometry::LineString2d line;
  for (const auto & point : trajectory_along(ego_turn).points) {
    line.emplace_back(point.pose.position.x, point.pose.position.y);
  }
  EXPECT_TRUE(boost::geometry::intersects(polygon, line));
  const auto targets =
    autoware::planning_validator::intersection_collision_checker::discover_right_target_lanelets(
      map, trajectory_along(ego_turn), ego_lanes, true, true, 0.78539816339);
  ASSERT_EQ(targets.size(), 1U);
  EXPECT_EQ(targets.front().id(), crossing_target.id());
}

TEST(RightTargetDiscoveryCore, EnforcesRightTurnAndCrossingPoliciesBeforeDiscovery)
{
  const auto ego_turn = make_lane(
    100,
    {lanelet::Point3d(100, 0.0, 1.0, 0.0), lanelet::Point3d(101, 20.0, 3.0, 0.0)},
    {lanelet::Point3d(102, 0.0, -1.0, 0.0), lanelet::Point3d(103, 20.0, 1.0, 0.0)}, "right");
  const auto crossing_target = make_lane(
    200,
    {lanelet::Point3d(200, 9.0, -10.0, 0.0), lanelet::Point3d(201, 9.0, 10.0, 0.0)},
    {lanelet::Point3d(202, 11.0, -10.0, 0.0), lanelet::Point3d(203, 11.0, 10.0, 0.0)});
  lanelet::LaneletMap map;
  map.add(ego_turn);
  map.add(crossing_target);
  autoware::planning_validator::intersection_collision_checker::LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {ego_turn};
  ego_lanes.trajectory_lanelets = {ego_turn};

  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::discover_right_target_lanelets(
      map, trajectory_along(ego_turn), ego_lanes, true, false, 0.78539816339).empty());
  const auto not_right = make_lane(
    400,
    {lanelet::Point3d(400, 0.0, 1.0, 0.0), lanelet::Point3d(401, 20.0, 1.0, 0.0)},
    {lanelet::Point3d(402, 0.0, -1.0, 0.0), lanelet::Point3d(403, 20.0, -1.0, 0.0)}, "left");
  ego_lanes.turn_lanelets = {not_right};
  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::discover_right_target_lanelets(
      map, trajectory_along(ego_turn), ego_lanes, true, true, 0.78539816339).empty());
}
