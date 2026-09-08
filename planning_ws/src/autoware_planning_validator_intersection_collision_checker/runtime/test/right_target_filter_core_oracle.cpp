#include "autoware/planning_validator_intersection_collision_checker/right_target_filter_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <algorithm>

namespace
{
lanelet::ConstLanelet make_lane(
  const lanelet::Id base, const lanelet::Points3d & left, const lanelet::Points3d & right,
  const std::string & turn_direction = "")
{
  lanelet::Lanelet lane(
    base + 30, lanelet::LineString3d(base + 10, left), lanelet::LineString3d(base + 20, right));
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  if (!turn_direction.empty()) lane.attributes()["turn_direction"] = turn_direction;
  return lane;
}

lanelet::ConstLanelet straight_lane(
  const lanelet::Id base, const double x = 0.0, const std::string & turn_direction = "")
{
  return make_lane(
    base,
    {lanelet::Point3d(base, x - 1.0, 0.0, 0.0), lanelet::Point3d(base + 1, x - 1.0, 20.0, 0.0)},
    {lanelet::Point3d(base + 2, x + 1.0, 0.0, 0.0), lanelet::Point3d(base + 3, x + 1.0, 20.0, 0.0)},
    turn_direction);
}
}  // namespace

TEST(RightTargetFilterCore, ExcludesActualRouteLaneButKeepsAnotherRoadLane)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_INTERSECTION_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto route_lane = map->laneletLayer.get(3008);
  ASSERT_EQ(route_lane.attribute("turn_direction").value(), "right");

  autoware::planning_validator::intersection_collision_checker::LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {route_lane};
  ego_lanes.trajectory_lanelets = {route_lane};
  EXPECT_FALSE(autoware::planning_validator::intersection_collision_checker::eligible_right_target(
    route_lane, ego_lanes, true, true, 0.78539816339));

  // The frozen intersection map's other lanelets are not tagged `road`; use
  // an explicit road lane to isolate this source policy from map metadata.
  const auto other = straight_lane(1000, 50.0);
  EXPECT_TRUE(autoware::planning_validator::intersection_collision_checker::eligible_right_target(
    other, ego_lanes, true, true, 0.78539816339));
}

TEST(RightTargetFilterCore, AppliesTurnAndFirstSegmentCrossingPolicies)
{
  using autoware::planning_validator::intersection_collision_checker::LaneletSelection;
  using autoware::planning_validator::intersection_collision_checker::eligible_right_target;
  const auto turning_candidate = straight_lane(100, 0.0, "left");
  EXPECT_FALSE(eligible_right_target(turning_candidate, LaneletSelection{}, false, true, 0.78539816339));

  const auto crossing_candidate = straight_lane(200, 0.0);
  const auto curved_turn = make_lane(
    300,
    {lanelet::Point3d(300, -10.0, 1.0, 0.0), lanelet::Point3d(301, 0.0, 1.0, 0.0),
      lanelet::Point3d(302, 0.0, 11.0, 0.0)},
    {lanelet::Point3d(303, -10.0, -1.0, 0.0), lanelet::Point3d(304, 0.0, -1.0, 0.0),
      lanelet::Point3d(305, 0.0, 9.0, 0.0)},
    "right");
  LaneletSelection ego_lanes;
  ego_lanes.turn_lanelets = {curved_turn};
  EXPECT_FALSE(eligible_right_target(crossing_candidate, ego_lanes, true, false, 0.78539816339));
  EXPECT_TRUE(eligible_right_target(crossing_candidate, ego_lanes, true, true, 0.78539816339));
}
