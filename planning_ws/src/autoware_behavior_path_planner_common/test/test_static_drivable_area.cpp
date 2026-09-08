#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <gtest/gtest.h>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <cmath>
#include <lanelet2_core/primitives/Lanelet.h>

namespace utils = autoware::behavior_path_planner::utils;
namespace bpp = autoware::behavior_path_planner;

namespace {
lanelet::ConstLanelet combinationLane(const lanelet::Id id, const double y)
{
  lanelet::Points3d left{lanelet::Point3d(id * 10 + 1, 0, y + 1, 0),
                        lanelet::Point3d(id * 10 + 2, 10, y + 1, 0)};
  lanelet::Points3d right{lanelet::Point3d(id * 10 + 3, 0, y - 1, 0),
                         lanelet::Point3d(id * 10 + 4, 10, y - 1, 0)};
  return lanelet::Lanelet(id, lanelet::LineString3d(id * 10 + 5, left),
                         lanelet::LineString3d(id * 10 + 6, right));
}
}

TEST(GoalBoundaryDistance, VehicleCornersAndMissingBoundary)
{
  geometry_msgs::Pose pose;
  pose.position.x = 5.0;
  pose.orientation.w = 1.0;
  const lanelet::ConstLanelets lanes{combinationLane(400, 0)};
  const auto left = utils::getSignedDistanceFromBoundary(lanes, 1.0, 2.0, 1.0, pose, true);
  const auto right = utils::getSignedDistanceFromBoundary(lanes, 1.0, 2.0, 1.0, pose, false);
  ASSERT_TRUE(left);
  ASSERT_TRUE(right);
  EXPECT_NEAR(*left, -0.5, 1e-9);
  EXPECT_NEAR(*right, 0.5, 1e-9);
  EXPECT_FALSE(utils::getSignedDistanceFromBoundary({}, 1.0, 2.0, 1.0, pose, true));
  pose.position.x = 20.0;
  EXPECT_FALSE(utils::getSignedDistanceFromBoundary(lanes, 1.0, 2.0, 1.0, pose, true));
}

TEST(DrivableLaneCombination, MergesSharedLaneAndAppendsRemainingSection)
{
  const auto a = combinationLane(100, 0), b = combinationLane(101, 2), c = combinationLane(102, 4);
  const auto result = utils::combineDrivableLanes({{a, a, {}}}, {{a, b, {}}, {c, c, {}}});
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].right_lane.id(), a.id());
  EXPECT_EQ(result[0].left_lane.id(), b.id());
  EXPECT_TRUE(result[0].middle_lanes.empty());
  EXPECT_EQ(result[1].right_lane.id(), c.id());
}

TEST(DrivableLaneCombination, PreservesMiddleLanesDuringFurtherExpansion)
{
  const auto a = combinationLane(200, 0), b = combinationLane(201, 2);
  const auto c = combinationLane(202, 4), d = combinationLane(203, 6);
  const auto result = utils::combineDrivableLanes({{a, c, {b}}}, {{b, d, {c}}});
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0].right_lane.id(), a.id());
  EXPECT_EQ(result[0].left_lane.id(), d.id());
  ASSERT_EQ(result[0].middle_lanes.size(), 2U);
  EXPECT_EQ(result[0].middle_lanes[0].id(), b.id());
  EXPECT_EQ(result[0].middle_lanes[1].id(), c.id());
}

TEST(DrivableLaneCombination, AreaUsesMergedLanesAndOriginalExpansionState)
{
  const auto a = combinationLane(300, 0), b = combinationLane(301, 2);
  bpp::DrivableAreaInfo first, second;
  first.drivable_lanes = {{a, a, {}}};
  second.drivable_lanes = {{a, b, {}}};
  first.obstacles.resize(1);
  second.obstacles.resize(2);
  first.drivable_margin = 1;
  second.drivable_margin = 2;
  first.is_already_expanded = true;
  first.enable_expanding_hatched_road_markings = true;
  second.enable_expanding_intersection_areas = true;
  const auto result = utils::combineDrivableAreaInfo(first, second);
  ASSERT_EQ(result.drivable_lanes.size(), 1U);
  EXPECT_EQ(result.drivable_lanes[0].left_lane.id(), b.id());
  EXPECT_EQ(result.obstacles.size(), 3U);
  EXPECT_EQ(result.drivable_margin, 2);
  EXPECT_TRUE(result.enable_expanding_hatched_road_markings);
  EXPECT_TRUE(result.enable_expanding_intersection_areas);
  EXPECT_FALSE(result.is_already_expanded);
}

namespace
{
geometry_msgs::Point point(const double x, const double y)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  return p;
}

autoware_internal_planning_msgs::PathWithLaneId makeStraightPath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const double x : {0.0, 5.0, 10.0, 15.0, 20.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId path_point;
    path_point.point.pose.position = point(x, 0.0);
    path.points.push_back(path_point);
    path.right_bound.push_back(point(x, -2.0));
    path.left_bound.push_back(point(x, 2.0));
  }
  return path;
}

bpp::DrivableAreaInfo::Obstacle makeRightObstacle(
  const double min_x, const double max_x, const double outer_y = -3.0,
  const double inner_y = -1.0)
{
  bpp::DrivableAreaInfo::Obstacle obstacle;
  obstacle.is_left = false;
  obstacle.pose.position = point((min_x + max_x) / 2.0, -2.0);
  auto & ring = obstacle.poly.outer();
  ring.emplace_back(min_x, outer_y);
  ring.emplace_back(min_x, inner_y);
  ring.emplace_back(max_x, inner_y);
  ring.emplace_back(max_x, outer_y);
  ring.emplace_back(min_x, outer_y);
  return obstacle;
}

bool hasPointNear(
  const std::vector<geometry_msgs::Point> & points, const double x, const double y)
{
  return std::any_of(points.begin(), points.end(), [&](const auto & p) {
    return std::abs(p.x - x) < 1e-3 && std::abs(p.y - y) < 1e-3;
  });
}
}  // namespace

TEST(StaticDrivableArea, GeneratesSingleAndEmptyLaneSets)
{
  EXPECT_TRUE(utils::generateDrivableLanes({}).empty());
  lanelet::Lanelet lane;
  lane.setId(42);
  const auto output = utils::generateDrivableLanes({lane});
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().left_lane.id(), 42);
  EXPECT_EQ(output.front().right_lane.id(), 42);
}

TEST(StaticDrivableArea, ClipsBoundaryCrossingObstacleAtBothIntersections)
{
  auto path = makeStraightPath();
  utils::extractObstaclesFromDrivableArea(path, {makeRightObstacle(4.0, 8.0)});

  ASSERT_GE(path.right_bound.size(), 4U);
  EXPECT_TRUE(hasPointNear(path.right_bound, 4.0, -2.0));
  EXPECT_TRUE(hasPointNear(path.right_bound, 8.0, -2.0));
  EXPECT_TRUE(hasPointNear(path.right_bound, 4.0, -1.0));
  EXPECT_TRUE(hasPointNear(path.right_bound, 8.0, -1.0));
}

TEST(StaticDrivableArea, PreservesSourceEquivalentOverlappingObstacleStitching)
{
  auto path = makeStraightPath();
  utils::extractObstaclesFromDrivableArea(
    path, {makeRightObstacle(4.0, 10.0), makeRightObstacle(8.0, 14.0)});

  // Pin the upstream stitching contract, including its repeated seam points.
  // This is deliberately not normalized in the ROS1 port: downstream code
  // receives the same boundary sequence as the ROS2 implementation.
  const std::vector<geometry_msgs::Point> expected = {
    point(0.0, -2.0), point(4.0, -2.0), point(4.0, -1.0), point(8.0, -1.0),
    point(8.0, -1.0), point(10.0, -1.0), point(10.0, -2.0), point(10.0, -2.0),
    point(15.0, -2.0), point(20.0, -2.0)};
  ASSERT_EQ(path.right_bound.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(path.right_bound.at(i).x, expected.at(i).x, 1e-3);
    EXPECT_NEAR(path.right_bound.at(i).y, expected.at(i).y, 1e-3);
  }
}
