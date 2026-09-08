#include <autoware/deprecated/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware/deprecated/boundary_departure_checker/uncrossable_boundary_checker.hpp>
#include <autoware/deprecated/boundary_departure_checker/utils.hpp>

#include <gtest/gtest.h>

namespace
{
autoware_internal_planning_msgs::PathWithLaneId make_path(const double y)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const double x : {0.0, 2.0, 4.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.position.y = y;
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  return path;
}

lanelet::Lanelet make_lane(const lanelet::Id id, const double left, const double right)
{
  lanelet::Point3d p1{lanelet::InvalId, -10.0, left, 0.0};
  lanelet::Point3d p2{lanelet::InvalId, 10.0, left, 0.0};
  lanelet::Point3d p3{lanelet::InvalId, 10.0, right, 0.0};
  lanelet::Point3d p4{lanelet::InvalId, -10.0, right, 0.0};
  return lanelet::Lanelet{id, lanelet::LineString3d{lanelet::InvalId, {p1, p2}},
    lanelet::LineString3d{lanelet::InvalId, {p4, p3}}};
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3, 0.2, 2.5, 1.5, 1.0, 1.0, 0.25, 0.25, 1.5, 0.5);
}

lanelet::LaneletMapPtr make_map(const std::initializer_list<lanelet::Lanelet> & lanelets)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : lanelets) map->add(lanelet);
  return map;
}
}  // namespace

TEST(BoundaryDepartureCheckerRos1, KeepsVehicleFootprintInsideLane)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker{{}, make_vehicle()};
  EXPECT_FALSE(checker.checkPathWillLeaveLane({make_lane(1, 2.0, -2.0)}, make_path(0.0)));
}

TEST(BoundaryDepartureCheckerRos1, RejectsVehicleFootprintOutsideLane)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker{{}, make_vehicle()};
  EXPECT_TRUE(checker.checkPathWillLeaveLane({make_lane(1, 2.0, -2.0)}, make_path(2.2)));
}

TEST(BoundaryDepartureCheckerRos1, AcceptsFootprintSpanningAdjacentLanelets)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker{{}, make_vehicle()};
  EXPECT_FALSE(checker.checkPathWillLeaveLane(
    {make_lane(1, 2.0, 0.0), make_lane(2, 0.0, -2.0)}, make_path(0.0)));
}

TEST(BoundaryDepartureCheckerRos1, FailsClosedWithoutVehicleGeometry)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker;
  EXPECT_TRUE(checker.checkPathWillLeaveLane({make_lane(1, 2.0, -2.0)}, make_path(0.0)));
}

TEST(BoundaryDepartureCheckerRos1, UpdateFailsClosedForIncompleteAsynchronousInput)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker{{}, make_vehicle()};
  const auto output = checker.update({});
  EXPECT_TRUE(output.will_leave_lane);
  EXPECT_TRUE(output.is_out_of_lane);
  EXPECT_TRUE(output.will_cross_boundary);
  EXPECT_TRUE(output.resampled_trajectory.empty());
}

TEST(BoundaryDepartureCheckerRos1, ReusesCachedFusedLaneletsAndCropsBeforeEndIndex)
{
  autoware::boundary_departure_checker::BoundaryDepartureChecker checker{{}, make_vehicle()};
  const auto map = make_map({make_lane(1, 2.0, -2.0)});
  std::vector<lanelet::Id> ids;
  std::optional<autoware_utils_geometry::Polygon2d> fused;
  const auto inside = make_path(0.0);
  EXPECT_FALSE(checker.checkPathWillLeaveLane(map, inside, ids, fused));
  ASSERT_TRUE(fused);
  ASSERT_EQ(ids.size(), 1U);
  EXPECT_FALSE(checker.checkPathWillLeaveLane(map, inside, ids, fused));
  const auto cropped = checker.cropPointsOutsideOfLanes(map, make_path(2.2), 0, ids, fused);
  EXPECT_EQ(cropped.points.size(), 2U);
}

TEST(BoundaryDepartureCheckerRos1, IndexesOnlyConfiguredUncrossableBoundaries)
{
  lanelet::Point3d first{lanelet::InvalId, 0.0, 0.0, 0.0};
  lanelet::Point3d second{lanelet::InvalId, 4.0, 0.0, 0.0};
  lanelet::Point3d third{lanelet::InvalId, 8.0, 0.0, 0.0};
  lanelet::LineString3d road_border{lanelet::utils::getId(), {first, second}};
  road_border.attributes()[lanelet::AttributeName::Type] = "road_border";
  lanelet::LineString3d divider{lanelet::utils::getId(), {second, third}};
  divider.attributes()[lanelet::AttributeName::Type] = "lane_divider";
  lanelet::LaneletMap map;
  map.add(road_border);
  map.add(divider);

  const auto tree = autoware::boundary_departure_checker::utils::build_uncrossable_boundaries_rtree(
    map, {"road_border"});
  EXPECT_EQ(tree.size(), 1U);
}

TEST(BoundaryDepartureCheckerRos1, ConstructsAndQueriesUncrossableChecker)
{
  lanelet::Point3d first{lanelet::InvalId, 0.0, 0.0, 0.0};
  lanelet::Point3d second{lanelet::InvalId, 4.0, 0.0, 0.0};
  lanelet::LineString3d road_border{lanelet::utils::getId(), {first, second}};
  road_border.attributes()[lanelet::AttributeName::Type] = "road_border";
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road_border);

  autoware::boundary_departure_checker::Param param;
  param.boundary_types_to_detect = {"road_border"};
  autoware::boundary_departure_checker::UncrossableBoundaryDepartureChecker checker{
    std::make_shared<autoware::boundary_departure_checker::ros1_compat::Clock>(), map, make_vehicle(), param};

  const auto tree = checker.build_uncrossable_boundaries_tree(map);
  ASSERT_TRUE(tree);
  EXPECT_EQ(tree->size(), 1U);
}

int main(int argc, char ** argv)
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
