#include "autoware/planning_validator/rear_lanelet_detection_core.hpp"
#include "autoware/planning_validator/rear_polygon_selection_core.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <memory>

namespace
{
lanelet::ConstLanelet source_lanelet(
  const double left_y = 2.0, const double right_y = -2.0, const lanelet::Id base_id = 1)
{
  lanelet::Points3d left{lanelet::Point3d(base_id, 0.0, left_y, 0.0),
    lanelet::Point3d(base_id + 1, 10.0, left_y, 0.0)};
  lanelet::Points3d right{lanelet::Point3d(base_id + 2, 0.0, right_y, 0.0),
    lanelet::Point3d(base_id + 3, 10.0, right_y, 0.0)};
  return lanelet::Lanelet(
    base_id + 30, lanelet::LineString3d(base_id + 10, left),
    lanelet::LineString3d(base_id + 20, right));
}
}  // namespace

TEST(RearLaneletDetectionCore, MakesSourceHalfLanesAndArcLengthDetectionPolygon)
{
  const auto lane = source_lanelet();
  const auto left_half = autoware::planning_validator::make_rear_half_lanelet(lane, false, 0.5, 0.2);
  const auto right_half = autoware::planning_validator::make_rear_half_lanelet(lane, true, 0.5, 0.2);
  ASSERT_GE(left_half.leftBound().size(), 2U);
  ASSERT_GE(right_half.rightBound().size(), 2U);
  EXPECT_GT(left_half.leftBound().front().y(), 2.0);
  EXPECT_LT(right_half.rightBound().front().y(), -2.0);

  geometry_msgs::Pose ego;
  ego.position.x = 5.0;
  ego.orientation.w = 1.0;
  const auto polygon = autoware::planning_validator::make_rear_detection_polygon(
    {lane}, ego, 2.0, 3.0);
  EXPECT_GE(polygon.size(), 4U);
}

TEST(RearLaneletDetectionCore, RemovesCurrentDrivingLaneFromDetectionArea)
{
  const auto lane = source_lanelet();
  const auto detection_lane = source_lanelet(4.0, 0.0, 100);
  std::vector<autoware::planning_validator::RearDetectionArea> areas{
    {detection_lane.polygon3d().basicPolygon(), {detection_lane}}};
  autoware::planning_validator::cut_rear_detection_areas_by_current_lanes({lane}, areas);
  ASSERT_FALSE(areas.front().polygon.empty());
  for (const auto & point : areas.front().polygon) EXPECT_GE(point.y(), 2.0);
}

TEST(RearLaneletDetectionCore, FailsClosedWithoutRouteHandlerOrTargetLanes)
{
  const auto lane = source_lanelet();
  EXPECT_TRUE(autoware::planning_validator::make_rear_previous_detection_areas(
    {lane}, {}, 0.0, 10.0, nullptr, 0.2, 0.2).empty());
  EXPECT_TRUE(autoware::planning_validator::make_rear_previous_detection_areas(
    {lane}, {lane}, 0.0, 10.0, nullptr, 0.2, 0.2).empty());
}

TEST(RearLaneletDetectionCore, BuildsPredecessorDetectionAreaFromLoadedRouteHandlerMap)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);

  lanelet::ConstLanelet target;
  bool found_predecessor = false;
  for (const auto & lane : map->laneletLayer) {
    if (!route_handler->getPreviousLanelets(lane).empty()) {
      target = lane;
      found_predecessor = true;
      break;
    }
  }
  ASSERT_TRUE(found_predecessor);

  const auto areas = autoware::planning_validator::make_rear_previous_detection_areas(
    {}, {target}, 0.0, 5.0, route_handler, 0.2, 0.2);
  ASSERT_FALSE(areas.empty());
  EXPECT_FALSE(areas.front().polygon.empty());
  EXPECT_FALSE(areas.front().lanes.empty());
}

TEST(RearLaneletDetectionCore, RoutesLoadedPredecessorAreaIntoPointcloudSelection)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  lanelet::ConstLanelet target;
  for (const auto & lane : map->laneletLayer) {
    if (!route_handler->getPreviousLanelets(lane).empty()) {
      target = lane;
      break;
    }
  }
  ASSERT_NE(target.id(), lanelet::InvalId);
  const auto areas = autoware::planning_validator::make_rear_previous_detection_areas(
    {}, {target}, 0.0, 5.0, route_handler, 0.2, 0.2);
  ASSERT_FALSE(areas.empty());

  using Point2d = boost::geometry::model::d2::point_xy<double>;
  boost::geometry::model::polygon<Point2d> polygon;
  autoware::planning_validator::RearDetectionPolygon selection_polygon;
  for (const auto & point : areas.front().polygon) {
    polygon.outer().emplace_back(point.x(), point.y());
    selection_polygon.push_back({static_cast<float>(point.x()), static_cast<float>(point.y()),
      static_cast<float>(point.z())});
  }
  polygon.outer().push_back(polygon.outer().front());
  Point2d inside;
  boost::geometry::centroid(polygon, inside);
  ASSERT_TRUE(boost::geometry::within(inside, polygon));

  autoware::planning_validator::RearPointCloud cloud;
  cloud.push_back({static_cast<float>(inside.x()), static_cast<float>(inside.y()), 0.0F});
  const auto selected = autoware::planning_validator::select_rear_points_in_detection_polygons(
    {selection_polygon}, cloud);
  ASSERT_EQ(selected->size(), 1U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
