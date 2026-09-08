#include "autoware/planning_validator/rear_object_assembly_core.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace
{
autoware_planning_msgs::LaneletRoute route_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = lane.id();
  primitive.primitive_type = "lanelet";
  autoware_planning_msgs::LaneletSegment segment;
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  autoware_planning_msgs::LaneletRoute route;
  route.segments.push_back(segment);
  return route;
}

autoware::planning_validator::RearDetectionArea full_lane_area(const lanelet::ConstLanelet & lane)
{
  autoware::planning_validator::RearDetectionArea area;
  area.lanes.push_back(lane);
  for (const auto & point : lane.polygon3d()) area.polygon.push_back(point.basicPoint());
  return area;
}
}  // namespace

TEST(RearObjectAssemblyCore, SelectsOnlyDetectionPointsAndProjectsOnRouteCenterline)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRoute(route_for(lane));
  ASSERT_TRUE(route_handler->isHandlerReady());
  ASSERT_FALSE(lane.centerline().empty());

  const auto center = lane.centerline()[lane.centerline().size() / 2U];
  autoware::planning_validator::RearPointCloud points;
  points.push_back({static_cast<float>(center.x()), static_cast<float>(center.y()), static_cast<float>(center.z())});
  points.push_back({100000.0F, 100000.0F, 0.0F});
  const auto object = autoware::planning_validator::assemble_rear_object_from_detection_areas(
    points, {full_lane_area(lane)}, route_handler);
  ASSERT_TRUE(object.has_value());
  EXPECT_GE(object->absolute_distance, 0.0);
  EXPECT_EQ(object->furthest_lane_id, lane.id());
}

TEST(RearObjectAssemblyCore, FailsClosedWithoutRouteHandlerOrArea)
{
  autoware::planning_validator::RearPointCloud points;
  points.push_back({0.0F, 0.0F, 0.0F});
  EXPECT_FALSE(autoware::planning_validator::assemble_rear_object_from_detection_areas(
    points, {}, nullptr).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
