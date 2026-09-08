#include "autoware/planning_validator/rear_current_lane_core.hpp"

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

geometry_msgs::Pose middle_pose(const lanelet::ConstLanelet & lane)
{
  const auto centerline = lane.centerline();
  const auto point = centerline[centerline.size() / 2U];
  geometry_msgs::Pose pose;
  pose.position.x = point.x();
  pose.position.y = point.y();
  pose.position.z = point.z();
  pose.orientation.w = 1.0;
  return pose;
}
}  // namespace

TEST(RearCurrentLaneCore, UsesRouteBoundLaneSequenceForLoadedMap)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto lane = *map->laneletLayer.begin();
  const auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  route_handler->setRoute(route_for(lane));
  ASSERT_TRUE(route_handler->isHandlerReady());
  const auto lanes = autoware::planning_validator::get_rear_current_lanes(
    route_handler, middle_pose(lane), 100.0, 100.0);
  ASSERT_FALSE(lanes.empty());
  EXPECT_EQ(lanes.front().id(), lane.id());
}

TEST(RearCurrentLaneCore, FailsClosedWithoutRouteBoundLane)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto lane = *map->laneletLayer.begin();
  const auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map);
  EXPECT_TRUE(autoware::planning_validator::get_rear_current_lanes(
    route_handler, middle_pose(lane), 100.0, 100.0).empty());
  EXPECT_TRUE(autoware::planning_validator::get_rear_current_lanes(
    nullptr, middle_pose(lane), 100.0, 100.0).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
