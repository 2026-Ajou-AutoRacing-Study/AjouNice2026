#include "autoware/planning_validator_intersection_collision_checker/point_in_lane_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

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
}  // namespace

TEST(PointInLaneCore, KeepsOnlyPointInsideActualRightTurnLanelet)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_INTERSECTION_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto lane = map->laneletLayer.get(3008);
  ASSERT_TRUE(lane.hasAttribute("turn_direction"));
  ASSERT_EQ(lane.attribute("turn_direction").value(), "right");
  autoware::route_handler::RouteHandler route_handler(map);
  route_handler.setRoute(route_for(lane));
  ASSERT_TRUE(route_handler.isHandlerReady());
  const auto center = lane.centerline()[lane.centerline().size() / 2U];
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::PointXYZ inside;
  inside.x = static_cast<float>(center.x()); inside.y = static_cast<float>(center.y()); inside.z = static_cast<float>(center.z());
  points.push_back(inside);
  pcl::PointXYZ outside = inside; outside.x += 100.0F; points.push_back(outside);
  const auto filtered = autoware::planning_validator::intersection_collision_checker_ros1::points_within_lanelet(points, lane);
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_FLOAT_EQ(filtered.front().x, inside.x);
}
