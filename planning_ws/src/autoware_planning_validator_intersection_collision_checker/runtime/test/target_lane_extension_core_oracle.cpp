#include "autoware/planning_validator_intersection_collision_checker/target_lane_extension_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>

TEST(TargetLaneExtensionCore, ExtendsLoadedTargetThroughActualRouteHandlerPredecessors)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  autoware::route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelet target;
  for (const auto & lane : map->laneletLayer) {
    if (!route_handler.getPreviousLanelets(lane).empty()) {
      target = lane;
      break;
    }
  }
  ASSERT_NE(target.id(), lanelet::InvalId);
  const auto & point = target.centerline().front();
  geometry_msgs::Pose overlap;
  overlap.position.x = point.x();
  overlap.position.y = point.y();
  overlap.position.z = point.z();
  overlap.orientation.w = 1.0;

  const auto single =
    autoware::planning_validator::intersection_collision_checker::extend_target_lanelet(
      route_handler, target, overlap, 0.0);
  ASSERT_EQ(single.size(), 1U);
  EXPECT_EQ(single.front().id(), target.id());

  const auto extended =
    autoware::planning_validator::intersection_collision_checker::extend_target_lanelet(
      route_handler, target, overlap, 1e6);
  ASSERT_GE(extended.size(), 2U);
  EXPECT_EQ(extended.back().id(), target.id());
  EXPECT_NE(extended.front().id(), target.id());
}
