#include "autoware/planning_validator_intersection_collision_checker/active_target_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::Trajectory trajectory_for(const lanelet::ConstLanelet & lane)
{
  autoware_planning_msgs::Trajectory trajectory;
  int32_t seconds = 0;
  for (const auto & source : lane.centerline()) {
    autoware_planning_msgs::TrajectoryPoint point;
    point.pose.position.x = source.x();
    point.pose.position.y = source.y();
    point.pose.position.z = source.z();
    point.pose.orientation.w = 1.0;
    point.time_from_start.sec = seconds++;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(ActiveTargetCore, AssemblesFutureOverlapAndExtendedLaneletsOrRemovesPriority)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  autoware::route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelet target;
  for (const auto & lane : map->laneletLayer) {
    if (!route_handler.getPreviousLanelets(lane).empty() && lane.centerline().size() >= 2U) {
      target = lane;
      break;
    }
  }
  ASSERT_NE(target.id(), lanelet::InvalId);
  const auto trajectory = trajectory_for(target);
  const auto active =
    autoware::planning_validator::intersection_collision_checker::make_active_target(
      route_handler, target, trajectory, 0U, 1e6, false);
  ASSERT_TRUE(active.has_value());
  EXPECT_EQ(active->id, target.id());
  EXPECT_EQ(active->lanelets.back().id(), target.id());
  EXPECT_NEAR(active->overlap_pose.position.x, target.centerline().front().x(), 1e-6);
  EXPECT_LE(active->overlap.first_index, active->overlap.last_index);
  EXPECT_FALSE(
    autoware::planning_validator::intersection_collision_checker::make_active_target(
      route_handler, target, trajectory, 0U, 1e6, true).has_value());
}
